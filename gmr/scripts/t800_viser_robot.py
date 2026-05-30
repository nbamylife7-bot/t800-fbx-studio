"""T800 robot rendering for viser (white / textured / transparent skins).

Textured skins use MuJoCo-compiled mesh vertices and per-face UV indices.
Textures are tinted with each geom's `rgba` and a small ambient floor so the
robot stays the same gray-blue as in MuJoCo instead of turning pitch black.
"""

from __future__ import annotations

from typing import Callable, Literal, Optional

import mujoco as mj
import numpy as np
import trimesh
import viser
from PIL import Image
from scipy.spatial.transform import Rotation as R
from trimesh.visual.material import PBRMaterial

import scripts.t800_foot_postprocess as foot_pp  # noqa: E402

SkinMode = Literal["white", "full", "transparent"]
ProgressCallback = Callable[[float, str], None]

WHITE_RGB = (245, 245, 245)
TRANSPARENT_ALPHA = 0.35
# MuJoCo tints textured geoms with geom rgba; viser PBR needs matte non-metallic albedo.
PBR_METALLIC = 0.04
PBR_ROUGHNESS = 0.82
SHADOW_FILL = 0.38
ALBEDO_LIFT = 1.12

# Straight standing pose: feet together, arms at sides (used before any motion is loaded).
_STANDING_JOINTS: dict[str, float] = {
    "J00_HIP_PITCH_L": -0.03,
    "J06_HIP_PITCH_R": -0.03,
    "J01_HIP_ROLL_L": 0.05,
    "J07_HIP_ROLL_R": -0.05,
    "J03_KNEE_PITCH_L": 0.08,
    "J09_KNEE_PITCH_R": 0.08,
    "J14_SHOULDER_ROLL_L": 0.38,
    "J21_SHOULDER_ROLL_R": -0.38,
    "J16_ELBOW_PITCH_L": -0.05,
    "J23_ELBOW_PITCH_R": -0.05,
}


def _set_joint_qpos(model: mj.MjModel, qpos: np.ndarray, joint_name: str, value: float) -> None:
    joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        return
    addr = int(model.jnt_qposadr[joint_id])
    if bool(model.jnt_limited[joint_id]):
        lo, hi = model.jnt_range[joint_id]
        value = float(np.clip(value, lo + 1e-4, hi - 1e-4))
    qpos[addr] = value


def t800_standing_qpos(model: mj.MjModel) -> np.ndarray:
    """Neutral upright T800 pose: feet together, arms along the body, on the ground."""
    qpos = np.zeros(model.nq, dtype=np.float64)
    qpos[3] = 1.0
    for name, value in _STANDING_JOINTS.items():
        _set_joint_qpos(model, qpos, name, value)
    qpos = foot_pp.postprocess_robot_qpos_feet(model, qpos, flatten=True)
    foot_pp.snap_robot_feet_to_ground(model, qpos)
    return qpos


def _mat_to_wxyz(mat9: np.ndarray) -> np.ndarray:
    xyzw = R.from_matrix(np.asarray(mat9, dtype=np.float64).reshape(3, 3)).as_quat()
    return np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]], dtype=np.float64)


def _geom_rgba255(model: mj.MjModel, geom_id: int) -> tuple[int, int, int]:
    rgba = model.geom_rgba[geom_id]
    return (int(rgba[0] * 255), int(rgba[1] * 255), int(rgba[2] * 255))


def _texture_rgba_for_geom(
    model: mj.MjModel,
    geom_id: int,
    *,
    alpha: float = 1.0,
) -> Optional[np.ndarray]:
    """Read and tint the MuJoCo material texture for a geom."""
    matid = int(model.geom_matid[geom_id])
    if matid < 0:
        return None
    texid = int(model.mat_texid[matid, 1])
    if texid < 0:
        return None

    width = int(model.tex_width[texid])
    height = int(model.tex_height[texid])
    nchannel = int(model.tex_nchannel[texid])
    adr = int(model.tex_adr[texid])
    raw = np.asarray(model.tex_data[adr : adr + width * height * nchannel], dtype=np.uint8)
    if nchannel == 3:
        rgb = raw.reshape(height, width, 3)
        rgba = np.dstack([rgb, np.full((height, width), 255, dtype=np.uint8)])
    elif nchannel == 4:
        rgba = raw.reshape(height, width, 4)
    else:
        return None

    geom_rgb = np.asarray(model.geom_rgba[geom_id][:3], dtype=np.float32)
    mat_rgb = np.asarray(model.mat_rgba[matid][:3], dtype=np.float32)
    tint = np.clip(geom_rgb * mat_rgb, 0.0, 1.0)
    tex_rgb = rgba[:, :, :3].astype(np.float32) / 255.0
    # MuJoCo: tint * texture, then headlight. Bake shadow fill so PBR stays gray, not black.
    lit = tint.reshape(1, 1, 3) * tex_rgb
    lit = lit * (1.0 - SHADOW_FILL) + tint.reshape(1, 1, 3) * SHADOW_FILL
    mod_rgb = np.clip(lit * ALBEDO_LIFT, 0.0, 1.0)

    out = rgba.copy()
    out[:, :, :3] = np.clip(mod_rgb * 255.0, 0, 255).astype(np.uint8)
    if alpha < 1.0:
        out[:, :, 3] = (out[:, :, 3].astype(np.float32) * alpha).astype(np.uint8)
    return out


def _mesh_arrays(model: mj.MjModel, mesh_id: int) -> tuple[np.ndarray, np.ndarray]:
    vadr = int(model.mesh_vertadr[mesh_id])
    vnum = int(model.mesh_vertnum[mesh_id])
    fadr = int(model.mesh_faceadr[mesh_id])
    fnum = int(model.mesh_facenum[mesh_id])
    verts = np.asarray(model.mesh_vert[vadr : vadr + vnum], dtype=np.float64)
    faces = np.asarray(model.mesh_face[fadr : fadr + fnum], dtype=np.int32)
    return verts, faces


def _mesh_with_uv_arrays(
    model: mj.MjModel,
    mesh_id: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Expand MuJoCo mesh faces into per-corner vertices + UVs (preserves seams)."""
    verts_src, faces_src = _mesh_arrays(model, mesh_id)
    fadr = int(model.mesh_faceadr[mesh_id])
    fnum = int(model.mesh_facenum[mesh_id])
    facetex = np.asarray(model.mesh_facetexcoord[fadr : fadr + fnum], dtype=np.int32)
    tcadr = int(model.mesh_texcoordadr[mesh_id])

    out_v: list[np.ndarray] = []
    out_uv: list[np.ndarray] = []
    out_f: list[list[int]] = []
    for fi in range(fnum):
        row: list[int] = []
        for ci in range(3):
            vi = int(faces_src[fi, ci])
            ti = int(facetex[fi, ci])
            out_v.append(verts_src[vi])
            uv = np.asarray(model.mesh_texcoord[tcadr + ti][:2], dtype=np.float64)
            uv[1] = 1.0 - uv[1]  # MuJoCo/OpenGL -> three.js/viser
            out_uv.append(uv)
            row.append(len(out_v) - 1)
        out_f.append(row)

    return (
        np.asarray(out_v, dtype=np.float64),
        np.asarray(out_uv, dtype=np.float64),
        np.asarray(out_f, dtype=np.int32),
    )


def _build_trimesh_from_mujoco(
    model: mj.MjModel,
    mesh_id: int,
    geom_id: int,
    *,
    alpha: float = 1.0,
) -> trimesh.Trimesh:
    tex = _texture_rgba_for_geom(model, geom_id, alpha=alpha)
    if tex is None or int(model.mesh_texcoordnum[mesh_id]) <= 0:
        verts, faces = _mesh_arrays(model, mesh_id)
        mesh = trimesh.Trimesh(
            vertices=np.asarray(verts, dtype=np.float64),
            faces=np.asarray(faces, dtype=np.int32),
            process=False,
        )
        color = _geom_rgba255(model, geom_id)
        lifted = tuple(min(255, int(c * ALBEDO_LIFT)) / 255.0 for c in color)
        mesh.visual = trimesh.visual.TextureVisuals(
            material=PBRMaterial(
                baseColorFactor=[*lifted, float(alpha)],
                metallicFactor=PBR_METALLIC,
                roughnessFactor=PBR_ROUGHNESS,
                doubleSided=True,
            ),
        )
        return mesh

    verts, uvs, faces = _mesh_with_uv_arrays(model, mesh_id)
    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
    mesh.visual = trimesh.visual.TextureVisuals(
        uv=uvs,
        material=PBRMaterial(
            baseColorTexture=Image.fromarray(tex),
            metallicFactor=PBR_METALLIC,
            roughnessFactor=PBR_ROUGHNESS,
            doubleSided=True,
        ),
    )
    return mesh


class RobotScene:
    """Loads T800 mesh geometry from MuJoCo and exposes per-frame pose updates."""

    def __init__(
        self,
        server: viser.ViserServer,
        model: mj.MjModel,
        skin: SkinMode = "white",
        *,
        initial_qpos: Optional[np.ndarray] = None,
    ) -> None:
        self.server = server
        self.model = model
        self.data = mj.MjData(model)
        self.skin = skin
        self.geom_handles: list[tuple[int, object]] = []
        self._last_qpos: Optional[np.ndarray] = None
        self._mesh_cache: dict[tuple[int, str, float], trimesh.Trimesh] = {}
        self._build_meshes()
        mj.mj_forward(self.model, self.data)
        start_qpos = initial_qpos if initial_qpos is not None else t800_standing_qpos(model)
        self.update_from_qpos(start_qpos)

    def set_skin(
        self,
        skin: SkinMode,
        *,
        on_progress: Optional[ProgressCallback] = None,
    ) -> None:
        if skin == self.skin and on_progress is None:
            return
        self.skin = skin
        for _, handle in self.geom_handles:
            handle.remove()
        self.geom_handles.clear()
        self._mesh_cache.clear()
        self._build_meshes(on_progress=on_progress)
        if self._last_qpos is not None:
            self.update_from_qpos(self._last_qpos)

    def _cached_trimesh(self, geom_id: int, mesh_id: int, alpha: float) -> trimesh.Trimesh:
        key = (geom_id, self.skin, alpha)
        if key not in self._mesh_cache:
            self._mesh_cache[key] = _build_trimesh_from_mujoco(
                self.model,
                mesh_id,
                geom_id,
                alpha=alpha,
            )
        return self._mesh_cache[key]

    def _build_meshes(self, on_progress: Optional[ProgressCallback] = None) -> None:
        m = self.model
        alpha = TRANSPARENT_ALPHA if self.skin == "transparent" else 1.0
        use_textures = self.skin in ("full", "transparent")

        mesh_geoms: list[tuple[int, int]] = []
        for g in range(m.ngeom):
            if m.geom_type[g] != mj.mjtGeom.mjGEOM_MESH:
                continue
            mesh_geoms.append((g, int(m.geom_dataid[g])))

        total = len(mesh_geoms)
        if on_progress is not None and use_textures:
            on_progress(0.0, "Preparing textures…")

        pending: list[tuple[int, object]] = []
        for idx, (g, mesh_id) in enumerate(mesh_geoms):
            name = f"/robot/geom_{g}"
            if use_textures:
                tm = self._cached_trimesh(g, mesh_id, alpha)
                pending.append((g, self.server.scene.add_mesh_trimesh(name, tm, visible=False)))
            else:
                verts, faces = _mesh_arrays(m, mesh_id)
                pending.append(
                    (
                        g,
                        self.server.scene.add_mesh_simple(
                            name,
                            vertices=np.asarray(verts, dtype=np.float32),
                            faces=np.asarray(faces, dtype=np.int32),
                            color=WHITE_RGB,
                            flat_shading=False,
                        ),
                    )
                )
            if on_progress is not None and use_textures and total > 0:
                pct = 100.0 * float(idx + 1) / float(total)
                on_progress(pct, f"Loading textures… {idx + 1}/{total}")

        if use_textures:
            for _, handle in pending:
                handle.visible = True

        self.geom_handles = pending

        if on_progress is not None and use_textures:
            on_progress(100.0, "Textures loaded.")

    def update_from_qpos(self, qpos: np.ndarray) -> None:
        self._last_qpos = np.asarray(qpos, dtype=np.float64).copy()
        n = min(len(qpos), self.model.nq)
        self.data.qpos[:n] = qpos[:n]
        mj.mj_forward(self.model, self.data)
        for g, handle in self.geom_handles:
            handle.position = np.array(self.data.geom_xpos[g], dtype=np.float64)
            handle.wxyz = _mat_to_wxyz(self.data.geom_xmat[g])
