"""Minimal FBX SDK Python helper compatible with Autodesk samples.

This module is used by legacy parsers expecting Autodesk's `FbxCommon.py`.
It intentionally implements only the subset required by
`general_motion_retargeting/third_party/poselib` FBX backend.
"""

from __future__ import annotations

import fbx

# Re-export SDK classes used by callers.
FbxCriteria = fbx.FbxCriteria
FbxAnimStack = fbx.FbxAnimStack
FbxAnimLayer = fbx.FbxAnimLayer


def InitializeSdkObjects():
    """Create `(manager, scene)` pair."""
    sdk_manager = fbx.FbxManager.Create()
    if sdk_manager is None:
        raise RuntimeError("Unable to create FBX SDK manager.")

    io_settings = fbx.FbxIOSettings.Create(sdk_manager, fbx.IOSROOT)
    sdk_manager.SetIOSettings(io_settings)

    scene = fbx.FbxScene.Create(sdk_manager, "Scene")
    if scene is None:
        raise RuntimeError("Unable to create FBX scene.")

    return sdk_manager, scene


def LoadScene(sdk_manager, scene, filename: str):
    """Load FBX file into an existing scene."""
    importer = fbx.FbxImporter.Create(sdk_manager, "")
    if not importer.Initialize(filename, -1, sdk_manager.GetIOSettings()):
        error = importer.GetStatus().GetErrorString()
        importer.Destroy()
        raise RuntimeError(f"FBX importer failed to initialize: {error}")

    ok = importer.Import(scene)
    if not ok:
        error = importer.GetStatus().GetErrorString()
        importer.Destroy()
        raise RuntimeError(f"FBX import failed: {error}")
    importer.Destroy()
    return True
