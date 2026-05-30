# Publishing to GitHub

Repository: https://github.com/nbamylife7-bot/t800-fbx-studio

```bash
cd web-version
git add .
git commit -m "Update README"
git push origin main
```

## After clone

```bash
git clone https://github.com/nbamylife7-bot/t800-fbx-studio.git
cd t800-fbx-studio
chmod +x install.sh run.sh scripts/*.sh
./install.sh
./scripts/install_fbx_sdk.sh
./run.sh        # localhost:8080
```

Or auto-download FBX SDK during install:

```bash
T800_AUTO_DOWNLOAD_FBX=1 ./install.sh
```

## Refresh bundled GMR backend (maintainers)

```bash
./scripts/bundle_gmr.sh
git add gmr/
git commit -m "Refresh bundled GMR backend"
git push
```
