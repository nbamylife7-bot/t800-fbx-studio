# Публикация на GitHub

Репозиторий: https://github.com/nbamylife7-bot/t800-fbx-studio

```bash
cd web-version
git add .
git commit -m "Fix FBX SDK install docs and download scripts"
git push origin main
```

## Что получит пользователь после clone

```bash
git clone https://github.com/nbamylife7-bot/t800-fbx-studio.git
cd t800-fbx-studio
chmod +x install.sh run.sh scripts/*.sh
./install.sh
./scripts/download_fbx_sdk.sh
source .fbx_sdk_cache/paths.env
./scripts/install_fbx_sdk.sh
./run.sh        # localhost:8080
```

Или одной командой с автозагрузкой SDK:

```bash
T800_AUTO_DOWNLOAD_FBX=1 ./install.sh
```

## Обновление backend после правок в cyanpuppets

```bash
./scripts/bundle_gmr.sh
git add gmr/
git commit -m "Refresh bundled GMR backend"
git push
```
