# Публикация на GitHub

Репозиторий готов в папке `web-version/` (~100 MB с моделью T800).

```bash
cd web-version
git init
git add .
git commit -m "T800 FBX Studio: FBX/BVH to PKL + viser web preview"
gh repo create t800-fbx-studio --public --source=. --remote=origin --push
```

Или без `gh`:

```bash
git remote add origin git@github.com:YOUR_USER/t800-fbx-studio.git
git push -u origin main
```

## Что получит пользователь после clone

```bash
git clone https://github.com/YOUR_USER/t800-fbx-studio.git
cd t800-fbx-studio
./install.sh    # conda env t800-studio
./run.sh        # localhost:8080
```

FBX SDK каждый ставит сам (Autodesk, бесплатно) — см. README.

## Обновление backend после правок в cyanpuppets

```bash
./scripts/bundle_gmr.sh
git add gmr/
git commit -m "Refresh bundled GMR backend"
```
