# Publish this repository to GitHub

This git repository root **is** the `param_id` project folder (not the parent `txk` tree).
Only files tracked here will be pushed.

**中文版：** [`zh/UPLOAD.md`](zh/UPLOAD.md)

## Before `git push`

1. Confirm proprietary CAD is ignored:
   ```bash
   git status
   git check-ignore -v assets/proprietary/urdf/*.urdf meshes/*.STL
   ```
2. Confirm the public demo URDF is tracked:
   `models/demo_7dof/demo_arm.urdf`
3. Smoke-test:
   ```bash
   ./scripts/run_demo.sh
   ```
4. Optionally refresh figures under `results/examples/`.

## Create remote and push

Suggested repository name: `arm-param-identification`

```bash
cd /path/to/param_id
git remote add origin git@github.com:<USER>/arm-param-identification.git
git branch -M main
git push -u origin main
```

Or with GitHub CLI:

```bash
gh repo create arm-param-identification --public --source=. --remote=origin --push
```
