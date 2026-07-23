# 将本仓库发布到 GitHub

本 git 仓库根目录**就是** `param_id` 项目文件夹（不是上层 `txk` 目录）。  
只有此处被跟踪的文件会推送。

英文原文：[`../UPLOAD.md`](../UPLOAD.md)

## `git push` 之前

1. 确认私有 CAD 已被忽略：
   ```bash
   git status
   git check-ignore -v assets/proprietary/urdf/*.urdf meshes/*.STL
   ```
2. 确认公开教学 URDF 在跟踪列表中：  
   `models/demo_7dof/demo_arm.urdf`
3. 冒烟测试：
   ```bash
   ./scripts/run_demo.sh
   ```
4. 可选：刷新 `results/examples/` 下的示例图。

## 创建远程并推送

建议仓库名：`arm-param-identification`

```bash
cd /path/to/param_id
git remote add origin git@github.com:<USER>/arm-param-identification.git
git branch -M main
git push -u origin main
```

或使用 GitHub CLI：

```bash
gh repo create arm-param-identification --public --source=. --remote=origin --push
```
