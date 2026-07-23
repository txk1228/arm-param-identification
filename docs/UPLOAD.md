# Publishing this repo to GitHub (portfolio)

## Before `git push` — checklist

1. **No proprietary CAD**
   ```bash
   git status
   git check-ignore -v assets/proprietary/urdf/i7_left_arm.urdf meshes/base_link.STL
   # both should be ignored
   ```
2. Confirm `models/demo_7dof/demo_arm.urdf` is tracked (public demo).
3. Run demo once:
   ```bash
   ./scripts/run_demo.sh
   ```
4. Copy 1–2 clean PNGs into `results/examples/` if you want README screenshots.

## Create remote & push

Suggested repo name: `arm-param-identification` or `robot-dynamics-param-id`

```bash
cd /path/to/param_id
git init
git add .
git status   # review carefully
git commit -m "$(cat <<'EOF'
Initial public release: 7-DoF dynamics/statics parameter ID simulation.

EOF
)"

# create empty repo on GitHub (txk1228), then:
git branch -M main
git remote add origin git@github.com:txk1228/arm-param-identification.git
git push -u origin main
```

Link it from https://github.com/txk1228/xiaoke-portfolio

## Portfolio blurb (EN)

> Simulation of 7-DoF manipulator dynamics/statics parameter identification with Pinocchio: regressor construction, QR base parameters, robust WLS, Fourier excitation, and trajectory collision checks—for gravity compensation / torque feedforward.

## Portfolio blurb (中文)

> 基于 Pinocchio 的七自由度机械臂动力学/静力学参数辨识仿真：回归矩阵、QR 基参数、鲁棒最小二乘、傅里叶激励与轨迹碰撞验证，面向重力补偿与力矩前馈。
