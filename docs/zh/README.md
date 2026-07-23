# 七自由度机械臂动力学 / 静力学参数辨识（仿真复现）

面向作品集 / GitHub 主页的开源仿真工程：用 Pinocchio 实现 \(τ=Yπ\) 回归辨识、QR 基参数、鲁棒最小二乘与激励轨迹验证。

**默认模型**为仓库自带的教学用 7-DoF 几何体臂（无专有 CAD）。本地若有私有 URDF，见 [`../assets/proprietary/README.md`](../assets/proprietary/README.md)。

## 快速跑通

```bash
conda activate param-id   # 或 env_isaaclab
cd <repo>
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf

python scripts/00_sanity_check.py
python scripts/identify_static.py  --method robust_wls
python scripts/identify_dynamic.py --method robust_wls --n-periods 3
python scripts/collision_view.py   --traj fourier --check-only
```

## 两条链路

| | 静力学 | 动力学 |
|--|--------|--------|
| 轨迹 | 低速 cosine | 傅里叶（解析 \(q,\dot q,\ddot q\)） |
| 估计 | 重力 + 库仑摩擦 | 惯量 + 科氏/离心 + 重力 + 库仑/粘性 |
| 用途 | 重力补偿 | 力矩前馈 |

更细的掌握提纲：[`掌握要点.md`](掌握要点.md)  
操作步骤：[`LEARNING.md`](LEARNING.md)

## 公开仓库注意

- `meshes/` 与 `assets/proprietary/` 已被 `.gitignore`（含体积很大的 STL、可能涉密的机型文件）  
- 上传 GitHub 前确认 **不会** 提交实习公司 URDF/网格  
- 仿真 RMSE ≠ 真机辨识精度
