# Results directory

Local experiment outputs. Most run artifacts are gitignored; only curated examples and showcase folders are tracked.

## Layout

```text
results/
├── examples/              # Committed example figures (static / dynamic robust WLS)
├── comparison/            # Stage-6 cross-comparison (tracked)
│   ├── data_*.npz         # Input datasets (ideal Pinocchio / engineering proxy)
│   ├── result_*.npz       # Identification outputs per group
│   ├── fig_*.png          # RMSE bars, param error, torque fit
│   ├── summary.json       # Numeric metrics
│   ├── conclusion.md      # Auto-generated narrative
│   └── experiment_config.yaml
├── gravity_comp/          # Gravity-compensation verification (tracked)
│   ├── fig_residual_bars.png
│   ├── gravity_comp_metrics.json
│   └── conclusion.md
├── baseline/              # Alignment & exported NPZ (mostly local)
│   ├── pinocchio_*.npz
│   ├── isaac_*.npz
│   └── compare_*_selfcheck/
├── isaac_static/          # ID from Isaac static NPZ
├── isaac_dynamic/         # ID from Isaac dynamic NPZ
├── *.npz, *.png           # Default demo outputs from run_demo.sh (gitignored)
└── collision_*.txt        # Collision check logs (gitignored)
```

## Regenerate showcase outputs

```bash
bash scripts/run_comparison_experiments.sh --skip-isaac
python scripts/verify_gravity_compensation.py --offline \
  --id-result results/static_ols.npz --out-dir results/gravity_comp
```

---

# 结果目录说明

本地实验输出目录。大部分运行产物不会提交到 Git；仅 `examples/`、`comparison/`、`gravity_comp/` 等展示用子目录会纳入版本库。

## 子目录

| 路径 | 内容 |
|------|------|
| `examples/` | 静/动力学辨识示例图 |
| `comparison/` | 三组对照实验（理想 OLS / 工程 OLS / 工程 robust WLS） |
| `gravity_comp/` | 重力补偿残差验证 |
| `baseline/` | 基准对齐导出的 NPZ、自检指标 |
| `isaac_static/` / `isaac_dynamic/` | 从 Isaac NPZ 辨识的结果 |
| 根目录 `*.npz` / `*.png` | `run_demo.sh` 默认输出（已 gitignore） |

## 指标阅读提示

交叉对比中 **基参数相对误差** 在工程组可能高达数百个百分点，因为数据含摩擦/异常而对照真值为无摩擦 URDF。工程上应优先看 **力矩内点 RMSE** 与重力补偿残差，详见 [`docs/zh/RESULTS_ANALYSIS.md`](../docs/zh/RESULTS_ANALYSIS.md)。
