# Documentation index / 文档索引

| Document | Language | Description |
|----------|----------|-------------|
| [../README.md](../README.md) | English | Project overview, Isaac GUI/headless demos, workflows |
| [zh/README.md](zh/README.md) | 中文 | 项目总览、Isaac 带 GUI/无头 Demo、工作流 |
| [METHOD.md](METHOD.md) | English | Algorithms: regressors, QR, robust estimators |
| [zh/METHOD.md](zh/METHOD.md) | **中文** | **算法原理：回归器、QR、鲁棒估计** |
| [LEARNING.md](LEARNING.md) | English | Pipeline reproduction & module map |
| [zh/LEARNING.md](zh/LEARNING.md) | **中文** | **管线复现与模块对照** |
| [BASELINE_ALIGNMENT.md](BASELINE_ALIGNMENT.md) | English | Pinocchio vs Isaac ideal-physics gate |
| [zh/BASELINE_ALIGNMENT.md](zh/BASELINE_ALIGNMENT.md) | **中文** | **基准对齐（理想物理，&lt; 5%）** |
| [zh/RESULTS_ANALYSIS.md](zh/RESULTS_ANALYSIS.md) | 中文 | 指标解读、读图要点与复现命令 |
| [UPLOAD.md](UPLOAD.md) | English | Pre-push checklist for public repos |
| [zh/UPLOAD.md](zh/UPLOAD.md) | **中文** | **公开仓库推送前检查清单** |
| [../results/README.md](../results/README.md) | EN / 中文 | Output directory layout / 结果目录说明 |
| [../assets/proprietary/README.md](../assets/proprietary/README.md) | English | Private URDF / mesh placement |

**中文文档阅读顺序：**  
[`zh/README.md`](zh/README.md) → [`zh/METHOD.md`](zh/METHOD.md) → [`zh/LEARNING.md`](zh/LEARNING.md) → [`zh/RESULTS_ANALYSIS.md`](zh/RESULTS_ANALYSIS.md)

---

## One-shot demo / 一键演示

| Script | Console language | Notes |
|--------|------------------|-------|
| [`scripts/run_demo.sh`](../scripts/run_demo.sh) | English | Default public demo |
| [`scripts/run_demo_zh.sh`](../scripts/run_demo_zh.sh) | 中文 | Same pipeline; metric hints in Chinese |

Both run: sanity check → static ID → dynamic ID → collision check. Outputs under `results/`.

Language is controlled by `PARAM_ID_LANG` (`en` / `zh`), implemented in [`utils/cli_lang.py`](../utils/cli_lang.py). Example:

```bash
PARAM_ID_LANG=zh python scripts/identify_static.py --method robust_wls
```

**环境：** `conda activate param-id`（方案 A）或 `env_isaaclab`（方案 B，含 Isaac）。  
详见 [`zh/README.md`](zh/README.md) / [`../README.md`](../README.md)。
