# Gravity compensation verification

- mode: `offline`
- urdf: `models/demo_7dof/demo_arm.urdf`
- id_result: `results/baseline/id_pinocchio_static/static_ols.npz`
- pass: **True**

## Offline residual vs RNEA truth

| mode | mean ||r|| | max ||r|| |
|------|----------:|---------:|
| none | 9.8370e+00 | 1.6403e+01 |
| urdf | 0.0000e+00 | 0.0000e+00 |
| identified | 3.3684e-14 | 5.5953e-14 |
