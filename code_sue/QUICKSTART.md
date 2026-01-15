# 快速入门 (Quick Start)

## 30秒上手

```bash
# 1. 进入项目目录
cd code_sue

# 2. 运行求解脚本
python solve.py

# 3. 查看结果
cat sue_solution_report.txt
```

## 完整指南

详见 [`USAGE_GUIDE.md`](USAGE_GUIDE.md)

## 文件速查

| 文件 | 功能 |
|-----|------|
| `solve.py` | ⭐ **主求解脚本** - 推荐使用 |
| `main.py` | 数据加载与管道 |
| `params.py` | 超参数与工具函数 |
| `sue_rmp.py` | 核心模型定义 |
| `USAGE_GUIDE.md` | 完整文档 |
| `README.md` | 背景说明 |

## 遇到问题？

1. **Gurobi找不到？** → 安装：`pip install gurobipy`
2. **数据文件缺失？** → 确保 `../data/model_input_data_10km.pkl` 存在
3. **模型太大太慢？** → 修改 `params.py` 中的断点数

详见 [`USAGE_GUIDE.md`](USAGE_GUIDE.md) 的FAQ部分。

---

**当前版本：** 1.0 ✓ 可用  
**最后更新：** 2026-01-05
