# SUE + MNL 模型 - 项目整理总结

## 项目完成状态 ✓

本项目已完全实现，包括：
- ✅ 完整的SUE+MNL数学模型 (218K变量, 191K约束)
- ✅ Gurobi求解器集成 (验证可用)
- ✅ 完整的文档和使用指南
- ✅ 多种运行方式支持

---

## 文件清单

### 📁 code_sue/ 文件夹（所有核心文件都在这里）

#### 核心程序文件
| 文件 | 功能 | 状态 |
|------|------|------|
| **solve.py** | ⭐ 主求解脚本 | ✅ 可用 |
| main.py | 数据加载与管道 | ✅ 完成 |
| params.py | 超参数与工具函数 | ✅ 完成 |
| sue_rmp.py | 核心模型定义 | ✅ 完成 |
| __init__.py | 包初始化 | ✅ 完成 |

#### 文档文件
| 文件 | 内容 |
|------|------|
| **QUICKSTART.md** | 快速入门指南（30秒上手）|
| **USAGE_GUIDE.md** | 完整使用指南（包含FAQ）|
| README.md | 文件结构和背景说明 |

---

## 快速使用

### 方式1: 从code_sue目录运行（推荐）
```bash
cd code_sue
python solve.py
```

### 方式2: 从项目根目录运行
```bash
python code_sue/solve.py
```

### 方式3: 作为模块运行
```bash
python -m code_sue.solve
```

---

## 求解结果

上一次求解的结果已保存到 `code_sue/` 目录中：

| 文件 | 说明 |
|------|------|
| `sue_solution_report.txt` | 求解摘要 (推荐先看这个) |
| `sue_solution.sol` | Gurobi原生解文件 (详细变量值) |
| `sue_model.lp` | 模型LP格式 (用于调试) |

**最近求解结果：**
- **目标函数值：** 33,666.71
- **求解状态：** OPTIMAL
- **求解耗时：** 0.71 秒
- **模型规模：** 218,168 变量 × 191,030 约束

---

## 文件说明

### solve.py - 主求解脚本

**特点：**
- 使用Gurobi原生API（最可靠）
- 5个清晰的执行步骤
- 自动保存详细结果
- 支持多种运行方式

**执行流程：**
```
Step 1: 数据加载与参数构建
        ↓
Step 2: 构建模型 (218K变量)
        ↓
Step 3: 导出为LP格式
        ↓
Step 4: 检测Gurobi环境
        ↓
Step 5: 求解并保存结果
```

---

### main.py - 数据管道

**主要函数：**
- `run_sue_pipeline(solve=True)` - 完整流程
- `load_model_data()` - 加载数据
- `_build_path_set()` - 构建路径

**用途：**
- 重用现有的数据处理代码
- 参数初始化和路径集合生成

---

### params.py - 参数管理

**关键参数：**
```python
THETA = 1.0              # 路由选择离散参数
MU = 1.0                 # 出行方式选择离散参数  
BIG_M = 1e6              # 大M法系数

# 分段线性化断点（用于外部近似）
BECKMANN_BREAKPOINTS = [0.0, 0.5, 1.0, 1.5]
ENTROPY_PATH_BREAKPOINTS = [1e-3, 0.1, 1.0]
ENTROPY_MODE_BREAKPOINTS = [1e-2, 0.1, 1.0]
```

**修改方法：**
直接编辑THETA、MU等常数，然后重新运行solve.py即可。

---

### sue_rmp.py - 核心模型

**变量数：** 218,168
- 边流量: v[a]
- 路径-出行方式流量: f[k,m]
- OD对出行方式出行量: q[o,d,m]
- 线性化辅助变量: tau, phi, xi
- 对偶变量: rho, lambda, gamma, alpha, beta, eta

**约束数：** 191,030
- 流量守恒约束
- 外部近似约束 (Beckmann + 熵)
- 对偶可行约束
- 强对偶相等性约束

---

## 数据文件

**输入数据：** (必需)
- `../data/model_input_data_10km.pkl`

**输出数据：** (solve.py自动生成)
- `sue_solution_report.txt` - 摘要结果
- `sue_solution.sol` - 详细解
- `sue_model.lp` - 模型文本格式

---

## 常见问题

### Q: 如何修改超参数?
**A:** 编辑 `params.py` 中的常数，如：
```python
THETA = 0.5  # 改为0.5
MU = 0.8     # 改为0.8
```
然后重新运行 `python solve.py`。

### Q: 求解太慢怎么办?
**A:** 减少断点数（精度-速度权衡）：
```python
BECKMANN_BREAKPOINTS = [0.0, 1.0]  # 只用2个点
```

### Q: 如何查看求解过程?
**A:** 运行solve.py后，会实时显示Gurobi的求解日志。

### Q: 如何提取变量值?
**A:** 使用Gurobi命令行查看 `sue_solution.sol` 文件，或用脚本处理。

更多FAQ详见 `USAGE_GUIDE.md`。

---

## 关键指标

| 指标 | 值 |
|------|-----|
| 网络节点 | 11,193 |
| 网络边 | 26,979 |
| OD对 | 32 |
| 路径数 | 192 |
| **模型变量** | **218,168** |
| **模型约束** | **191,030** |
| 模型构建时间 | ~10 秒 |
| 求解时间 (Gurobi) | < 1 秒 |

---

## 技术栈

**必需：**
- Python 3.9+
- docplex (IBM CPLEX Python API)
- gurobipy (Gurobi Python API)
- networkx, pandas, scipy

**已安装的版本：**
- docplex 2.30.251
- gurobipy 13.0.0
- networkx 3.6.1

---

## 文档导航

**新手推荐阅读顺序：**
1. 本文件 (项目总结) ← 你在这里
2. `QUICKSTART.md` (30秒快速入门)
3. `USAGE_GUIDE.md` (深入学习)

**功能查询：**
- 如何运行？ → 本文件 "快速使用" 部分
- 如何修改参数？ → 本文件 "常见问题" 部分或 `USAGE_GUIDE.md`
- 模型的数学含义？ → `USAGE_GUIDE.md` "模型详解" 部分
- 遇到错误？ → `USAGE_GUIDE.md` FAQ部分

---

## 根目录中的其他文件

以下文件在根目录中，**无需使用**（已整合到code_sue/中）：
- `solve_gurobi_native.py` - 备用脚本（功能已合并到solve.py）
- `verify_sue_model.py` - 模型验证脚本（可选）
- `solve_with_gurobi.py` - 过时脚本（已废弃）

**建议：** 所有操作都在 `code_sue/` 文件夹中进行。

---

## 项目结构总览

```
bus_Scheduling-main/
├── code/                      # 原始Wardrop模型（保持不变）
│   ├── main.py
│   ├── rmp_model.py
│   └── ...
│
├── code_sue/                  # ✅ SUE+MNL 模型（使用这个文件夹）
│   ├── solve.py              # ⭐ 主求解脚本
│   ├── main.py               # 数据加载
│   ├── params.py             # 参数管理
│   ├── sue_rmp.py            # 模型定义
│   ├── QUICKSTART.md         # 快速入门
│   ├── USAGE_GUIDE.md        # 完整指南
│   ├── README.md             # 背景说明
│   └── __init__.py
│
├── data/                       # 数据文件夹
│   ├── model_input_data_10km.pkl
│   └── ...
│
├── papers/                     # 论文文件夹
│
└── [根目录杂项文件]
```

---

## 下一步建议

1. **立即尝试：** 运行 `cd code_sue && python solve.py`
2. **深入学习：** 阅读 `USAGE_GUIDE.md` 的"模型详解"部分
3. **修改参数：** 编辑 `params.py` 中的THETA和MU值重新求解
4. **对比分析：** 与原始Wardrop模型的结果进行比较

---

## 联系与支持

- **问题排查：** 查看 `USAGE_GUIDE.md` FAQ部分
- **模型问题：** 检查数据文件是否存在：`../data/model_input_data_10km.pkl`
- **Gurobi问题：** 检查许可证：`gurobi_cl --version`

---

**项目版本：** 1.0  
**完成日期：** 2026年1月5日  
**状态：** ✅ 可用于研究和生产  

祝使用愉快！🎉
