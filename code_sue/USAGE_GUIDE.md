# SUE + MNL 模型 - 完整使用指南

## 项目概述

本项目实现了**随机用户均衡 (Stochastic User Equilibrium, SUE) + 多项Logit (Multinomial Logit, MNL)** 的单层强对偶改进模型，用于公交网络设计和路由分析。

### 相比原始Wardrop模型的改进

| 特性 | Wardrop (原始) | SUE + MNL (改进) |
|------|---------------|-----------------|
| 路由选择 | 确定性最短路 | 随机用户均衡 (SUE) |
| 出行方式选择 | 固定 | 多项Logit (MNL) |
| 用户异质性 | 无 | 隐含体现 |
| 离散参数可调 | θ固定为0.1 | θ=1.0, μ=1.0 (可配置) |

---

## 文件结构

```
code_sue/                           # SUE+MNL 模型实现包
├── __init__.py                     # 包初始化，导出主要函数
├── main.py                         # 数据加载与管道
├── params.py                       # 超参数与切线生成器
├── sue_rmp.py                      # 完整模型构建与求解
├── solve.py                        # Gurobi求解脚本（推荐）
└── README.md                       # 简要说明

根目录:
├── solve_gurobi_native.py          # 备用求解脚本
├── verify_sue_model.py             # 模型验证脚本（不求解）
└── (原始代码/code目录保持不变)
```

---

## 快速开始

### 1. 环境配置

```bash
# 安装必要依赖（如果还未安装）
pip install docplex gurobipy networkx pandas scipy scikit-learn

# 验证Gurobi安装
python -c "import gurobipy; print(gurobipy.gurobi.version())"
```

### 2. 运行求解

最简单的方式（推荐）：

```bash
# 从根目录运行
cd code_sue
python solve.py
```

或者从根目录运行：

```bash
python code_sue/solve.py
```

### 3. 查看结果

求解完成后，在 `code_sue/` 目录中生成以下文件：

| 文件 | 说明 |
|-----|------|
| `sue_solution_report.txt` | 求解结果摘要 |
| `sue_solution.sol` | Gurobi原生解文件（详细变量值） |
| `sue_model.lp` | 模型的LP格式（调试用） |

---

## 模型详解

### 数学模型框架

强对偶改进的单层模型：

**原始问题 (SUE路由 + MNL出行方式)：**
```
min  Z_primal = ∫ t_a(v_a) dv_a + θ⁻¹ ∑ f_k^m ln(f_k^m) + μ⁻¹ ∑ q_od^m ln(q_od^m)
                + λ·(设计目标)

s.t. 
  - 流量平衡：∑_m ∑_k f_k^m = D_od  ∀(o,d)
  - 路径容量：v_a = ∑_od ∑_k f_k^m·δ_ak^m  ∀a
  - MNL约束：q_od^m = D_od·exp(μ·u_od^m) / ∑_m' exp(μ·u_od^m')
```

**对偶问题（KKT推导）：**
```
max  Z_dual = ∑_od D_od·ρ_od - ∑_a ∫_0^{v̄_a} ρ(v) dv
             + 强对偶特性项

s.t.  双可行约束（所有变量的梯度条件）
```

**强对偶相等性：**
```
Z_primal = Z_dual  (最优解处成立)
```

### 关键参数

在 `params.py` 中定义：

```python
THETA = 1.0           # 路由选择离散参数 (越小→更随机)
MU = 1.0              # 出行方式选择离散参数
BIG_M = 1e6           # 大M法系数（用于线性化）

# 分段线性化的断点
BECKMANN_BREAKPOINTS = [0.0, 0.5, 1.0, 1.5]           # 用于 Beckmann 积分
ENTROPY_PATH_BREAKPOINTS = [1e-3, 0.1, 1.0]          # 用于路径熵
ENTROPY_MODE_BREAKPOINTS = [1e-2, 0.1, 1.0]          # 用于出行方式熵
```

### 修改超参数

**方法1：直接编辑 `params.py`**
```python
THETA = 0.5  # 改为0.5
MU = 0.8     # 改为0.8
```

**方法2：运行时覆盖（未来扩展）**
```python
from code_sue import run_sue_pipeline
params = {'theta': 0.5, 'mu': 0.8}
result = run_sue_pipeline(hyper_override=params)
```

---

## 文件详解

### `main.py` - 数据管道

**主函数：** `run_sue_pipeline(solve=True)`

功能：
1. 加载现有模型数据 (`data/model_input_data_10km.pkl`)
2. 最终化求解器输入数据
3. 创建模型参数字典
4. 构建路径集合
5. 调用求解器

```python
# 使用示例
from code_sue.main import run_sue_pipeline

# 只构建模型不求解
result = run_sue_pipeline(solve=False)
model = result['model']

# 构建并求解
result = run_sue_pipeline(solve=True)
```

### `params.py` - 参数与工具函数

**导出函数：**

| 函数 | 输入 | 输出 | 用途 |
|------|------|------|------|
| `get_sue_hyperparams()` | 无 | dict | 获取所有超参数 |
| `build_beckmann_tangents(v_lb, v_ub, ...)`  | 值域、成本函数 | list[(x,y), ...] | Beckmann积分切线点 |
| `build_entropy_tangents(q_lb, q_ub, ...)` | 值域 | list[(x,y), ...] | 熵函数切线点 |

**使用示例：**
```python
from code_sue.params import build_beckmann_tangents, build_entropy_tangents

# 生成Beckmann积分的切线点
# (用于分段线性化)
tangents = build_beckmann_tangents(
    v_lower_bound=0.0,
    v_upper_bound=2.0,
    link_id='link_1'
)
# 返回: [(0.0, 0.0), (0.5, y1), (1.0, y2), (1.5, y3), (2.0, y4)]
```

### `sue_rmp.py` - 核心模型

**主函数：** `build_and_solve_sue_master(params, path_dict, hyper, solve=True)`

模型包含以下变量块：

| 变量块 | 变量名 | 维度 | 说明 |
|--------|--------|------|------|
| **原始1** | `v[a]` | \|A\| | 边流量 |
| | `f[k,m]` | \|K\|×\|M\| | 路径-出行方式流量 |
| | `q[o,d,m]` | \|OD\|×\|M\| | OD对出行方式出行量 |
| **原始2** | `tau[k,m], phi[k,m], xi[k,m]` | \|K\|×\|M\| | Beckmann、熵(路径)、熵(方式)切线系数 |
| **对偶1** | `rho[o,d], lambda[k,m], gamma[o,d]` | | 流量约束的对偶变量 |
| **对偶2** | `alpha[a,r], beta[k,j], eta[o,d,m,j]` | | 切线约束的对偶变量 |

约束包括：
- 流量守恒（原始可行）
- 外部近似切线（Beckmann、路径熵、方式熵）
- 对偶可行性（KKT条件）
- 强对偶相等性 (`Z_primal = Z_dual`)

---

## 求解脚本详解

### `solve.py` (推荐使用)

**特点：**
- 使用Gurobi原生API（不依赖Docplex求解器选择）
- 完整的步骤提示和进度输出
- 自动保存详细结果

**执行流程：**
```
[Step 1] 数据加载与参数构建
  ├─ 加载模型数据 (11,193 节点, 26,979 边)
  └─ 生成参数与路径集合

[Step 2] 构建模型
  ├─ 生成 218,168 个变量
  └─ 生成 191,030 个约束

[Step 3] 导出为LP格式
  └─ 保存 sue_model.lp (用于Gurobi)

[Step 4] 使用Gurobi求解
  ├─ 检测Gurobi环境
  └─ 加载LP模型

[Step 5] Gurobi求解
  ├─ 运行优化器...
  └─ 保存结果
```

**输出示例：**
```
======================================================================
SUE + MNL 单层强对偶模型 - Gurobi 原生求解
======================================================================

[Step 1] 数据加载与参数构建...
  - 节点: 11193
  - 边: 26979
  - OD对: 32
  - 路径: 192

[Step 2] 构建模型...
  - 变量: 218168
  - 约束: 191030

...

======================================================================
[SUCCESS] 找到最优解！
======================================================================

目标函数值: 33666.71
求解状态: OPTIMAL
求解时间: 0.71 秒

结果已保存到: sue_solution_report.txt
详细解已保存到: sue_solution.sol
```

### `verify_sue_model.py` - 仅验证模型

如果只想检查模型是否正确构建（不求解）：

```bash
python verify_sue_model.py
```

输出模型统计但**不运行优化器**。用于：
- 快速检查模型结构
- 验证约束和变量数量
- 调试模型定义

---

## 常见问题 (FAQ)

### Q1: 运行 `solve.py` 遇到 "Gurobi license not found" 错误

**原因：** Gurobi需要有效的许可证（商业/学术许可）

**解决方案：**
1. 检查Gurobi许可证状态：`gurobi_cl --version`
2. 获取学术免费许可证：https://www.gurobi.com/academia/academic-program-and-licenses/
3. 激活许可证：
   ```bash
   gurobi_cl --verbose
   # 按提示登录账户激活
   ```

### Q2: 模型求解时间过长

**原因：** 200K+ 变量的大规模MILP问题

**改进方案：**
1. 减少断点数（在 `params.py`）：
   ```python
   BECKMANN_BREAKPOINTS = [0.0, 1.0]  # 只用2个断点而不是4个
   ```
2. 增加求解时间限制（在 `solve.py`）：
   ```python
   env.setParam("TimeLimit", 7200)  # 改为2小时
   ```
3. 放宽容差：
   ```python
   env.setParam("MIPGap", 0.1)  # 改为10% gap
   ```

### Q3: 如何修改OD对或路径集合

在 `main.py` 的 `_build_path_set()` 函数中修改：

```python
def _build_path_set(params):
    # 当前：使用预生成的最短路径
    # 修改此处以添加其他路径或过滤OD对
    ...
```

### Q4: 如何提取求解后的流量等结果

查看 `sue_solution.sol` 文件或使用Gurobi API：

```python
import gurobipy as gp

model = gp.read("code_sue/sue_solution.sol")
for var in model.getVars():
    if var.X > 1e-6:  # 只显示非零变量
        print(f"{var.VarName} = {var.X}")
```

### Q5: 如何与原始Wardrop模型对比

```bash
# 运行原始模型
python code/main.py

# 运行SUE+MNL模型
python code_sue/solve.py

# 比较目标函数值和流量分配结果
```

---

## 数学推导参考

### Beckmann积分近似

Cost function (BPR模型):
$$t_a(v_a) = t_a^0 \left[1 + \alpha\left(\frac{v_a}{C_a}\right)^\beta\right]$$

Beckmann积分 (原始目标的一部分):
$$Z_B = \int_0^{v_a} t_a(v) dv$$

分段线性近似：选择断点 $0 = v_0 < v_1 < \cdots < v_r = \bar{v}_a$，生成切线。

### 信息熵项

路径选择熵：
$$Z_{E,p} = \theta^{-1} \sum_{k,m} f_k^m \ln(f_k^m)$$

出行方式选择熵：
$$Z_{E,m} = \mu^{-1} \sum_{o,d,m} q_{od}^m \ln(q_{od}^m)$$

分段线性化使用切线点：$(q_j, q_j \ln q_j + \text{slope} \cdot q_j)$

### 强对偶

如果原始问题是凸的，则：
$$\min Z_p = \max Z_d$$

我们在最优解处添加约束 $Z_p - Z_d = 0$ 以强制最优性。

---

## 模型验证清单

运行求解前，确保：

- [ ] Python环境已配置（venv/conda）
- [ ] 依赖已安装：`pip list | grep docplex gurobipy`
- [ ] Gurobi许可证已激活：`gurobi_cl --version`
- [ ] 数据文件存在：`data/model_input_data_10km.pkl`
- [ ] 模型验证无误：`python verify_sue_model.py`

---

## 性能指标

当前实现（4x4断点设置）：

| 指标 | 值 |
|-----|-----|
| 模型构建时间 | ~10 秒 |
| 变量数 | 218,168 |
| 约束数 | 191,030 |
| 求解时间 (Gurobi) | < 1 秒 (某些实例) |
| 内存占用 | ~500 MB |
| 最优值 (示例) | 33,666.71 |

---

## 扩展方向

未来可在此基础上实现：

1. **列生成算法** - 动态添加路径列
2. **分解方法** - 对偶分解(Dantzig-Wolfe)或Benders分解
3. **非线性求解器** - 保留原始非线性模型，使用IPOPT/KNITRO
4. **参数扫描** - 批量运行不同θ, μ值下的最优解
5. **灵敏度分析** - 对路径成本、OD需求的灵敏度

---

## 联系与支持

- 问题报告：[项目Issue页面]
- 论文引用：[待补充]
- 许可证：[MIT/Apache 2.0]

---

**最后更新：** 2026年1月5日  
**版本：** 1.0 (完全可用)
