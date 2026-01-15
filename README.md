# Bus Scheduling Integrated Bilevel Optimization Model

## 项目简介

本项目实现了一个基于双层优化的公交线路设计与频率设置模型，结合了随机用户均衡(SUE)和多项Logit模型(MNL)的交通网络优化系统。

## 核心特性

### 双层优化框架
- **上层**：公交运营商最小化系统总成本（运营成本 + 用户成本 + 背景交通成本）
- **下层**：用户通过SUE+MNL模型进行路径和出行方式选择

### 关键技术
- **McCormick线性化**：处理双线性项 f × t（流量 × 时间）
- **强对偶重构**：将下层凸优化问题转换为约束条件
- **分段线性逼近**：处理Beckmann积分和熵函数
- **Big-M方法**：连接上层决策变量和下层响应变量
- **车队规模决策**：显式整数变量 $n_r$，包含容量约束与激活约束
- **模式集同步**：路径模式 D/X/B，外生模式 R/K/W/O 仅以固定效用参与 MNL，需求守恒等式覆盖全部 7 个模式

## 项目结构

```
bus_Scheduling-main/
├── code/                          # 原始模型代码
│   ├── data_load.py              # 数据加载
│   ├── model_parameters.py       # 参数配置
│   └── ...
├── code_sue/                      # SUE-MNL模型实现
│   ├── sue_rmp.py                # 限制主问题
│   ├── sue_sp.py                 # 子问题
│   └── ...
├── integrated_bilevel/            # 双层集成模型
│   ├── model.py                  # 核心模型构建
│   ├── solve.py                  # 求解器
│   ├── parameters.py             # 超参数配置
│   ├── utils.py                  # 工具函数
│   ├── gurobi_wrapper.py         # Gurobi包装器
│   └── docs/                     # 数学文档
│       └── BILEVEL_MATHEMATICAL_FORMULATION.tex
├── data/                          # 数据文件（不上传）
└── papers/                        # 参考文献

```

## 数学模型

详细数学推导见 `integrated_bilevel/docs/BILEVEL_MATHEMATICAL_FORMULATION.tex`

### 目标函数

$$Z_{total} = Z_{sys-op} + Z_{user} + Z_{bg}$$

其中：
- **Z_sys-op**: 系统运营成本（公交固定成本 + 运营成本 + 环境成本）
- **Z_user**: 用户出行成本（时间价值 × 出行时间）
- **Z_bg**: 背景交通成本

### McCormick线性化

对于双线性项 $\zeta = f \times t$，使用4不等式包络：

1. $\zeta \geq t_L \cdot f$
2. $\zeta \geq f_U \cdot t_L + t_U \cdot f - f_U \cdot t_U$
3. $\zeta \leq t_U \cdot f$
4. $\zeta \leq f_U \cdot t_U + t_L \cdot f - f_U \cdot t_L$

## 安装要求

### Python依赖
```bash
pip install networkx numpy pandas gurobipy docplex
```

### 求解器
- Gurobi Optimizer 9.0+ (需要学术许可证或商业许可证)

## 使用方法

### 1. 准备数据
```bash
# 确保数据文件位于 data/ 目录
# model_input_data_10km.pkl
```

### 2. 运行集成模型
```bash
cd integrated_bilevel
python solve.py --time-limit 3600 --mip-gap 0.05
```

### 3. 参数配置
编辑 `integrated_bilevel/parameters.py` 调整：
- SUE分散参数 θ
- MNL尺度参数 μ
- Beckmann分段断点
- 熵函数逼近断点

### 4. 查看结果
```bash
# 模型LP文件
integrated_bilevel.lp

# 求解结果
solution_result.json

# 不可行性诊断（如果模型不可行）
infeasible_model.ilp
```

## 模型规模

典型实例（10km范围）：
- 节点：11,184
- 边：26,936
- OD对：32
- 变量：~191,000（160个二进制变量 + 191,000个连续变量）
- 约束：~192,000

## 求解性能

- 预处理：移除 54,341 行，53,413 列
- 求解时间：取决于网络规模（1-2小时）
- MIP gap：通常设置为 5%

## 主要贡献

1. **理论贡献**：证明了KKT条件等价于SUE+MNL均衡
2. **技术创新**：McCormick线性化处理拥堵效应
3. **实现完整**：从数据加载到求解的完整pipeline

## 已知问题

- 大规模网络可能导致内存不足
- McCormick约束可能在某些情况下过于保守
- 需要仔细调整Big-M参数避免数值问题

## 引用

如果使用本代码，请引用：
```
[待添加论文信息]
```

## 许可证

本项目仅供学术研究使用。

## 联系方式

[添加您的联系信息]
