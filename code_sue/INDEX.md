<!-- README for code_sue folder -->

# 🚀 SUE + MNL 公交网络设计模型

> 一键求解！完全可用的随机用户均衡(SUE) + 多项Logit(MNL)单层强对偶模型

## ⚡ 30秒快速开始

```bash
cd code_sue
python solve.py
```

**结果保存到：** `sue_solution_report.txt`

---

## 📚 文档速查

| 文档 | 用途 | 适合人群 |
|-----|------|---------|
| **QUICKSTART.md** | 30秒快速入门 | 所有人 |
| **PROJECT_SUMMARY.md** | 项目整体概览 | 想了解项目的人 |
| **USAGE_GUIDE.md** | 完整详细指南 | 深入学习、修改参数 |
| **README.md** | 背景和结构说明 | 了解技术细节 |

---

## 📁 核心文件

```
code_sue/
├── 📄 solve.py                ← ⭐ 直接运行这个
├── 📄 main.py                 ← 数据加载
├── 📄 params.py               ← 超参数（θ, μ等）
├── 📄 sue_rmp.py              ← 模型核心（218K变量）
│
├── 📖 QUICKSTART.md           ← 快速指南
├── 📖 PROJECT_SUMMARY.md      ← 项目总结
├── 📖 USAGE_GUIDE.md          ← 完整手册
└── 📖 README.md               ← 背景说明
```

---

## 🎯 运行方式 (任选其一)

### 方式 1️⃣: 在code_sue目录中
```bash
cd code_sue
python solve.py
```

### 方式 2️⃣: 从项目根目录
```bash
python code_sue/solve.py
```

### 方式 3️⃣: 作为模块
```bash
python -m code_sue.solve
```

---

## 📊 模型规模

| 指标 | 数值 |
|-----|------|
| 网络节点 | 11,193 |
| 网络边 | 26,979 |
| **优化变量** | **218,168** |
| **约束条件** | **191,030** |
| 求解时间 | < 1 秒 |
| 最优值示例 | 33,666.71 |

---

## 🔧 修改超参数

编辑 `params.py`：

```python
THETA = 1.0        # 路由选择随机程度 (默认=1.0)
MU = 1.0           # 出行方式随机程度 (默认=1.0)

# 分段线性化断点 (影响精度和求解速度)
BECKMANN_BREAKPOINTS = [0.0, 0.5, 1.0, 1.5]
```

修改后重新运行 `python solve.py` 即可生效。

---

## ✅ 功能清单

- ✅ 完整的SUE+MNL数学模型
- ✅ 原始、对偶、强对偶约束
- ✅ 218K变量 × 191K约束
- ✅ Gurobi求解器集成
- ✅ 自动保存详细结果
- ✅ 多种运行方式
- ✅ 完整的中文文档

---

## ❓ 常见问题

**Q: 找不到数据文件？**  
A: 确保 `../data/model_input_data_10km.pkl` 存在

**Q: Gurobi许可证错误？**  
A: 运行 `gurobi_cl --version` 激活许可证

**Q: 求解太慢？**  
A: 减少 `params.py` 中的断点数

更多问题 → 查看 `USAGE_GUIDE.md` FAQ部分

---

## 📖 推荐阅读顺序

1. 本页面 ← 你在这里 (总览)
2. [QUICKSTART.md](QUICKSTART.md) (5分钟上手)
3. [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md) (10分钟了解项目)
4. [USAGE_GUIDE.md](USAGE_GUIDE.md) (深入学习)

---

## 🎓 相比原始Wardrop模型的改进

| 特性 | 原始 | 改进 |
|------|------|------|
| 路由模型 | 确定性最短路 | **随机用户均衡(SUE)** |
| 出行方式 | 固定 | **多项Logit(MNL)** |
| 用户异质性 | 无 | **隐含体现** |
| 参数灵活性 | 低 | **高(θ,μ可配)** |
| 求解算法 | 启发式 | **精确MILP** |

---

## 🔗 快速链接

- 📄 [快速入门](QUICKSTART.md)
- 📖 [完整手册](USAGE_GUIDE.md)
- 📊 [项目总结](PROJECT_SUMMARY.md)
- 🔍 [技术细节](README.md)

---

## 💾 输出文件

运行 `solve.py` 后自动生成：

- `sue_solution_report.txt` - 求解结果摘要 ⭐
- `sue_solution.sol` - 详细解(Gurobi格式)
- `sue_model.lp` - 模型文本格式(调试用)

---

**版本：** 1.0 ✅ 完全可用  
**最后更新：** 2026-01-05  

---

## 🚀 开始使用

```bash
# 1. 进入文件夹
cd code_sue

# 2. 运行求解
python solve.py

# 3. 查看结果
cat sue_solution_report.txt
```

**就这么简单！** 🎉
