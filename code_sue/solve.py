#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SUE + MNL 模型求解脚本 - 使用Gurobi原生API

使用方法 (任选其一):
    1. python solve.py                    (在code_sue目录中运行)
    2. python -m code_sue.solve           (从项目根目录运行)
    3. python code_sue/solve.py           (从项目根目录运行)
"""
import sys
import os
import time
from pathlib import Path

# Windows中文编码修复
if sys.platform == 'win32':
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')


def main():
    """主求解函数"""
    print("="*70)
    print("SUE + MNL 单层强对偶模型 - Gurobi 原生求解")
    print("="*70)
    
    try:
        # 灵活的导入处理 - 支持多种运行方式
        try:
            # 方式1: 作为包模块运行
            from .main import (
                load_model_data, finalize_data_for_solver, 
                create_model_parameters, get_sue_hyperparams, _build_path_set
            )
            from .sue_rmp import build_and_solve_sue_master
        except ImportError:
            # 方式2: 作为脚本直接运行
            current_dir = os.path.dirname(os.path.abspath(__file__))
            parent_dir = os.path.dirname(current_dir)
            sys.path.insert(0, parent_dir)
            
            from code_sue.main import (
                load_model_data, finalize_data_for_solver, 
                create_model_parameters, get_sue_hyperparams, _build_path_set
            )
            from code_sue.sue_rmp import build_and_solve_sue_master
        
        print("\n[Step 1] 数据加载与参数构建...")
        data = load_model_data("../data/model_input_data_10km.pkl")
        solver_input = finalize_data_for_solver(data)
        params = create_model_parameters(solver_input)
        path_dict = _build_path_set(params)
        hyper = get_sue_hyperparams()
        
        print(f"  - 节点: {len(params['N'])}")
        print(f"  - 边: {len(params['A'])}")
        print(f"  - OD对: {len(params['K'])}")
        print(f"  - 路径: {len(path_dict)}")
        
        print("\n[Step 2] 构建模型...")
        model_result = build_and_solve_sue_master(params, path_dict, hyper, solve=False)
        mdl = model_result["model"]
        print(f"  - 变量: {mdl.number_of_variables}")
        print(f"  - 约束: {mdl.number_of_constraints}")
        
        print("\n[Step 3] 导出为LP格式...")
        lp_file = "sue_model.lp"
        mdl.export(lp_file)
        print(f"  - 已保存: {lp_file}")
        
        print("\n[Step 4] 使用Gurobi求解器...")
        print("  检测Gurobi环境...")
        
        try:
            import gurobipy as gp
            from gurobipy import GRB
            
            print(f"  - Gurobi版本: {gp.gurobi.version()}")
            
            print("  - 从LP文件读入Gurobi模型...")
            env = gp.Env(empty=True)
            env.setParam("OutputFlag", 1)  # 显示求解过程
            env.setParam("TimeLimit", 3600)  # 1小时限制
            env.setParam("MIPGap", 0.05)  # 5% gap
            env.start()
            
            gurobi_mdl = gp.read(lp_file, env=env)
            
            print("\n[Step 5] Gurobi求解...")
            print("-" * 70)
            
            solve_start = time.time()
            gurobi_mdl.optimize()
            solve_time = time.time() - solve_start
            
            print("-" * 70)
            print(f"\n求解耗时: {solve_time:.2f} 秒")
            
            # 检查结果
            if gurobi_mdl.status == GRB.OPTIMAL:
                print("\n" + "="*70)
                print("[SUCCESS] 找到最优解！")
                print("="*70)
                print(f"\n目标函数值: {gurobi_mdl.objVal:.2f}")
                print(f"求解状态: OPTIMAL")
                print(f"求解时间: {solve_time:.2f} 秒")
                
                # 保存详细结果
                result_file = Path("sue_solution_report.txt")
                with open(result_file, "w", encoding="utf-8") as f:
                    f.write("="*70 + "\n")
                    f.write("SUE + MNL 强对偶模型求解结果\n")
                    f.write("="*70 + "\n\n")
                    f.write(f"求解器: Gurobi\n")
                    f.write(f"求解状态: OPTIMAL\n")
                    f.write(f"目标函数值: {gurobi_mdl.objVal:.2f}\n")
                    f.write(f"求解耗时: {solve_time:.2f} 秒\n\n")
                    f.write(f"模型规模:\n")
                    f.write(f"  - 变量数: {gurobi_mdl.numVars}\n")
                    f.write(f"  - 约束数: {gurobi_mdl.numConstrs}\n")
                    f.write(f"  - 节点: {len(params['N'])}\n")
                    f.write(f"  - 边: {len(params['A'])}\n")
                    f.write(f"  - OD对: {len(params['K'])}\n")
                    f.write(f"  - 路径: {len(path_dict)}\n\n")
                    f.write(f"超参数设置:\n")
                    f.write(f"  - 路径选择离散参数 θ: {hyper['theta']}\n")
                    f.write(f"  - 出行方式离散参数 μ: {hyper['mu']}\n")
                    f.write(f"  - Big-M: {hyper['big_m']}\n")
                
                print(f"\n结果已保存到: {result_file}")
                
                # 保存详细解文件
                sol_file = "sue_solution.sol"
                gurobi_mdl.write(sol_file)
                print(f"详细解已保存到: {sol_file}")
                
                return True
                
            elif gurobi_mdl.status == GRB.TIME_LIMIT:
                print("\n" + "="*70)
                print("[TIME_LIMIT] 达到时间限制")
                print("="*70)
                print(f"\n目标函数值 (当前最佳): {gurobi_mdl.objBound:.2f}")
                print(f"求解时间: {solve_time:.2f} 秒")
                if gurobi_mdl.objVal < float('inf'):
                    print(f"已找到的最优值: {gurobi_mdl.objVal:.2f}")
                return True
                
            elif gurobi_mdl.status == GRB.INFEASIBLE:
                print("\n[INFEASIBLE] 模型不可行")
                return False
                
            else:
                print(f"\n[{gurobi_mdl.status}] 求解未完成")
                return False
                
        except ImportError:
            print("  [ERROR] Gurobi未安装或无法导入")
            print("  请运行: pip install gurobipy")
            return False
            
    except Exception as e:
        print(f"\n[ERROR] 执行错误: {e}")
        import traceback
        traceback.print_exc()
        return False



if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] 被用户中断")
        sys.exit(1)
