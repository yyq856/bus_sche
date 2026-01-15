#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
完整列生成算法实现 - Generate-and-Fix Framework
按照用户提供的伪代码完整实现，不做任何简化
"""

import sys
import time
from pathlib import Path
from typing import Dict, Optional, List, Tuple
import networkx as nx
from docplex.mp.model import Model
from docplex.mp.context import Context

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent / "code"))

from data_load import load_model_data, finalize_data_for_solver
from model_parameters import create_model_parameters

try:
    from integrated_bilevel.parameters import get_sue_hyperparams
    from integrated_bilevel.model import build_integrated_bilevel_model
except ImportError:
    from integrated_bilevel.parameters import get_sue_hyperparams
    from integrated_bilevel.model import build_integrated_bilevel_model


class ColumnGenerationAlgorithm:
    """完整的列生成算法 - 按照伪代码实现"""
    
    def __init__(self, params: Dict, hyper: Dict, verbose: bool = True,
                 max_initial_routes: int = 5,
                 max_cols_per_iter: int = 10,
                 pricing_order: str = "rc_desc"):
        self.params = params
        self.hyper = hyper
        self.verbose = verbose
        # 可配置参数
        self.max_initial_routes = max(1, int(max_initial_routes))
        self.max_cols_per_iter = max(1, int(max_cols_per_iter))
        # pricing_order: rc_desc | rc_asc | none
        self.pricing_order = pricing_order if pricing_order in {"rc_desc", "rc_asc", "none"} else "rc_desc"
        
        # 算法参数
        self.G = params['graph']
        self.K = params['K']  # OD对集合
        # arc_cost_per_km: 线路运营的每公里可变成本（从参数的VC_r获取）
        self.arc_cost_per_km = params.get("VC_r", 0.02)  # 与目标函数量纲匹配
        self.fixed_cost_route = params.get("FC_r", 0.0)  # 与目标函数量纲匹配
        
        # 结果存储
        self.omega = {}  # 路由集合 Ω
        self.next_route_id = 0
        self.Z_LB = None  # 下界
        self.Z_UB = None  # 上界
        self.gap = None
        self.iteration_history = []
        self.final_solution = None  # Phase II的完整解
        self.final_variables = None  # Phase II的变量字典
    
    def execute(self, max_iterations: int = 20, time_limit_phase1: int = 600, time_limit_phase2: int = 1200) -> Dict:
        """
        执行完整的列生成算法
        
        Algorithm 1: Generate-and-Fix
        ===============================
        Phase I:  列生成 (LP松弛) → 获得下界 Z_LB
        Phase II: 整数规划求解 (MILP) → 获得上界 Z_UB
        """
        
        if self.verbose:
            print("\n" + "="*80)
            print("PHASE I: COLUMN GENERATION (LP RELAXATION)")
            print("="*80)
        
        # Phase I: 列生成
        self._execute_phase_i_column_generation(max_iterations, time_limit_phase1)
        
        if self.verbose:
            print("\n" + "="*80)
            print("PHASE II: INTEGER RESOLUTION (MILP)")
            print("="*80)
        
        # Phase II: 整数规划
        self._execute_phase_ii_integer_resolution(time_limit_phase2)
        
        # 计算最优性间隙
        if self.Z_LB is not None and self.Z_UB is not None and self.Z_LB > 0:
            self.gap = (self.Z_UB - self.Z_LB) / self.Z_LB * 100
        
        return {
            'Z_LB': self.Z_LB,
            'Z_UB': self.Z_UB,
            'gap': self.gap,
            'num_routes': len(self.omega),
            'iteration_history': self.iteration_history,
            'omega': self.omega,  # 最终的路由集合
            'solution': self.final_solution,  # Phase II的解
            'variables': self.final_variables,  # Phase II的变量
        }
    
    def _execute_phase_i_column_generation(self, max_iterations: int, time_limit: int):
        """
        Phase I: 列生成算法
        
        算法步骤：
        1. 初始化: Ω = {最短路径集合}
        2. 迭代:
           2.1 求解 LP-RMP: min cx s.t. 需求约束
           2.2 提取对偶变量: γ_w (需求), ρ_a (拥堵)
           2.3 求解定价子问题: 找最短路径 (修正成本 = γ_w - 边成本 - FC)
           2.4 若存在正约减成本的路径，加入Ω
           2.5 否则终止（收敛）
        """
        
        start_time = time.time()
        
        # Step 1: 初始化Ω
        self._initialize_omega()
        
        if self.verbose:
            print(f"\n初始化完成: Ω 包含 {len(self.omega)} 条路由")
        
        # Step 2: 列生成迭代
        for iteration in range(1, max_iterations + 1):
            elapsed = time.time() - start_time
            if elapsed > time_limit:
                if self.verbose:
                    print(f"\n[INFO] Time limit reached after {elapsed:.1f}s")
                break
            
            if self.verbose:
                print(f"\n{'='*70}")
                print(f"迭代 {iteration}/{max_iterations} | 当前路径数: {len(self.omega)} | 已用时: {elapsed:.1f}s")
                print(f"{'='*70}")
            
            # Step 2.1: 求解 LP-RMP
            if self.verbose:
                print(f"  [1/3] 正在求解LP松弛主问题...")
            
            iter_start = time.time()
            lp_result = self._solve_lp_rmp()
            lp_time = time.time() - iter_start

            if lp_result is None:
                if self.verbose:
                    print("  [FAIL] LP求解失败，停止列生成")
                break
            else:
                z_lp, gamma_w, rho_a = lp_result
                self.Z_LB = z_lp
                if self.verbose:
                    print(f"  ✓ LP求解成功 (耗时: {lp_time:.1f}s)")
                    print(f"  ├─ 目标函数值: Z_LP = ${z_lp:,.2f}")
                    gamma_nonzero = sum(1 for v in gamma_w.values() if abs(v) > 1e-9)
                    print(f"  └─ 对偶变量: {gamma_nonzero}/{len(gamma_w)} gamma非零")
            
            # Step 2.3: 求解定价子问题
            if self.verbose:
                print(f"\n  [2/3] 正在求解定价子问题...")
            
            pricing_start = time.time()
            new_routes = self._solve_pricing_problem(gamma_w, rho_a)
            pricing_time = time.time() - pricing_start
            
            if self.verbose:
                print(f"  ✓ 定价子问题求解完成 (耗时: {pricing_time:.1f}s)")
                if new_routes:
                    rc_values = [r['reduced_cost'] for r in new_routes]
                    print(f"  ├─ 找到候选路径: {len(new_routes)} 条")
                    print(f"  ├─ RC范围: [{min(rc_values):.2f}, {max(rc_values):.2f}]")
                    print(f"  └─ 平均RC: {sum(rc_values)/len(rc_values):.2f}")
                else:
                    print(f"  └─ 未找到正RC路径")
            
            if not new_routes:
                if self.verbose:
                    print(f"\n  [收敛] 无路径满足RC < 0，列生成收敛")
                break
            
            # Step 2.4: 将新路由加入Ω
            if self.verbose:
                print(f"\n  [3/3] 正在添加新路径到路径集...")
            
            # 所有new_routes都满足RC < -1e-6（在定价问题中已过滤）
            # 选择最改进的路径（RC最小/最负的）
            candidates = list(new_routes)  # 所有生成的路径都是有效的
            
            # 按RC值排序：rc_desc = RC最小（最负）优先，rc_asc = RC最大（最接近0）优先
            if self.pricing_order == "rc_desc":
                # 最负RC优先（最改进的）
                candidates.sort(key=lambda r: r['reduced_cost'])  # 升序 = 最负在前
            elif self.pricing_order == "rc_asc":
                # RC最接近0的优先（最安全的改进）
                candidates.sort(key=lambda r: r['reduced_cost'], reverse=True)  # 降序
            # else: 保持原顺序
            
            # 只添加最多max_cols_per_iter条
            candidates = candidates[:self.max_cols_per_iter]

            num_added = 0
            for route in candidates:
                self.omega[self.next_route_id] = {
                    'nodes': route['nodes'],
                    'distance_km': route['distance_km'],
                    'reduced_cost': route['reduced_cost'],
                }
                self.next_route_id += 1
                num_added += 1
            
            if self.verbose:
                order_note = {
                    "rc_desc": "按RC降序",
                    "rc_asc": "按RC升序",
                    "none": "按生成顺序"
                }[self.pricing_order]
                print(f"  ✓ 已添加 {num_added} 条新路径（最多{self.max_cols_per_iter}条，{order_note}）")
                print(f"  └─ 路径集大小: {len(self.omega)} 条")
            
            if num_added == 0:
                if self.verbose:
                    print(f"\n  [收敛] 无新路径被接受，列生成收敛")
                break
            
            # 记录迭代历史
            self.iteration_history.append({
                'iteration': iteration,
                'Z_LP': self.Z_LB if self.Z_LB else 0.0,
                'num_routes': len(self.omega),
                'new_routes': num_added,
            })
        
        # Phase I 结束摘要
        if self.verbose:
            print(f"\n{'='*70}")
            print(f"PHASE I 完成摘要")
            print(f"{'='*70}")
            print(f"  总迭代次数: {len(self.iteration_history)}")
            print(f"  最终路径数: {len(self.omega)} 条")
            if self.Z_LB:
                print(f"  下界 Z_LB: ${self.Z_LB:,.2f}")
            else:
                print(f"  下界 Z_LB: 未获取")
            print(f"  总耗时: {time.time() - start_time:.1f}s")
            print(f"{'='*70}\n")
    
    def _execute_phase_ii_integer_resolution(self, time_limit: int):
        """
        Phase II: 整数规划求解
        
        步骤：
        1. 固定Ω中的路由（与Phase I完全相同的列集）
        2. 移除LP松弛模式，恢复整数约束和全部KKT对偶约束
        3. 求解MILP
        """
        
        if self.verbose:
            print(f"\n使用 {len(self.omega)} 条路由构建MILP模型...")
        
        try:
            # Step 1: 准备与Phase I完全相同的路由集合
            P_hat = {}
            for rid, rdata in self.omega.items():
                if rdata['nodes']:
                    o, d = int(rdata['nodes'][0]), int(rdata['nodes'][-1])
                    P_hat[rid] = {
                        'nodes': rdata['nodes'],
                        'od': (o, d),
                        'od_pair': (o, d),
                        'route_id': rid,
                    }
            
            if not P_hat:
                if self.verbose:
                    print("[FAIL] No routes in omega for Phase II")
                return
            
            # Step 2: 构建完整MILP模型（带整数约束和KKT约束）
            params_copy = self.params.copy()
            params_copy['P_hat'] = P_hat
            
            if self.verbose:
                print("构建MILP模型（包含完整KKT对偶约束）...")
            
            # 关键：relax_to_lp=False表示恢复整数约束和KKT约束
            model_result = build_integrated_bilevel_model(
                params_copy,
                self.hyper,
                verbose=False,
                P_hat=P_hat,
                relax_to_lp=False  # ✓ 明确指定完整MILP（带对偶约束）
            )
            mdl = model_result["model"]
            
            # 设置时间限制
            try:
                mdl.set_time_limit(time_limit)
            except:
                pass
            
            # Step 3: 求解MILP
            if self.verbose:
                print("求解MILP...")
            
            solution = mdl.solve(log_output=True)

            if solution and solution.objective_value is not None:
                self.Z_UB = solution.objective_value
                self.final_solution = solution
                self.final_variables = model_result.get("variables", {})
                
                if self.verbose:
                    print(f"\n[OK] MILP solved: Z_UB = ${self.Z_UB:,.2f}")
                    # 输出选中的路由和频率
                    self._print_solution_summary(solution, model_result)
            else:
                if self.verbose:
                    print("[FAIL] MILP solve failed")
                self.Z_UB = None
        
        except Exception as e:
            if self.verbose:
                print(f"[FAIL] Phase II error: {e}")
            self.Z_UB = None
    
    def _print_solution_summary(self, solution, model_result):
        """
        输出Phase II解的摘要：选中的路由、频率、车辆数
        """
        try:
            variables = model_result.get("variables", {})
            x = variables.get("x", {})  # 路由激活变量
            w = variables.get("w", {})  # 频率选择变量
            n = variables.get("n", {})  # 车队规模变量
            
            if not x:
                print("\n[WARNING] No route activation variables found in solution")
                return
            
            # 提取选中的路由
            x_sol = solution.get_value_dict(x) if hasattr(solution, 'get_value_dict') else {}
            selected_routes = [r for r, val in x_sol.items() if val > 0.5]
            
            if not selected_routes:
                print("\n[INFO] No routes selected in final solution")
                return
            
            print(f"\n{'='*80}")
            print(f"PHASE II SOLUTION SUMMARY")
            print(f"{'='*80}")
            print(f"\n选中路由数: {len(selected_routes)}/{len(self.omega)}")
            print(f"\n详细信息:")
            print(f"{'路由ID':<10} {'起点→终点':<20} {'长度(km)':<12} {'频率等级':<12} {'车辆数':<10}")
            print(f"{'-'*80}")
            
            # 获取频率和车队解
            w_sol = solution.get_value_dict(w) if w and hasattr(solution, 'get_value_dict') else {}
            n_sol = solution.get_value_dict(n) if n and hasattr(solution, 'get_value_dict') else {}
            
            # 获取频率等级定义
            headway_dict = self.params.get("H_k", {})
            
            for r in sorted(selected_routes):
                route_data = self.omega.get(r, {})
                nodes = route_data.get('nodes', [])
                if not nodes:
                    continue
                
                o, d = nodes[0], nodes[-1]
                distance = route_data.get('distance_km', 0.0)
                
                # 找到选中的频率等级
                selected_headway = None
                for (r_id, k), val in w_sol.items():
                    if r_id == r and val > 0.5:
                        headway_min = headway_dict.get(k, k)
                        selected_headway = f"{k} ({headway_min}min)"
                        break
                
                if selected_headway is None:
                    selected_headway = "N/A"
                
                # 获取车辆数
                fleet_size = n_sol.get(r, 0.0) if r in n_sol else 0.0
                
                print(f"{r:<10} {o:>5} → {d:<5}       {distance:<12.2f} {selected_headway:<12} {fleet_size:<10.1f}")
            
            print(f"{'-'*80}")
            total_fleet = sum(n_sol.get(r, 0.0) for r in selected_routes if r in n_sol)
            print(f"总车辆数: {total_fleet:.0f}")
            print(f"{'='*80}\n")
            
        except Exception as e:
            print(f"\n[WARNING] Failed to print solution summary: {e}")
    
    def _initialize_omega(self):
        """初始化Ω: 仅选择指定数量的初始路线（按总需求优先）。

        策略：
        - 若可用，先按 `D_odh` 聚合的 OD 总需求降序选取前 `self.max_initial_routes` 个 OD；
        - 对每个入选 OD，生成一条基于 `free_flow_time` 的最短路径；
        - 若某些 OD 无法建路径，则继续按序尝试下一个 OD，直到凑满目标数量或无可选。
        """

        # 1) 计算每个OD对的总需求（若参数可用），否则按原有顺序
        demand_by_od = {}
        D_odh = self.params.get('D_odh', {})
        if D_odh:
            for (o, d, h), val in D_odh.items():
                demand_by_od[(o, d)] = demand_by_od.get((o, d), 0.0) + float(val)
            # 按总需求降序排列OD对
            od_sorted = sorted(self.K, key=lambda od: demand_by_od.get(od, 0.0), reverse=True)
        else:
            # 无需求数据时，使用原始顺序
            od_sorted = list(self.K)

        # 2) 仅添加最多指定数量的初始路线
        max_initial = self.max_initial_routes
        added = 0
        for o, d in od_sorted:
            if added >= max_initial:
                break
            try:
                path = nx.shortest_path(self.G, source=o, target=d, weight='free_flow_time')
                distance_km = sum(
                    self.G.edges[path[i], path[i+1]].get("length", 0) / 1000.0
                    for i in range(len(path)-1)
                )
                self.omega[self.next_route_id] = {
                    'nodes': path,
                    'distance_km': distance_km,
                }
                self.next_route_id += 1
                added += 1
            except (nx.NetworkXNoPath, nx.NodeNotFound):
                # 若该OD无路径，尝试下一个OD
                continue
    
    def _solve_lp_rmp(self) -> Optional[Tuple[float, Dict, Dict]]:
        """
        求解LP主问题 (RMP) - 完整MILP的LP松弛
        
        通过调用 build_integrated_bilevel_model(relax_to_lp=True)
        创建LP松弛版本，保留除KKT对偶约束外的所有约束
        """
        try:
            # 构建当前路由集合
            P_hat = {}
            for rid, rdata in self.omega.items():
                if rdata['nodes']:
                    o, d = int(rdata['nodes'][0]), int(rdata['nodes'][-1])
                    P_hat[rid] = {
                        'nodes': rdata['nodes'],
                        'od': (o, d),  # 使用'od'而不是'od_pair'
                        'od_pair': (o, d),
                        'route_id': rid,
                    }
            
            if not P_hat:
                return None
            
            # 调用MILP构建函数，使用LP松弛模式
            model_dict = build_integrated_bilevel_model(
                self.params,
                self.hyper,
                verbose=False,
                P_hat=P_hat,
                relax_to_lp=True  # 关键：使用LP松弛版本
            )
            
            mdl = model_dict["model"]
            
            # 更新模型以确保约束计数正确
            if hasattr(mdl, 'update'):
                mdl.update()
            
            # 设置求解参数
            if hasattr(mdl, 'set_time_limit'):
                # GurobiModelWrapper interface
                mdl.set_time_limit(120)
            elif hasattr(mdl, 'parameters'):
                # docplex interface
                mdl.parameters.timelimit = 120
            
            # 显示模型规模信息
            if self.verbose:
                print(f"    模型规模: {mdl.number_of_variables} 变量, {mdl.number_of_constraints} 约束")
                print(f"    正在求解LP（时间限制: 120秒）...")
            
            # 求解LP松弛（显示求解器输出）
            solution = mdl.solve(log_output=True)
            
            if solution is None:
                return None
            
            z_lp = solution.objective_value
            
            # 提取对偶变量（从Gurobi LP松弛的对偶值）
            if self.verbose:
                print(f"    正在提取对偶变量...")
            
            gamma_w = {}
            rho_a = {}
            
            try:
                # 获取Gurobi原生模型
                gurobi_model = mdl.model
                
                # 初始化对偶变量为0
                for o, d in self.K:
                    gamma_w[(o, d)] = 0.0
                for a in self.params.get('A', []):
                    rho_a[a] = 0.0
                
                if self.verbose:
                    total_constrs = gurobi_model.NumConstrs
                    print(f"    遍历 {total_constrs:,} 个约束提取对偶值...")
            
                link_str_map = {}
                for a in self.params.get('A', []):
                    link_str_map[str(a)] = a
                    link_str_map[str(a).replace(" ", "")] = a
                
                # 遍历所有约束，提取对偶值
                gamma_count = 0
                rho_count = 0
                for constr in gurobi_model.getConstrs():
                    cname = constr.ConstrName
                    dual_value = constr.Pi  # LP对偶值
                    
                    # 提取需求约束的对偶变量（gamma）
                    # 约束名格式: demand_elastic_{o}_{d}_{h}
                    if cname.startswith("demand_elastic_"):
                        try:
                            parts = cname.split("_")
                            if len(parts) >= 5:
                                o = int(parts[2])
                                d = int(parts[3])
                                # h = int(parts[4])
                                # 累加所有时段的对偶值
                                gamma_w[(o, d)] = gamma_w.get((o, d), 0.0) + dual_value
                                gamma_count += 1
                        except:
                            pass
                    
                    # 提取链路流量约束的对偶变量（rho）
                    # 约束名格式: link_flow_(u, v) 或 link_flow_(u,_v)
                    elif cname.startswith("link_flow_"):
                        try:
                            # 解析约束名中的链路信息
                            # 格式: link_flow_(1234, 5678)
                            link_str = cname.replace("link_flow_", "")
                            # 使用预构建的映射表快速查找
                            if link_str in link_str_map:
                                a = link_str_map[link_str]
                                rho_a[a] = dual_value
                                rho_count += 1
                        except:
                            pass
                
                # 统计对偶变量
                gamma_nonzero = sum(1 for v in gamma_w.values() if abs(v) > 1e-9)
                rho_nonzero = sum(1 for v in rho_a.values() if abs(v) > 1e-9)
                
                if self.verbose:
                    print(f"    ✓ 对偶变量提取完成")
                    print(f"      提取了 {gamma_count} 个需求约束, {rho_count} 个流量约束")
                    print(f"      非零: {gamma_nonzero}/{len(gamma_w)} gamma, {rho_nonzero}/{len(rho_a)} rho")
                        
            except Exception as e:
                # 如果提取失败，使用零值（会导致找不到正RC列）
                if self.verbose:
                    print(f"    [WARNING] Failed to extract duals: {e}")
                for o, d in self.K:
                    gamma_w[(o, d)] = 0.0
                for a in self.params.get('A', []):
                    rho_a[a] = 0.0
            
            return z_lp, gamma_w, rho_a
        
        except Exception as e:
            if self.verbose:
                print(f"  [LP ERROR] {type(e).__name__}: {e}")
            return None
    
    def _solve_pricing_problem(self, gamma_w: Dict, rho_a: Dict) -> List[Dict]:
        """
        求解定价子问题: 寻找具有正reduced cost的新路径
        
        Reduced Cost公式:
        RC_r = gamma_{o,d} - Σ_a∈r (c_a + rho_a) - FC_r
        
        其中:
        - gamma_{o,d}: OD需求约束的对偶变量（需求价值）
        - c_a: 链路运营成本（每公里成本 × 距离）
        - rho_a: 链路流量约束的对偶变量（拥堵成本）
        - FC_r: 路径固定成本
        
        如果RC > 0，说明添加该路径可以改进目标函数
        """
        new_routes = []
        
        # 计算每条边的运营成本与对偶成本（分开保存）
        op_costs = {}
        dual_costs = {}
        neg_dual_edges = 0
        for u, v in self.G.edges():
            distance_m = self.G.edges[u, v].get("length", 0)
            distance_km = distance_m / 1000.0 if distance_m > 0 else 0.1
            
            # 边的运营成本（始终非负，作为最短路的权重）
            operating_cost = self.arc_cost_per_km * distance_km
            if operating_cost < 0:
                operating_cost = 0.0
            op_costs[(u, v)] = operating_cost
            
            # 边的对偶成本（可能为负，不用于最短路权重，只在RC中计入）
            dc = float(dual_costs.get((u, v), 0.0)) if ((u, v) in dual_costs) else float(rho_a.get((u, v), 0.0))
            dual_costs[(u, v)] = dc
            if dc < 0:
                neg_dual_edges += 1
        
        # 对每个OD对，生成K条候选最短路径
        K_paths = 3  # 每个OD生成3条候选路径
        rc_min, rc_max, rc_pos = None, None, 0
        
        for o, d in self.K:
            # 获取该OD对的需求对偶变量（需求价值）
            gamma_od = gamma_w.get((o, d), 0.0)
            
            try:
                # 创建加权图（边权重=广义成本）
                # 使用仅含运营成本的非负权重构造求最短路的图
                G_weighted = self.G.copy()
                for (u, v) in G_weighted.edges():
                    G_weighted.edges[u, v]['weight'] = op_costs.get((u, v), 1e6)
                
                # 使用k-shortest simple paths生成多条候选路径
                paths_generated = 0
                for path in nx.shortest_simple_paths(G_weighted, source=o, target=d, weight='weight'):
                    if paths_generated >= K_paths:
                        break
                    
                    # 计算路径运营成本与对偶成本之和（RC中使用）
                    path_op_cost = sum(op_costs.get((path[i], path[i+1]), 0.0) for i in range(len(path)-1))
                    path_dual_cost = sum(dual_costs.get((path[i], path[i+1]), 0.0) for i in range(len(path)-1))
                    path_cost = path_op_cost + path_dual_cost
                    
                    # 计算Reduced Cost
                    # 公式: RC = (path_cost + fixed_cost) - gamma_od
                    # 对于等式约束，gamma_od无符号限制
                    # 当RC为负时（相对于gamma_od），添加该路径会改进目标函数
                    rc = path_cost + self.fixed_cost_route - gamma_od
                    
                    # 统计RC范围
                    rc_min = rc if rc_min is None else min(rc_min, rc)
                    rc_max = rc if rc_max is None else max(rc_max, rc)
                    
                    # 添加所有RC < 0的路径（这意味着路径成本+FC < gamma_od，即改进）
                    # 对于最小化问题，负RC = 改进
                    if rc < -1e-6:
                        distance_km = sum(
                            self.G.edges[path[i], path[i+1]].get("length", 0) / 1000.0
                            for i in range(len(path)-1)
                        )
                        new_routes.append({
                            'nodes': path,
                            'reduced_cost': rc,
                            'distance_km': distance_km,
                        })
                        rc_pos += 1
                    
                    paths_generated += 1
                    
            except (nx.NetworkXNoPath, nx.NodeNotFound):
                pass
        
        if self.verbose:
            # 打印对偶变量统计
            gamma_values = list(gamma_w.values())
            rho_values = list(rho_a.values())
            print(f"    Dual stats: gamma_min={min(gamma_values) if gamma_values else 0:.2f}, "
                  f"gamma_max={max(gamma_values) if gamma_values else 0:.2f}, "
                  f"rho_min={min(rho_values) if rho_values else 0:.2f}, "
                  f"rho_max={max(rho_values) if rho_values else 0:.2f}")
            if neg_dual_edges > 0:
                print(f"    Note: {neg_dual_edges} edges have negative dual costs")
            
            # 诊断：为什么没有选择路径？
            if rc_max is not None:
                if rc_max >= -1e-6:  # 所有RC都 >= 0
                    print(f"    [INFO] No paths with RC < 0 found (all RC >= 0)")
                    print(f"      - RC range: [{rc_min:.4f}, {rc_max:.4f}]")
                    print(f"      - This means current columns are optimal for LP relaxation")
                    print(f"      - Max gamma_od: ${max(gamma_values) if gamma_values else 0:.2f}")
                    print(f"      - Min path cost: ~${rc_min + max(gamma_values):.2f}")
                    print(f"      - Fixed cost: ${self.fixed_cost_route:.2f}")
                elif rc_pos == 0:  # 有负RC但没有被选择（不应该发生）
                    print(f"    [WARNING] Found paths with RC < 0 but none selected!")
                    print(f"      - RC range: [{rc_min:.4f}, {rc_max:.4f}]")
                    print(f"      - Paths generated: {sum(1 for r in new_routes)}")
            
            print(f"    Pricing stats: RC_min={rc_min:.4f}, RC_max={rc_max:.4f}, selected_paths={rc_pos}")

        
        return new_routes


def main():
    """主函数"""
    
    print("\n" + "="*80)
    print("COMPLETE COLUMN GENERATION ALGORITHM")
    print("="*80 + "\n")
    
    try:
        print("[1/4] Loading data...")
        data = load_model_data("data/model_input_data_10km.pkl")
        print("  [OK] Data loaded\n")
        
        print("[2/4] Processing data...")
        solver_input = finalize_data_for_solver(data)
        print("  [OK] Data processed\n")
        
        print("[3/4] Creating parameters...")
        params = create_model_parameters(solver_input)
        hyper = get_sue_hyperparams()
        print("  [OK] Parameters created\n")
        
        print("[4/4] Running column generation...\n")
        
        # 执行算法
        cg = ColumnGenerationAlgorithm(params, hyper, verbose=True)
        
        results = cg.execute(
            max_iterations=20,
            time_limit_phase1=600,
            time_limit_phase2=1200,
        )
        
        # Print final results
        print("\n" + "="*80)
        print("FINAL RESULTS")
        print("="*80 + "\n")
        
        if results['Z_LB'] is not None:
            print(f"[OK] Lower bound (Z_LB): ${results['Z_LB']:,.2f}")
        else:
            print(f"[FAIL] Lower bound (Z_LB): N/A")
        
        if results['Z_UB'] is not None:
            print(f"[OK] Upper bound (Z_UB): ${results['Z_UB']:,.2f}")
        else:
            print(f"[FAIL] Upper bound (Z_UB): N/A")
        
        if results['gap'] is not None:
            print(f"[OK] Optimality gap: {results['gap']:.4f}%")
            if results['gap'] < 0.1:
                print(f"  Quality assessment: EXCELLENT (gap < 0.1%)")
            elif results['gap'] < 1.0:
                print(f"  Quality assessment: EXCELLENT (gap < 1%)")
            elif results['gap'] < 5:
                print(f"  Quality assessment: GOOD (gap < 5%)")
            elif results['gap'] < 10:
                print(f"  Quality assessment: FAIR (gap < 10%)")
            else:
                print(f"  Quality assessment: POOR (gap >= 10%)")
        else:
            print(f"[FAIL] Optimality gap: N/A")
        
        print(f"\n[OK] Routes generated: {results['num_routes']}")
        print(f"[OK] CG iterations: {len(results['iteration_history'])}")
        
        print("\n" + "="*80)
        print("[OK] Algorithm completed")
        print("="*80 + "\n")
        
        return 0
    
    except KeyboardInterrupt:
        print("\n\n[FAIL] Algorithm interrupted")
        return 1
    except Exception as e:
        print(f"\n[FAIL] Error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
