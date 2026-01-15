# -*- coding: utf-8 -*-
# main.py
import time
import os
import sys
from data_load import load_model_data, finalize_data_for_solver
from model_parameters import create_model_parameters
# 【核心修改】导入新的、职责分明的函数
from rmp_model import build_and_solve_rmp_lp, solve_final_milp
from subproblem_solver import solve_pricing_subproblems
import networkx as nx



def run_column_generation(is_test=True):
    """
    执行完整的列生成算法来解决应急疏散公交网络设计问题。
    """
    start_time = time.time()

    # --- 1. 统一的数据加载和参数准备流程 ---
    print("--- Phase 0: Data Loading and Preparation ---")
    if is_test:
        data_pickle_file = '../data/model_input_data_10km.pkl'
        print("Running in TEST mode.")
    else:
        data_pickle_file = '../data/model_input_data_10km.pkl'
        print("Running in FULL mode.")

    initial_data = load_model_data(data_pickle_file)
    if not initial_data: return

    solver_input = finalize_data_for_solver(initial_data)
    if not solver_input: return

    params = create_model_parameters(solver_input)
    G = params['graph']

    data_prep_time = time.time() - start_time
    print(f"Data preparation completed in {data_prep_time:.2f} seconds.")

    # --- 2. 初始化候选集 (保持您的原始逻辑) ---
    print("\n--- Phase 1: Initialization ---")
    R_hat = {}
    P_hat = {}
    route_id_counter = 0
    path_id_counter = 0

    print("Initializing candidate sets with shortest paths (based on free-flow time)...")
    for o, d in params['K']:
        try:
            init_path_nodes = nx.shortest_path(G, source=o, target=d, weight='free_flow_time')
            R_hat[route_id_counter] = {'nodes': init_path_nodes}
            P_hat[path_id_counter] = {
                'nodes': init_path_nodes,
                'route_id': route_id_counter,
                'od': (o, d)
            }
            route_id_counter += 1
            path_id_counter += 1
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            print(f"Warning: Could not find an initial path for OD=({o},{d}).")
            continue

    if not R_hat:
        print("Error: Could not initialize any routes. Aborting.")
        return

        # --- 3. 列生成迭代循环 ---
    print("\n--- Phase 2: Column Generation Loop ---")
    max_cg_iter = 20
    for iter_num in range(max_cg_iter):
        print(f"\n--- Iteration {iter_num + 1}/{max_cg_iter} ---")
        print(f"Current candidate routes: {len(R_hat)}, Current passenger paths: {len(P_hat)}")

        # --- 3.1 求解 RMP (LP 松弛) ---
        solution, duals, t_a_hat_sol, rmp_vars, obj_components = build_and_solve_rmp_lp(params, R_hat, P_hat)

        if not solution or not duals:
            print("RMP (LP) failed or returned no duals, terminating CG.")
            break

         # a. 打印目标函数分解
        print("Objective Breakdown:")
        calculated_total = 0
        for name, expr in obj_components.items():
            value = solution.get_value(expr)
            print(f"  - {name:<25}: {value:12.2f}")
            calculated_total += value
        print("-------------------------------------------------")
        print(f"  Solver's Objective:                {solution.get_objective_value():12.2f}")
        print("-------------------------------------------------")

        # b. 获取所有需要的变量解
        q_sol = solution.get_value_dict(rmp_vars['q'])
        y_sol = solution.get_value_dict(rmp_vars['y'])
        slack_sol = solution.get_value_dict(rmp_vars['slack_share'])
        x_sol = solution.get_value_dict(rmp_vars['x'])


        # d. 打印非零的松弛变量值
        non_zero_slacks = {key: val for key, val in slack_sol.items() if val > 1e-6}
        if non_zero_slacks:
            print(f"  - WARNING: Found {len(non_zero_slacks)} markets with non-zero slack_share!")
            # (可以添加更详细的打印)
        else:
            print("  - INFO: All slack_share variables are zero.")

        # e. 打印“部分激活”的线路
        activated_routes = {r: val for r, val in x_sol.items() if val > 1e-6}
        if activated_routes:
            print(f"  - Partially Activated Routes (x > 0):")
            for r, val in activated_routes.items():
                print(f"    - x_{r}: {val:.4f}")
        else:
            print("  - No routes are activated in this LP solution.")
        print("------------------------------------------")

        # --- 3.2 求解所有定价子问题 ---
        new_columns = solve_pricing_subproblems(params, duals, t_a_hat_sol, R_hat, P_hat)

        # --- 3.3 检查终止条件 ---
        if not new_columns:
            print("\nConvergence achieved: No new columns with negative reduced cost found.")
            break

        # --- 3.4 添加新列到候选集 (保持不变) ---
        print(f"Adding {len(new_columns)} new columns to the candidate sets...")
        for col_info in new_columns:
            path_nodes = col_info['path_nodes']
            market = col_info['market']
            o, d, h = market
            if col_info['type'] == 'new':
                R_hat[route_id_counter] = {'nodes': path_nodes}
                P_hat[path_id_counter] = {
                    'nodes': path_nodes,
                    'route_id': route_id_counter,
                    'od': (o, d)
                }
                route_id_counter += 1
                path_id_counter += 1
            elif col_info['type'] == 'existing':
                found_route = False
                for r_id, r_data in R_hat.items():
                    if set(path_nodes).issubset(set(r_data['nodes'])):
                        P_hat[path_id_counter] = {
                            'nodes': path_nodes,
                            'route_id': r_id,
                            'od': (o, d)
                        }
                        path_id_counter += 1
                        found_route = True
                        break
                if not found_route:
                    print(
                        f"Warning: An 'existing' type path for OD({o},{d}) could not be associated with any current route. Discarding.")

    else:
        print(f"\nReached max iterations ({max_cg_iter}). Terminating CG loop.")

    # --- 4. 求解最终的 MILP 问题 (保持您的原始逻辑) ---
    print("\n--- Phase 3: Solving Final MILP ---")
    final_solution, final_vars, final_obj_components = solve_final_milp(params, R_hat, P_hat)

    # --- 在这里指定输出路径 ---

    # 1. 定义一个文件夹来存放所有报告
    output_dir = '../reports'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 2. 创建一个唯一的文件名
    # 可以包含时间戳，或者关键参数，来区分不同的实验
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    # 示例：包含 beta_const['B'] 的值在文件名中
    beta_b_val = params['beta_const'].get('B', 'N/A')
    report_filename = f"solution_report_betaB_{beta_b_val}_{timestamp}.txt"

    # 3. 组合成完整的文件路径
    full_report_path = os.path.join(output_dir, report_filename)

    # 4. 调用报告函数，并传入文件路径
    generate_solution_report(params, final_solution, final_vars, final_obj_components, P_hat,
                             output_filepath=full_report_path)

    # --- 5. 【最终修复】结果报告部分 ---
    if not final_solution:
        print("\nFinal MILP could not be solved. Check logs for details.")
    else:
        print("\n" + "=" * 50)
        print("--- FINAL OPTIMAL SOLUTION ---")
        print(f"Total Societal Cost: {final_solution.get_objective_value():.2f}")
        print("=" * 50)

        # --- 【核心修改】全新的、简单的最终结果报告 ---
        print("\n--- Objective Function Breakdown ---")
        for name, expr in final_obj_components.items():
            value = final_solution.get_value(expr)
            print(f"  - {name:<25}: {value:12.2f}")

        print("\n--- Operated Bus Routes and Headways ---")
        operated_routes = 0
        # 正确地获取 x 和 w 的值
        x_sol = final_solution.get_value_dict(final_vars['x'])
        w_sol = final_solution.get_value_dict(final_vars['w'])

        for r, x_val in x_sol.items():
            if x_val > 0.99:
                operated_routes += 1
                chosen_headway = "None"
                for k, h_val in params['H_k'].items():
                    if w_sol.get((r, k), 0) > 0.99:
                        chosen_headway = f"{h_val * 60:.0f} mins"
                        break
                print(f"  - Route {r}: Activated with headway {chosen_headway}")

        if operated_routes == 0:
            print("  - No bus routes were operated.")

    end_time = time.time()
    print(f"\nTotal execution time: {end_time - start_time:.2f} seconds.")


def generate_solution_report(params, solution, all_vars, obj_components, P_hat, output_filepath=None):
    """
    【v3 最终版】
    生成一份全面的实验结果报告，包含输入设定、成本、决策、行为和网络性能。
    专为批量实验和导出到Excel设计。
    """
    original_stdout = sys.stdout
    if output_filepath:
        try:
            f = open(output_filepath, 'w', encoding='utf-8')
            sys.stdout = f
            print(f"--- Solution report is being written to: {output_filepath} ---")
        except IOError as e:
            print(f"Error opening file {output_filepath}. Report will be printed to console. Error: {e}")
            sys.stdout = original_stdout

    if not solution:
        print("\n" + "=" * 80 + "\n--- NO SOLUTION FOUND, CANNOT GENERATE REPORT ---\n" + "=" * 80)
    else:
        # ==============================================================================
        # 1. 输入设定总结 (INPUT SUMMARY)
        # ==============================================================================
        print("\n" + "=" * 80)
        print("--- EXPERIMENT SUMMARY REPORT ---")
        print("=" * 80)
        print("\n1. INPUT PARAMETERS & MODEL SCOPE")
        print("-" * 40)
        total_demand = sum(params['D_odh'].values())
        print(f"  - {'Total OD Pairs':<30}: {len(params['K'])}")
        print(f"  - {'Total Demand':<30}: {total_demand:,.0f} passengers")
        print(
            f"  - {'Total Candidate Bus Routes':<30}: {len(params.get('initial_R_hat', all_vars['x']))}")  # 假设params中有initial_R_hat
        print(f"  - {'Total Generated Bus Paths':<30}: {len(P_hat)}")

        # ==============================================================================
        # 2. 总体性能: 成本分析 (OVERALL PERFORMANCE)
        # ==============================================================================
        print("\n2. OVERALL PERFORMANCE: COST ANALYSIS")
        print("-" * 40)
        print(f"  - {'TOTAL SOCIETAL COST':<30}: {solution.get_objective_value():,.2f}")
        for name, component in obj_components.items():
            value = solution.get_value(component)
            print(f"    - {name:<28}: {value:15,.2f}")

        # ==============================================================================
        # 3. 系统决策: 公交运营 (SYSTEM DECISIONS)
        # ==============================================================================
        print("\n3. SYSTEM DECISIONS: BUS OPERATIONS")
        print("-" * 40)
        x_sol = solution.get_value_dict(all_vars['x'])
        w_sol = solution.get_value_dict(all_vars['w'])
        activated_routes = {r for r, val in x_sol.items() if val > 0.99}
        if not activated_routes:
            print("  - No bus routes were operated.")
        else:
            print(f"  - {'Activated Routes':<30}: {len(activated_routes)}")
            print(f"    {'Route ID':<15} | {'Headway (mins)':<20}")
            print("    " + "-" * 40)
            for r in sorted(list(activated_routes)):
                headway_min_str = "N/A"
                for k, h_hr in params['H_k'].items():
                    if w_sol.get((r, k), 0) > 0.99:
                        headway_min_str = f"{h_hr * 60:.1f}"
                        break
                print(f"    {r:<15} | {headway_min_str:<20}")

        # ==============================================================================
        # 4. 出行行为结果: 需求分布 (BEHAVIORAL OUTCOMES)
        # ==============================================================================
        print("\n4. BEHAVIORAL OUTCOMES: DEMAND DISTRIBUTION")
        print("-" * 40)
        q_sol = solution.get_value_dict(all_vars['q'])
        y_sol = solution.get_value_dict(all_vars['y'])
        demand_per_mode = {m: 0 for h in params['M_h'].values() for m in h}

        for (o, d, h, m), share in q_sol.items():
            if share > 1e-6: demand_per_mode[m] += params['D_odh'].get((o, d, h), 0) * share
        for (i, h), share in y_sol.items():
            if share > 1e-6:
                path_od = P_hat[i]['od']
                demand_per_mode['B'] += params['D_odh'].get((path_od[0], path_od[1], h), 0) * share

        print(f"  {'Mode':<15} | {'Passengers':>15} | {'Share (%)':>12}")
        print("  " + "-" * 50)
        sorted_modes = sorted(demand_per_mode.items(), key=lambda item: item[1], reverse=True)
        for mode, count in sorted_modes:
            if count > 1e-6:
                percentage = (count / total_demand) * 100 if total_demand > 0 else 0
                print(f"  {mode:<15} | {count:>15,.2f} | {percentage:>11.2f}%")

        # ==============================================================================
        # 5. 网络性能: 拥堵分析 (NETWORK PERFORMANCE)
        # ==============================================================================
        print("\n5. NETWORK PERFORMANCE: CONGESTION ANALYSIS")
        print("-" * 40)
        t_a_hat_sol = solution.get_value_dict(all_vars['t_a_hat'])
        v_a_auto_sol = solution.get_value_dict(all_vars['v_a_auto'])
        delays = []
        for a in params['A']:  # Iterate over all links in the network
            t_hat = t_a_hat_sol.get(a, 0)
            t0_sec = params['graph'].edges[a].get('free_flow_time', 0)
            if t0_sec == 0: continue

            t0_hr = t0_sec / 3600.0
            delay_sec = (t_hat - t0_hr) * 3600

            if delay_sec > 1.0:
                v_auto_val = v_a_auto_sol.get(a, 0)
                capacity_val = params['graph'].edges[a].get('capacity', 1)
                v_c_ratio = v_auto_val / capacity_val if capacity_val > 0 else 0
                delays.append({'link': a, 'delay_sec': delay_sec, 'v_c_ratio': v_c_ratio})

        if not delays:
            print("  - No significant congestion detected on any links.")
        else:
            avg_delay = sum(d['delay_sec'] for d in delays) / len(delays) if delays else 0
            max_delay = max(d['delay_sec'] for d in delays) if delays else 0
            print(f"  - {'Congested Links (>1s delay)':<30}: {len(delays)}")
            print(f"  - {'Average Delay on Congested Links':<30}: {avg_delay:.2f} seconds")
            print(f"  - {'Maximum Delay on a Single Link':<30}: {max_delay:.2f} seconds")

            sorted_delays = sorted(delays, key=lambda x: x['delay_sec'], reverse=True)
            print("\n  - Top 5 Most Congested Links:")
            print(f"    {'Link':<25} | {'Delay (seconds)':>18} | {'V/C Ratio':>12}")
            print("    " + "-" * 60)
            for item in sorted_delays[:5]:
                print(f"    {str(item['link']):<25} | {item['delay_sec']:>18.2f} | {item['v_c_ratio']:>11.2f}")

        print("\n" + "=" * 80)
        print("--- END OF REPORT ---")
        print("=" * 80)

    if output_filepath:
        sys.stdout = original_stdout
        f.close()
        print(f"Report successfully saved to {output_filepath}")

if __name__ == '__main__':
    run_column_generation(is_test=True)