# -*- coding: utf-8 -*-
# rmp_model.py
from docplex.mp.model import Model
from docplex.mp.vartype import VarType
import time
import numpy as np
from docplex.mp.conflict_refiner import ConflictRefiner, VarUbConstraintWrapper, VarLbConstraintWrapper


# --- 效用分析函数 (LP阶段) ---
def analyze_lp_utilities(params, solution, all_vars, P_hat):
    """
    在RMP LP求解后，分析并打印各个出行模式的系统效用 (Utility)。
    """
    print("\n" + "-" * 25 + " LP SOLUTION UTILITY ANALYSIS " + "-" * 25)

    # 从LP解中获取关键变量的值
    u_od_auto_sol = solution.get_value_dict(all_vars['u_od_auto'])
    q_sol = solution.get_value_dict(all_vars['q'])

    # 找到份额 > 0 的市场进行分析
    sample_markets = []
    analyzed_markets = set()
    for (o, d, h, m), share in q_sol.items():
        market = (o, d, h)
        if share > 0.001 and market not in analyzed_markets:
            sample_markets.append(market)
            analyzed_markets.add(market)
            if len(sample_markets) >= 3:  # 最多分析3个市场
                break

    # 如果没找到，就用日志中那个市场
    if not sample_markets:
        market_to_find = (979, 7282, 'h1')
        if market_to_find in params['D_odh'] and params['D_odh'][market_to_find] > 0:
            sample_markets.append(market_to_find)

    if not sample_markets:
        print("No active markets with significant shares found to analyze.")
        print("-" * 78)
        return

    for market in sample_markets:
        o, d, h = market
        print(f"\n--- Analyzing Market: OD=({o}, {d}), Pop={h} ---")

        utilities = {}

        # 1. 内生汽车模式 (D, X)
        u_auto_sec = u_od_auto_sol.get((o, d), 0)
        for m in ['D', 'X']:
            if (o, d, h, m) in all_vars['q']:  # 检查模式是否存在
                V_mode = (params['beta_TT_m'][m] * (u_auto_sec / 3600.0) +
                          params['beta_const'].get(m, 0))
                utilities[m] = V_mode

        # 2. 外生模式 (R, W, O)
        for m in ['R', 'W', 'O']:
            if (o, d, h, m) in params['V_bar_odhm']:
                utilities[m] = params['V_bar_odhm'][(o, d, h, m)]

        # 3. 内生公交模式 (B) - 估算一个潜在的效用
        # 对于RMP中已存在的公交路径
        existing_bus_paths_in_market = [i for i, data in P_hat.items() if data['od'] == (o, d)]

        # 为了简化，我们还是用一个通用的估算
        # 假设公交时间和汽车一样，等待时间为平均发车间隔的一半
        avg_headway_hr = np.mean(list(params['H_k'].values())) / 3600.0 if params['H_k'] else 1.0
        potential_WT_hr = 0.5 * avg_headway_hr
        potential_TT_hr = u_auto_sec / 3600.0  # 乐观假设：公交和汽车一样快

        V_bus_potential = (params['beta_TT_m']['B'] * potential_TT_hr +
                           params['beta_WT'] * potential_WT_hr +
                           params['beta_const'].get('B', 0))
        utilities['B (potential)'] = V_bus_potential

        # 打印结果
        print("Mode Utilities (V) based on current LP solution:")
        sorted_utilities = sorted(utilities.items(), key=lambda item: item[1], reverse=True)
        for mode, util in sorted_utilities:
            print(f"  - V_{mode:<14}: {util:8.4f}")

    print("-" * 78)


def debug_background_cost_calculation(params, solution, all_vars,obj_components):
    """
    在求解后，手动重新计算 Z_bg，并打印详细的诊断信息。
    """
    print("\n" + "#" * 25 + " Z_bg POST-SOLVE DIAGNOSTICS " + "#" * 25)

    # 获取解中的 t_a_hat 值
    t_a_hat_sol = solution.get_value_dict(all_vars['t_a_hat'])

    # 获取参数
    A = params['A']
    bar_v_a = params['bar_v_a']
    G = params['graph']
    cost_factor = params['VOT_bg'] + params['C_op_env_hat']['bg']

    total_calculated_z_bg = 0
    link_costs = []

    num_links_with_bar_v_a = 0
    num_links_with_delay = 0

    for a in A:
        # 提取所需的值
        t_hat_val = t_a_hat_sol.get(a, 0)
        # 【关键】从 G.edges 的属性中获取 t0，确保单位是小时
        t0_val = G.edges[a].get('free_flow_time', 0) / 3600.0
        bar_v_a_val = bar_v_a.get(a, 0)

        if bar_v_a_val > 0:
            num_links_with_bar_v_a += 1

            delay = t_hat_val - t0_val

            # 只有在延误大于一个很小的阈值时才计算
            if delay > 1e-6:
                num_links_with_delay += 1
                link_cost = bar_v_a_val * delay * cost_factor
                total_calculated_z_bg += link_cost
                link_costs.append({'link': a, 'cost': link_cost, 'delay_sec': delay * 3600, 'bar_v_a': bar_v_a_val})

    print(f"  - Cost Factor (VOT_bg + C_op_env_hat_bg): {cost_factor:.2f}")
    print(f"  - Total links in set 'A': {len(A)}")
    print(f"  - Links with non-zero 'bar_v_a' in params: {num_links_with_bar_v_a}")
    print(f"  - Links with non-zero delay (t_hat > t0): {num_links_with_delay}")
    print("-" * 78)
    print(f"  - Manually Calculated Z_bg: {total_calculated_z_bg:.2f}")

    # 从 obj_components 获取模型报告的 Z_bg
    # 注意: 这需要 solution.get_objective_value() 能工作，但我们可以直接从表达式获取
    reported_z_bg = solution.get_value(obj_components['Background_Traffic_Cost'])
    print(f"  - Model's Reported Z_bg:    {reported_z_bg:.2f}")
    print("-" * 78)

    if num_links_with_delay > 0:
        print("  - Top 5 Links Contributing to Z_bg:")
        # 按成本从高到低排序
        sorted_links = sorted(link_costs, key=lambda x: x['cost'], reverse=True)
        for item in sorted_links[:5]:
            print(
                f"    - Link {item['link']}: Cost={item['cost']:.2f} (Delay={item['delay_sec']:.2f}s, bar_v_a={item['bar_v_a']:.2f})")
    else:
        print("  - CRITICAL FINDING: No links experienced any significant delay.")
        print("  - This confirms that the congestion feedback loop is broken.")
        print("  - The most likely cause remains a key mismatch in the 'v_a_auto' calculation.")

    print("#" * 78)

# --- 【新增】需求分布汇总函数 ---
def summarize_demand_distribution(params, solution, all_vars, P_hat):
    """
    计算并打印模型解中，总需求在不同出行模式间的分布情况。
    """
    print("\n" + "-" * 25 + " DEMAND DISTRIBUTION SUMMARY " + "-" * 25)

    q_sol = solution.get_value_dict(all_vars['q'])
    y_sol = solution.get_value_dict(all_vars['y'])

    demand_per_mode = {m: 0 for h in params['M_h'].values() for m in h}
    total_demand = sum(params['D_odh'].values())

    # 1. 计算非公交模式的需求
    for (o, d, h, m), share in q_sol.items():
        if share > 1e-6:
            demand_value = params['D_odh'].get((o, d, h), 0) * share
            demand_per_mode[m] += demand_value

    # 2. 计算公交模式的需求
    for (i, h), share in y_sol.items():
        if share > 1e-6:
            path_od = P_hat[i]['od']
            demand_value = params['D_odh'].get((path_od[0], path_od[1], h), 0) * share
            demand_per_mode['B'] += demand_value

    print(f"Total Demand: {total_demand:.2f} passengers")
    print("-" * 78)
    print(f"{'Mode':<15} | {'Passengers':>15} | {'Share':>10}")
    print("-" * 78)

    # 按照乘客数排序打印
    sorted_modes = sorted(demand_per_mode.items(), key=lambda item: item[1], reverse=True)

    for mode, count in sorted_modes:
        if count > 1e-6:
            percentage = (count / total_demand) * 100 if total_demand > 0 else 0
            print(f"{mode:<15} | {count:>15.2f} | {percentage:>9.2f}%")

    print("-" * 78)

# --- 内部核心构建函数 (不对外直接调用) ---
def _build_rmp_common(params, R_hat, P_hat, integer_mode):
    """
    一个内部函数，用于构建RMP的核心逻辑，根据 argument 决定变量类型。
    """
    mdl = Model(name=f"EvacuationTransitRMP_{'MILP' if integer_mode else 'LP'}")


    # 提取参数和集合...
    A, H, K, D_odh, VOT_h = params['A'], params['H'], params['K'], params['D_odh'], params['VOT_h']
    r_indices, k_indices, path_indices = list(R_hat.keys()), list(params['J_r'].keys()), list(P_hat.keys())

    # 预处理...
    link_to_routes_map, link_to_paths_map, od_to_paths_map, link_to_auto_path_map, auto_path_ids = {}, {}, {}, {}, []
    for r_id, r_data in R_hat.items():
        for u, v in zip(r_data['nodes'][:-1], r_data['nodes'][1:]):
            link_to_routes_map.setdefault((u, v), []).append(r_id)
    for p_id, p_data in P_hat.items():
        for u, v in zip(p_data['nodes'][:-1], p_data['nodes'][1:]):
            link_to_paths_map.setdefault((u, v), []).append(p_id)
    for p_id, p_data in P_hat.items():
        od_to_paths_map.setdefault(p_data['od'], []).append(p_id)
    for (o, d), paths in params['P_auto_od'].items():
        for p_idx, path_nodes in enumerate(paths):
            path_id = ((o, d), p_idx)
            auto_path_ids.append(path_id)
            for u, v in zip(path_nodes[:-1], path_nodes[1:]):
                link_to_auto_path_map.setdefault((u, v), []).append(path_id)

    # --- 3. 定义决策变量 ---
    # 定义 w_keys (无论模式如何都需要)
    w_keys = [(r, k) for r in r_indices for k in k_indices]

    if integer_mode:
        print("Defining variables x, w, z in BINARY mode.")

        # 使用 binary_var_dict：默认 lb=0, ub=1，类型为 Binary
        x = mdl.binary_var_dict(r_indices, name='x')
        w = mdl.binary_var_dict(w_keys, name='w')
        z = mdl.binary_var_dict(auto_path_ids, name='z')

    else:
        print("Defining variables x, w, z in CONTINUOUS mode (0 to 1).")

        # 使用 continuous_var_dict：需要明确设置 lb=0, ub=1
        x = mdl.continuous_var_dict(r_indices, lb=0, ub=1, name='x')
        w = mdl.continuous_var_dict(w_keys, lb=0, ub=1, name='w')
        z = mdl.continuous_var_dict(auto_path_ids, lb=0, ub=1, name='z')

    q_keys = [(o, d, h, m) for o, d in K for h in H for m in params['M_h'][h] if m != 'B']
    q = mdl.continuous_var_dict(q_keys, lb=0, ub=1, name='q')

    y_keys = [(i, h) for i in path_indices for h in H]
    y = mdl.continuous_var_dict(y_keys, lb=0, ub=1, name='y')

    LOG_SHARE_LOWER_BOUND = -1000.0
    lq = mdl.continuous_var_dict(q.keys(), name='lq', ub=0, lb=LOG_SHARE_LOWER_BOUND)
    ly = mdl.continuous_var_dict(y.keys(), name='ly', ub=0, lb=LOG_SHARE_LOWER_BOUND)
    g = mdl.continuous_var_dict(auto_path_ids, lb=0, name='g')
    v_a_auto = mdl.continuous_var_dict(A, lb=0, name='v_auto')
    t_a_hat = mdl.continuous_var_dict(A, lb=0, name='t_hat')
    u_od_auto = mdl.continuous_var_dict(K, lb=0, name='u_auto')
    TT_i = mdl.continuous_var_dict(path_indices, name='TT')
    WT_i = mdl.continuous_var_dict(path_indices, name='WT')

    slack_share_keys = [(o, d, h) for o, d in K for h in H if (o, d, h) in D_odh]
    slack_share = mdl.continuous_var_dict(slack_share_keys, name='slack_share')
    abs_slack_share = mdl.continuous_var_dict(slack_share_keys, lb=0, name='abs_slack_share')

    # --- 【新增】为线性化定义辅助变量 (Zeta Variables) ---
    zeta_qu_keys = [(o, d, h, m) for (o, d, h, m) in q_keys if m in ['D', 'X']]
    zeta_qu = mdl.continuous_var_dict(zeta_qu_keys, lb=0, name='zeta_qu')

    # y*TT and y*WT have the same keys as y
    zeta_yTT = mdl.continuous_var_dict(y_keys, lb=0, name='zeta_yTT')
    zeta_yWT = mdl.continuous_var_dict(y_keys, lb=0, name='zeta_yWT')

    # --- 4. 定义约束 ---
    constraints = {}

    constraints['tt_definition'] = {}
    for i in path_indices:
        # 获取路径 i 包含的所有路段
        path_data = P_hat[i]
        path_nodes = path_data['nodes']
        # 将节点列表转换为路段 (edge) 列表
        path_links = list(zip(path_nodes[:-1], path_nodes[1:]))

        # 定义 TT_i 等于路径上所有路段 t_a_hat 的和
        # 注意 t_a_hat 的单位是小时，所以 TT_i 的单位也是小时
        path_travel_time_expr = mdl.sum(t_a_hat[a] for a in path_links)

        # 添加约束
        constraints['tt_definition'][i] = mdl.add_constraint(
            TT_i[i] == path_travel_time_expr,
            ctname=f"def_TT_{i}"
        )
    # --- TT_i 定义结束 ---

    # --- 定义 WT_i (等待时间) 的约束 ---
    constraints['wt_definition'] = {}
    for i in path_indices:
        route_id = P_hat[i]['route_id']
        headway_expr = mdl.sum(params['H_k'][k] * w.get((route_id, k), 0) for k in k_indices)
        constraints['wt_definition'][i] = mdl.add_constraint(
            WT_i[i] == 0.5 * headway_expr,
            ctname=f"def_WT_{i}"
        )
    # --- WT_i 定义结束 ---

    if params.get('MIN_OTHER_SHARE', 0) > 0:
        min_other_share_val = params['MIN_OTHER_SHARE']
        print(f"Applying minimum share constraint for 'O' mode: {min_other_share_val * 100:.4f}%")

        # 遍历所有 q 变量的键
        for key in q_keys:
            # key 的格式是 (o, d, h, m)
            if key[3] == 'O':  # 如果模式是 'O'
                # 为这个 q[key] 变量添加一个大于等于约束
                mdl.add_constraint(q[key] >= min_other_share_val,
                                   ctname=f"min_share_O_{key[0]}_{key[1]}_{key[2]}")

    constraints['headway_selection'] = {
        r: mdl.add_constraint(mdl.sum(w.get((r, k), 0) for k in k_indices) == x[r], ctname=f"headway_sel_{r}") for r in
        r_indices}
    constraints['path_route_linking'] = {
        (i, h): mdl.add_constraint(y[i, h] <= x[P_hat[i]['route_id']], ctname=f"link_y{i}_x{P_hat[i]['route_id']}") for
        i, h in y_keys}
    constraints['bus_capacity'] = {a: mdl.add_constraint(
        mdl.sum(D_odh.get((P_hat[i]['od'][0], P_hat[i]['od'][1], h), 0) * y.get((i, h), 0) for i in
                link_to_paths_map.get(a, []) for h in H) <=
        mdl.sum(params['C_B'] / params['H_k'][k] * w.get((r, k), 0) for r in link_to_routes_map.get(a, []) for k in
                k_indices),
        ctname=f"cap_{a[0]}_{a[1]}"
    ) for a in A}


    constraints['share_conservation'] = {}
    for o, d in K:
        for h in H:
            if (o, d, h) in D_odh and D_odh[(o, d, h)] > 0:
                bus_share = mdl.sum(y[i, h] for i in od_to_paths_map.get((o, d), []))
                other_modes_share = mdl.sum(q[o, d, h, m] for m in params['M_h'][h] if m != 'B')
                constraints['share_conservation'][(o, d, h)] = mdl.add_constraint(
                    bus_share + other_modes_share + slack_share[o, d, h] == 1, ctname=f"share_{o}_{d}_{h}")
    for key in slack_share_keys:
        mdl.add_constraint(abs_slack_share[key] >= slack_share[key])
        mdl.add_constraint(abs_slack_share[key] >= -slack_share[key])

    for c_idx, s_c in enumerate(params['C_log']):
        ls_c, ls_prime_c = params['LS_c'][c_idx], params['LS_prime_c'][c_idx]
        for key in q: mdl.add_constraint(lq[key] <= ls_c + ls_prime_c * (q[key] - s_c))
        for key in y: mdl.add_constraint(ly[key] <= ls_c + ls_prime_c * (y[key] - s_c))

    constraints['sblp_paths'] = {}
    # --- 为所有非公交模式（内生和外生）构建 SBLP 约束 ---
    for o, d, h in D_odh:
        if D_odh[o, d, h] <= 0: continue

        lq_base = lq[o, d, h, 'O']
        V_base = params['V_bar_odhm'][o, d, h, 'O']

        # 1. 内生汽车模式 (D, X)
        for m in ['D', 'X']:
            if (o, d, h, m) in q:
                beta_tt_specific = params['beta_TT_m'][m]
                V_mode_expr = beta_tt_specific * (u_od_auto[o, d] / 3600.0) + params['beta_const'][m]
                mdl.add_constraint(lq[o, d, h, m] - lq_base <= V_mode_expr - V_base,
                                   ctname=f"sblp_mode_{o}_{d}_{h}_{m}_eq")

        # 2. 外生模式 (R, W)
        for m in ['R', 'W']:
            if (o, d, h, m) in q:
                V_mode_param = params['V_bar_odhm'][o, d, h, m]
                mdl.add_constraint(lq[o, d, h, m] <= V_mode_param - V_base + lq_base,
                                   ctname=f"sblp_mode_{o}_{d}_{h}_{m}_eq")

    # 3. 内生公交模式 (B)
    for i in path_indices:
        path_data = P_hat[i]
        o_path, d_path = path_data['od']
        beta_tt_bus = params['beta_TT_m']['B']
        V_path_expr = beta_tt_bus * TT_i[i] + params['beta_WT'] * WT_i[i] +  params['beta_TC'] * 2 +params['beta_const']['B']

        for h in H:
            if (i, h) in y:
                lq_base = lq[o_path, d_path, h, 'O']
                V_base = params['V_bar_odhm'][o_path, d_path, h, 'O']

                constraints['sblp_paths'][(i, h)] = mdl.add_constraint(
                    ly[i, h] <= V_path_expr - V_base + lq_base,
                    ctname=f"sblp_path_{i}_{h}"
                )

    for a in A: mdl.add_constraint(v_a_auto[a] == mdl.sum(g[p_id] for p_id in link_to_auto_path_map.get(a, [])))
    for o, d in K:
        auto_paths = [((o, d), i) for i in range(len(params['P_auto_od'].get((o, d), [])))]
        auto_demand = mdl.sum(D_odh.get((o, d, h), 0) * q.get((o, d, h, m), 0) for h in H for m in ['D', 'X'])
        mdl.add_constraint(mdl.sum(g[p_id] for p_id in auto_paths) == auto_demand)

    for u, v, data in params['graph'].edges(data=True):
        a = (u, v)
        for b_idx in range(len(params['B_bpr'])):
            v_ab = params['B_bpr'][b_idx] * data['capacity']
            T_ab, T_prime_ab = params['T_ab'][(a, b_idx)], params['T_prime_ab'][(a, b_idx)]
            mdl.add_constraint(t_a_hat[a] >= T_ab + T_prime_ab * (v_a_auto[a] - v_ab))

    for p_id in auto_path_ids:
        (o, d), p_idx = p_id
        path_nodes = params['P_auto_od'][(o, d)][p_idx]
        path_links = list(zip(path_nodes[:-1], path_nodes[1:]))
        path_tt = mdl.sum(t_a_hat[a] for a in path_links)
        mdl.add_constraint(path_tt* 3600.0 - u_od_auto.get((o, d), 0) >= 0)
        mdl.add_constraint(path_tt* 3600.0 - u_od_auto.get((o, d), 0) <= params['M'] * z[p_id])
        mdl.add_constraint(g[p_id] <= params['M'] * (1 - z[p_id]))

    # --- McCormick Relaxation Constraints ---
    # 4.1. For q * u_auto
    for key in zeta_qu_keys:
        o, d, h, m = key
        q_var = q[key]
        u_var_sec = u_od_auto[(o, d)]
        zeta_var = zeta_qu[key]

        U_L_sec = params['U_auto_L'][(o, d)]
        U_U_sec = params['U_auto_U'][(o, d)]

        # Naming convention: Mc_qu_[o]_[d]_[h]_[m]_[index]
        name_base = f"Mc_qu_{o}_{d}_{h}_{m}"

        mdl.add_constraint(zeta_var >= U_L_sec * q_var,
                           ctname=f"{name_base}_1")
        mdl.add_constraint(zeta_var >= u_var_sec + U_U_sec * (q_var - 1),
                           ctname=f"{name_base}_2")
        mdl.add_constraint(zeta_var <= U_U_sec * q_var,
                           ctname=f"{name_base}_3")
        mdl.add_constraint(zeta_var <= u_var_sec + U_L_sec * (q_var - 1),
                           ctname=f"{name_base}_4")

    # 4.2. For y * WT
    WT_L_hr = 0.0
    WT_U_hr = params['WT_U']
    for key in y_keys:
        i, h = key
        y_var = y[key]
        wt_var_hr = WT_i[i]
        zeta_var = zeta_yWT[key]

        name_base = f"Mc_yWT_{i}_{h}"

        # Constraint 1: zeta >= L * y (L=0)
        mdl.add_constraint(zeta_var >= 0,
                           ctname=f"{name_base}_1")
        # Constraint 2: zeta >= u + U * (y - 1)
        mdl.add_constraint(zeta_var >= wt_var_hr + WT_U_hr * (y_var - 1),
                           ctname=f"{name_base}_2")
        # Constraint 3: zeta <= U * y
        mdl.add_constraint(zeta_var <= WT_U_hr * y_var,
                           ctname=f"{name_base}_3")
        # Constraint 4: zeta <= u + L * (y - 1) (L=0, simplified to zeta <= u)
        mdl.add_constraint(zeta_var <= wt_var_hr,
                           ctname=f"{name_base}_4")

    # 4.3. For y * TT
    G = params['graph']
    TT_i_L, TT_i_U = {}, {}
    for i in path_indices:
        path_nodes = P_hat[i]['nodes']
        path_links = zip(path_nodes[:-1], path_nodes[1:])
        # Sum of free_flow_time (seconds), then convert to hours
        lower_bound_sec = sum(G.edges[link].get('free_flow_time', 0) for link in path_links)
        TT_i_L[i] = lower_bound_sec / 3600.0
        TT_i_U[i] = 30 * TT_i_L[i]

    for key in y_keys:
        i, h = key
        y_var, tt_var_hr, zeta_var = y[key], TT_i[i], zeta_yTT[key]
        tt_L, tt_U = TT_i_L[i], TT_i_U[i]
        name_base = f"Mc_yTT_{i}_{h}"
        mdl.add_constraint(zeta_var >= tt_L * y_var, ctname=f"{name_base}_1")
        mdl.add_constraint(zeta_var >= tt_var_hr + tt_U * (y_var - 1), ctname=f"{name_base}_2")
        mdl.add_constraint(zeta_var <= tt_U * y_var, ctname=f"{name_base}_3")
        mdl.add_constraint(zeta_var <= tt_var_hr + tt_L * (y_var - 1), ctname=f"{name_base}_4")

    # --- 5. 定义目标函数 ---
    # --- Component 1: System Operator Costs (Z_sys_op) ---
    Z_sys_op_bus = mdl.sum(params['FC_r'] * x[r] for r in r_indices) + \
                   mdl.sum(params['C_op_rk_template'][k] * w[r, k] for r, k in w.keys())
    Z_sys_op_auto = mdl.sum(D_odh.get((o, d, h), 0) * params['C_op_env_hat'][m] * (zeta_qu[o, d, h, m] / 3600.0)
                            for (o, d, h, m) in zeta_qu_keys)
    Z_sys_op = Z_sys_op_bus + Z_sys_op_auto

    # --- Component 2: User Costs (Z_user) ---
    # Time-based user cost for auto (D/X)
    Z_user_auto = mdl.sum(D_odh.get((o, d, h), 0) * VOT_h[h] * (zeta_qu.get((o, d, h, m), 0) / 3600.0)
                          for (o, d, h, m) in zeta_qu_keys)
    # Monetary user cost for auto (distance-based per OD)
    Z_user_auto_fee = mdl.sum(
        params['D_odh'].get((o, d, h), 0) * q[o, d, h, m] *
        params['cost_coeff_auto'].get(m, 0) * params['Cost_od_auto'].get((o, d, m), 0)
        for o, d, h, m in q.keys() if m in ['D', 'X']
    )
    Z_user_bus = mdl.sum(D_odh.get((P_hat[i]['od'][0], P_hat[i]['od'][1], h), 0) * VOT_h[h] * \
                         (zeta_yTT.get((i, h), 0) + zeta_yWT.get((i, h), 0))
                         for i, h in y_keys)
    # Exogenous modes (R, W): use fixed utility V_bar (includes time + fare) -> user cost = -VOT * V_bar
    Z_user_exog = mdl.sum(
        params['D_odh'].get((o, d, h), 0) * q[o, d, h, m] *
        (- VOT_h[h] * params['V_bar_odhm'].get((o, d, h, m), 0))
        for o, d, h, m in q.keys() if m in ['R', 'W']
    )

    Z_user = Z_user_auto + Z_user_auto_fee + Z_user_bus + Z_user_exog

    Z_bg = mdl.sum(
        params['bar_v_a'].get(a, 0) * \
        (t_a_hat[a] - (params['graph'].edges[a]['free_flow_time'] / 3600.0)) * \
        (params['VOT_bg'] + params['C_op_env_hat']['bg'])
        for a in A #if params['bar_v_a'].get(a, 0) > 0
    )

    # --- Component 4: Penalty Costs ---
    BIG_PENALTY = 1e6
    penalty_cost = mdl.sum(abs_slack_share[key] * BIG_PENALTY for key in slack_share_keys)
    OTHER_MODE_PENALTY = 1e3
    penalty_other_mode = mdl.sum(
        params['D_odh'].get((o, d, h), 0) * q[o, d, h, 'O'] * OTHER_MODE_PENALTY
        for o, d, h, m in q_keys if m == 'O'
    )

    # --- 【新增】将所有组件打包到一个字典中 ---
    obj_components = {
        "System_Operator_Cost": Z_sys_op,
        "User_Cost": Z_user,
        "Background_Traffic_Cost": Z_bg,
        "Slack_Penalty": penalty_cost,
        "Other_Mode_Penalty": penalty_other_mode
    }

    # --- 最终目标函数 ---
    # 使用打包好的组件来构建总目标，确保一致性
    mdl.minimize(mdl.sum(obj_components.values()))

    # 【新增】导出模型文件以便离线分析

    lp_filename = f"debug_rmp_iter{time.time():.0f}.lp"
    print(f"Exporting model to file: {lp_filename}")
    mdl.export_as_lp(lp_filename)

    all_vars = {
        # 核心决策变量
        'x': x,
        'w': w,
        'z': z,

        # 份额变量
        'q': q,
        'y': y,

        # 流量/时间变量
        'g': g,
        'v_a_auto': v_a_auto,
        't_a_hat': t_a_hat,
        'u_od_auto': u_od_auto,
        'TT_i': TT_i,
        'WT_i': WT_i,

        # 辅助/松弛变量
        'lq': lq,
        'ly': ly,
        'slack_share': slack_share,
        'abs_slack_share': abs_slack_share,
        'zeta_qu': zeta_qu,
        'zeta_yTT': zeta_yTT,
        'zeta_yWT': zeta_yWT
    }

    return mdl, constraints, all_vars, obj_components, P_hat


# --- 公开函数 1: 用于列生成循环 ---
def build_and_solve_rmp_lp(params, R_hat, P_hat):
    start_time = time.time()
    mdl, constraints, all_vars, obj_components, P_hat_internal = _build_rmp_common(params, R_hat, P_hat,
                                                                                   integer_mode=False)
    build_time = time.time() - start_time
    print(f"RMP LP built in {build_time:.2f} seconds.")

    solution = mdl.solve(log_output=False)

    # --- 【新逻辑】正确处理不同的求解状态 ---
    solve_status = mdl.get_solve_status()

    if solve_status is None or solve_status.name not in ["OPTIMAL_SOLUTION", "FEASIBLE_SOLUTION"]:
        print(f"\n--- RMP LP SOLVE FAILED or DID NOT FIND AN OPTIMAL SOLUTION ---")
        print(f"Solve Status: {solve_status.name if solve_status else 'No Solution Found'}")

        # 只有在确实不可行时才调用冲突分析器
        if solve_status and "INFEASIBLE" in solve_status.name:
            print("\n" + "=" * 50)
            print("INVOKING CONFLICT REFINER...")
            cref = ConflictRefiner()
            conflict_result = cref.refine_conflict(mdl, log_output=True)
            if conflict_result:
                print("\n--- CONFLICT REFINER REPORT ---")
                conflict_result.display()
        else:
            # 如果是无界或其他问题，打印求解细节
            print(f"Solve Details: {mdl.solve_details}")

        return None, None, None, None, None

    # --- 求解成功！---
    print(f"RMP LP solved. Status: {solve_status.name}, Objective: {solution.get_objective_value():.2f}")

    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    # --- 【新增】在这里调用效用分析函数 ---
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    if solution:  # 确保解存在
        # 我们需要 P_hat_internal，因为它包含了模型内部构建的所有路径信息
        analyze_lp_utilities(params, solution, all_vars, P_hat_internal)
        summarize_demand_distribution(params, solution, all_vars, P_hat_internal)
        #debug_background_cost_calculation(params, solution, all_vars, obj_components)  # 传入 obj_components
    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    # --- 效用分析结束 ---
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    duals = {}
    duals['mu_a'] = {a: constraints['bus_capacity'][a].dual_value for a in params['A'] if
                     a in constraints.get('bus_capacity', {})}
    duals['lambda_odh'] = {odh: const.dual_value for odh, const in constraints.get('share_conservation', {}).items()}
    duals['delta_ih'] = {ih: const.dual_value for ih, const in constraints.get('path_route_linking', {}).items()}
    temp_pi = {}
    for (i, h), constr in constraints.get('sblp_paths', {}).items():
        market = (P_hat_internal[i]['od'][0], P_hat_internal[i]['od'][1], h)
        temp_pi.setdefault(market, []).append(constr.dual_value)
    duals['pi_odh'] = {market: sum(values) / len(values) for market, values in temp_pi.items()}
    t_a_hat_vars = all_vars['t_a_hat']
    t_a_hat_sol = solution.get_value_dict(all_vars['t_a_hat'])
    return solution, duals, t_a_hat_sol, all_vars, obj_components


# --- 公开函数 2: 用于最终求解 ---
def solve_final_milp(params, R_hat, P_hat):
    start_time = time.time()
    mdl, _, all_vars, obj_components, _ = _build_rmp_common(params, R_hat, P_hat, integer_mode=True)
    build_time = time.time() - start_time
    print(f"Final MILP built in {build_time:.2f} seconds.")

    solution = mdl.solve(log_output=False)

    if not solution:
        print(f"\n--- FINAL MILP SOLVE FAILED --- \nSolve Details: {mdl.solve_details}")
        return None

    print(f"Final MILP solved. Objective: {solution.get_objective_value():.2f}")

    # 调用需求分布汇总
    if solution:
        # 注意：这里 P_hat 是从函数参数传入的，不是 _build_rmp_common 返回的
        summarize_demand_distribution(params, solution, all_vars, P_hat)

    return solution, all_vars, obj_components