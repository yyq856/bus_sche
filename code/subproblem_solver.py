# -*- coding: utf-8 -*-
# subproblem_solver.py
import networkx as nx
import itertools


def solve_pricing_subproblems(params, duals, t_a_hat_sol, R_hat, P_hat):
    """
    为每个市场 (o,d,h) 解决两阶段定价子问题，以决定是利用现有网络还是创建新线路。

    Args:
        params (dict): 模型参数。
        duals (dict): 从RMP中提取的对偶变量，应包含 'mu_a', 'lambda_odh', 'pi_odh', 'delta_ih'。
        t_a_hat_sol (dict): RMP求解出的链路旅行时间。
        R_hat (dict): 当前RMP中的候选公交线路集 {route_id: {'nodes': [node_list]}}。

    Returns:
        list: 一个包含有希望的新列的列表。每个元素是一个字典，
              指明了路径类型 ('existing' or 'new') 和相关数据。
    """
    print("\n--- Solving Pricing Subproblems for all markets ---")
    candidate_columns = []
    G = params['graph']

    # --- 1. 构建只包含现有公交线路的子图 G_bus ---
    existing_bus_edges = set()
    for r_data in R_hat.values():
        for u, v in zip(r_data['nodes'][:-1], r_data['nodes'][1:]):
            existing_bus_edges.add((u, v))

    # 创建一个只包含这些边的子图
    G_bus = G.edge_subgraph(existing_bus_edges).copy() if existing_bus_edges else nx.DiGraph()


    reported_active_pi = set()

    # --- 2. 遍历所有市场，解决子问题 ---
    debug_log_printed = False
    for od_pair in params['K']:
        o, d = od_pair
        for h in params['H']:
            market = (o, d, h)

            D_odh = params['D_odh'].get(market, 0)
            if D_odh == 0: continue

            # --- 2.1 获取对偶值 ---
            pi_odh = duals['pi_odh'].get(market, 0)
            lambda_odh = duals['lambda_odh'].get(market, 0)
            deltas_for_market = [d for (i, h_), d in duals['delta_ih'].items() if
                                 h_ == h and i in P_hat and P_hat[i]['od'] == od_pair]
            delta_odh = sum(deltas_for_market) / len(deltas_for_market) if deltas_for_market else 0

            # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            # --- 【修改】检查 pi_odh 是否不为零 (即约束是否激活) ---
            # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            # 我们检查其绝对值是否大于一个小的容差
            if abs(pi_odh) > 1e-6:
                # 为了避免重复打印，只在第一次遇到时报告
                if market not in reported_active_pi:
                    print("\n" + "*" * 30 + " INFO " + "*" * 30)
                    print(f"*** Active SBLP constraint detected! ***")
                    print(f"  - Market: {market}")
                    print(f"  - pi_odh: {pi_odh:.4f}")
                    print(f"  - This indicates the model is actively trading off utility vs. cost for this market.")
                    print("*" * 68)
                    reported_active_pi.add(market)
            # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            # --- 检查结束 ---
            # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

            # --- 2.2 【调试日志】打印对偶值 ---
            if not debug_log_printed:
                print("\n" + "=" * 50)
                print(f"DEBUGGING REDUCED COST for market {market}")
                print("=" * 50)
                print(f"  - Dual pi_odh (SBLP):         {pi_odh:.4f}")
                print(f"  - Dual lambda_odh (Share):    {lambda_odh:.4f}")
                print(f"  - Dual delta_odh (Linking):   {delta_odh:.4f}")
                mu_a_sample = list(duals['mu_a'].items())[0] if duals['mu_a'] else ("N/A", 0)
                print(f"  - Dual mu_a (Capacity, sample {mu_a_sample[0]}): {mu_a_sample[1]:.4f}")
                print("-" * 50)

            # --- 2.3 计算常数项 ---
            avg_headway_hr = sum(params['H_k'].values()) / len(params['H_k'])
            WT_new_hr = 0.5 * avg_headway_hr

            # 分解计算
            term_const_vot = D_odh * params['VOT_h'][h] * WT_new_hr
            term_const_pi_wt = -pi_odh * params['beta_WT'] * WT_new_hr
            term_const_pi_const = -pi_odh * params['beta_const']['B']
            term_const_lambda = -lambda_odh
            const_terms_base = term_const_vot + term_const_pi_wt + term_const_pi_const + term_const_lambda

            # --- 2.4 【调试日志】打印常数项构成 ---
            if not debug_log_printed:
                print("--- Constant Terms Breakdown ---")
                print(f"  - D_odh * VOT_h * WT      : {term_const_vot:.4f}")
                print(f"  - (-pi * beta_WT * WT)    : {term_const_pi_wt:.4f}")
                print(f"  - (-pi * beta_const_B)    : {term_const_pi_const:.4f}")
                print(f"  - (-lambda_odh)           : {term_const_lambda:.4f}")
                print(f"  -> Total ConstantTerms_base: {const_terms_base:.4f}")
                print("-" * 50)

            # --- 2.5 计算链路成本---
            for u_edge, v_edge in G.edges():
                a = (u_edge, v_edge)
                t_a_hr = t_a_hat_sol.get(a, G.edges[a].get('free_flow_time', 0) / 3600.0)
                mu_a_val = duals['mu_a'].get(a, 0)

                term_link_vot = D_odh * params['VOT_h'][h] * t_a_hr
                term_link_pi = -pi_odh * params['beta_TT_m']['B'] * t_a_hr
                term_link_mu = -mu_a_val * D_odh

                G.edges[a]['cg_cost'] = term_link_vot + term_link_pi + term_link_mu

            # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            # --- 【新增】在这里插入我们的负权重检查和诊断模块 ---
            # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

            # 遍历图中的所有边，检查 'cg_cost' 是否为负
            negative_edges_found = False
            for u, v, data in G.edges(data=True):
                cost = data['cg_cost']
                if cost < 0:
                    # 如果这是第一次发现负权重，打印一个标题
                    if not negative_edges_found:
                        print("\n" + "!" * 60)
                        print(f"!!! NEGATIVE WEIGHTS DETECTED for market {market} !!!")
                        print("!" * 60)
                        negative_edges_found = True

                    # 打印这条出问题的边的详细信息
                    print(f"\n--- Detailed Breakdown for Negative Edge ({u}, {v}) ---")
                    print(f"  Calculated cg_cost: {cost:.4f}")

                    # 重新计算并打印所有分量，确保我们看到的是正确的值
                    a = (u, v)
                    t_a_hr = t_a_hat_sol.get(a, G.edges[a].get('free_flow_time', 0) / 3600.0)
                    mu_a_val = duals['mu_a'].get(a, 0)

                    # 打印用于计算的参数
                    print("  Parameters used:")
                    print(f"    - pi_odh:      {pi_odh:.4f}")
                    print(f"    - beta_TT['B']: {params['beta_TT_m']['B']:.4f}")
                    print(f"    - mu_a:        {mu_a_val:.4f}")
                    print(f"    - D_odh:       {D_odh:.4f}")
                    print(f"    - VOT_h:       {params['VOT_h'][h]:.4f}")
                    print(f"    - t_a_hr:      {t_a_hr:.4f}")

                    # 重新计算并打印各项成本
                    re_term_link_vot = D_odh * params['VOT_h'][h] * t_a_hr
                    re_term_link_pi = -pi_odh * params['beta_TT_m']['B'] * t_a_hr
                    re_term_link_mu = mu_a_val * D_odh

                    print("  Cost Components:")
                    print(f"    - term_link_vot [D*VOT*t]:      {re_term_link_vot:12.4f}")
                    print(f"    - term_link_pi  [-pi*beta*t]:   {re_term_link_pi:12.4f}   <--- 重点检查这一项")
                    print(f"    - term_link_mu  [mu*D]:         {re_term_link_mu:12.4f}")
                    print("    -----------------------------------------")
                    print(
                        f"    - Total (verified):           {re_term_link_vot + re_term_link_pi + re_term_link_mu:12.4f}")

            # 如果发现了任何负权重，就立即抛出错误并停止程序
            # 这样可以避免 networkx 报错，而是给我们更清晰的错误信息
            if negative_edges_found:
                raise ValueError(
                    f"Negative 'cg_cost' weights found for market {market}. See detailed breakdown above. Halting.")

            # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            # --- 诊断模块结束 ---
            # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

            # --- 2.6 【调试日志】打印示例链路成本构成 ---
            if not debug_log_printed:
                sample_edge = list(G.edges())[0]
                sample_t_a = t_a_hat_sol.get(sample_edge, G.edges[sample_edge].get('free_flow_time', 0) / 3600.0)
                sample_mu_a = duals['mu_a'].get(sample_edge, 0)

                s_term_vot = D_odh * params['VOT_h'][h] * sample_t_a
                s_term_pi = -pi_odh * params['beta_TT_m']['B'] * sample_t_a
                s_term_mu = sample_mu_a * D_odh
                s_total_cost = G.edges[sample_edge]['cg_cost']

                print(f"--- Link Cost Breakdown (Sample Edge {sample_edge}) ---")
                print(f"  - D_odh * VOT_h * t_a       : {s_term_vot:.4f}")
                print(f"  - (-pi * beta_TT * t_a)     : {s_term_pi:.4f}")
                print(f"  - mu_a * D_odh              : {s_term_mu:.4f}")
                print(f"  -> Total cg_cost:             {s_total_cost:.4f}")
                print("-" * 50)
                debug_log_printed = True

            # --- 2.7 Stage 1 & 2  ---
            rc1, path1 = float('inf'), None
            if o in G_bus and d in G_bus and nx.has_path(G_bus, o, d):
                try:
                    path_cost1 = nx.shortest_path_length(G_bus, source=o, target=d, weight='cg_cost')
                    path1 = nx.shortest_path(G_bus, source=o, target=d, weight='cg_cost')
                    rc1 = path_cost1 + const_terms_base - delta_odh
                except (nx.NetworkXNoPath, nx.NodeNotFound):
                    pass

            rc2, path2 = float('inf'), None
            if nx.has_path(G, o, d):
                try:
                    path_cost2 = nx.shortest_path_length(G, source=o, target=d, weight='cg_cost')
                    path2 = nx.shortest_path(G, source=o, target=d, weight='cg_cost')
                    rc2 = path_cost2 + const_terms_base
                except (nx.NetworkXNoPath, nx.NodeNotFound):
                    print(f"警告: 在全网络中找不到OD对({o},{d})的路径。")
                    continue

            # --- 2.8 决策  ---
            min_rc = min(rc1, rc2)
            if min_rc < -1e-6:
                if min_rc == rc1:
                    print(f"  - Found column for market {market}: Use EXISTING network. RC={min_rc:.4f}")
                    candidate_columns.append(
                        {'type': 'existing', 'market': market, 'path_nodes': path1, 'reduced_cost': min_rc})
                else:
                    print(f"  - Found column for market {market}: Create NEW route. RC={min_rc:.4f}")
                    candidate_columns.append(
                        {'type': 'new', 'market': market, 'path_nodes': path2, 'reduced_cost': min_rc})

    print(f"--- Subproblem solving finished. Found {len(candidate_columns)} promising new columns. ---")
    return candidate_columns


# --- 主执行流程 (用于测试本文件) ---
if __name__ == '__main__':
    # 这个测试需要一个 'params' 和 'duals' 的模拟对象
    # 这是一个非常简化的示例，仅用于测试代码结构

    # 假设我们已经加载了 solver_input 并生成了 params
    # from data_load import ...
    # from model_parameters import ...
    # solver_input = ...
    # params = create_model_parameters(solver_input)

    # 这里我们手动创建一些假的 params 和 duals 用于演示
    print("Creating mock data for testing subproblem solver...")
    mock_G = nx.path_graph(5, create_using=nx.DiGraph)
    for u, v in mock_G.edges():
        mock_G.edges[u]['free_flow_time'] = 10
        mock_G.edges[u]['capacity'] = 1000
        mock_G.edges[u]['alpha'] = 0.15
        mock_G.edges[u]['beta'] = 4.0

    mock_params = {
        'graph': mock_G,
        'K': [(0, 4)],
        'H': ['h1'],
        'D_odh': {(0, 4, 'h1'): 100},
        'VOT_h': {'h1': 20},
        'beta_TT': -0.5, 'beta_WT': -1.0, 'beta_const': {'B': -1},
        'H_k': {'k0': 5 / 60},
        'P_hat': {0: {'od': (0, 2)}}  # 假设有一个已存在的路径
    }

    mock_duals = {
        'mu_a': {edge: 0.01 for edge in mock_G.edges()},
        'pi_odh': {(0, 4, 'h1'): -50},
        'lambda_odh': {(0, 4, 'h1'): -1000},
        'delta_ih': {(0, 'h1'): -200}  # 假设路径0的delta是-200
    }

    mock_t_a_hat = {edge: 12 for edge in mock_G.edges()}

    # 假设已有一个候选线路
    mock_R_hat = {0: {'nodes': [0, 1, 2]}}

    # 运行子问题求解器
    new_cols = solve_pricing_subproblems(mock_params, mock_duals, mock_t_a_hat, mock_R_hat)

    print("\nTest finished. Found columns:")
    print(new_cols)