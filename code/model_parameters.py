# -*- coding: utf-8 -*-
# model_parameters.py
import numpy as np
import networkx as nx
import geopy.distance


def create_model_parameters(solver_input):
    """
    基于dataload.py准备好的solver_input字典，
    生成MILP模型所需的所有参数。
    """
    if not solver_input:
        raise ValueError("Solver input data is empty. Cannot create parameters.")

    print("正在创建模型参数...")
    params = {}
    G = solver_input['graph']
    params['graph'] = G
    # --- 1. 集合与索引 ---
    params['A'] = list(G.edges())
    params['N'] = list(G.nodes())
    params['H'] = list(solver_input['population_segments'].keys())
    params['K'] = solver_input['od_pairs']
    params['M_h'] = {h: ['D', 'X', 'B', 'R', 'W', 'O'] for h in params['H']}

    # --- 2. OD需求 ---
    params['D_odh'] = solver_input['demand']

    # --- 3. 成本与效用参数 ---
    params['MIN_OTHER_SHARE'] = 0.001  # 设置一个非常小的最小份额 (0.01%)
    params['VOT_h'] = {h: data['vot'] for h, data in solver_input['population_segments'].items()}
    params['VOT_bg'] = 20  # 背景交通的价值时间，单位：元/小时
    
    # ===== 中国城市公交运营参数 =====
    # 固定成本（FC_r）：相对于模型量纲的单次出行固定成本（设为10）
    params['FC_r'] = 10.0   # 固定成本（与RC量纲匹配）
    
    # VC_r: 可变运营成本 = 燃油(1.5元/km) + 轮胎磨损(0.3元/km) + 维修保养(0.2元/km) = 2元/km
    params['VC_r'] = 0.02     # 调整为每km成本（相对小的值，与lambda匹配）
    # 公交班次间隔：常见的5个档次，覆盖高峰到非高峰
    headway_minutes = [5, 10, 15, 20, 30]  # 单位：分钟
    params['J_r'] = {f'k{i}': hw / 60.0 for i, hw in enumerate(headway_minutes)}
    params['H_k'] = params['J_r']
    
    # 运营周期：假设线路营运时段为5小时（如7:00-12:00或14:00-19:00）
    T_op = 5  # 单位：小时
    # 运营成本 = 运营周期 / 班次间隔 × 折扣系数（反映需要的车辆数和运营强度）
    params['C_op_rk_template'] = {k: (T_op / h) * params['VC_r'] * 25 for k, h in params['H_k'].items()}
    # 运营环境成本（外部性）：污染、噪音、道路磨损等，单位：元/小时
    params['C_op_env_hat'] = {
        'D': 15,    # 小汽车：污染、噪音、道路磨损等
        'X': 18,    # 出租车：更高的污染（长时间运营）
        'bg': 10    # 背景车流：平均污染成本
    }
    
    # 公交票价：中国城市普遍为1-5元，这里取3元（常见的中等城市票价）
    params['Fare_B'] = 3.0
    # 各出行方式对在途时间的敏感度（负值表示不效用）
    # 基于出行方式的舒适性和生产性：开车能工作/休闲，步行最累
    params['beta_TT_m'] = {
        'D': -2.0,  # 小汽车：可以工作、听音乐等，时间价值相对低
        'X': -2.5,  # 出租车：被动出行，舒适度比私车低
        'B': -1.5,  # 公交：可工作，有座位时舒适
        'R': -1.2,  # 地铁：快速、有座，相对高效
        'W': -3.5   # 步行：最累，尤其长距离，时间价值高
    }
    params['beta_WT'] = -1.0
    params['beta_TC'] = -0.2
    params['beta_AT'] = -0.8
    
    # --- Auto mode cost parameters (D: drive alone, X: taxi/ride-sharing) ---
    # Cost coefficients for D and X modes (utility per unit of cost, negative = disutility)
    params['cost_coeff_auto'] = {
        'D': -0.05,   # Drive alone: cost coefficient (RMB → disutility)
        'X': -0.10,   # Taxi/rideshare: higher cost coefficient (more expensive than driving)
    }
    
    # --- Calculate auto mode (D/X) travel distance-based costs ---
    # Cost structure:
    # D (drive): 燃油成本(1.5元/km) + 车辆折旧维修保险(0.5元/km) = 2元/km
    # X (taxi): 3元起步费 + 2.5元/km (常见的出租车/网约车价格)
    params['Cost_od_auto'] = {}
    for o, d in params['K']:
        o_coords = (G.nodes[o]['y'], G.nodes[o]['x'])
        d_coords = (G.nodes[d]['y'], G.nodes[d]['x'])
        dist_km = geopy.distance.great_circle(o_coords, d_coords).km
        
        # 小汽车运营成本（完全成本法）
        drive_cost = dist_km * 2.0  # 1.5元/km燃油 + 0.5元/km其他
        params['Cost_od_auto'][(o, d, 'D')] = drive_cost
        
        # 出租车/网约车成本（中等城市典型价格）
        taxi_cost = 3.0 + dist_km * 2.5  # 起步费 + 里程费
        params['Cost_od_auto'][(o, d, 'X')] = taxi_cost
    params['beta_const'] = {
        'D': 0.0,    # 私家车：基准
        'X': -0.1,   # 出租车：略低于私家车
        'B': 0.8,    # 公交：不参与外生效用（此处保留不影响）
        'R': -0.2,   # 地铁：靠近中性，提升可选性
        'W': -1.5,   # 步行：显著降低惩罚，便于短距离分担
        'O': -2.0}   # 其他：允许极小份额

    # --- 【新增】为外生模式定义广义社会成本系数 ---
    params['generalized_cost_coeff_m'] = {
        # 地铁 R: 假设其社会总成本是普通人时间价值的 3 倍
        'R': params['VOT_h']['h1'] * 3.0,  # 假设 h1 是一个有代表性的人群
        # 步行 W: 假设社会总成本是时间价值的 1.8 倍 (可能更慢，占用空间)
        'W': params['VOT_h']['h1'] * 1.8,
    }

    # --- 计算外生模式 (特别是地铁 'R') 的时间和费用 ---
    params['Time_od_exog'] = {}
    params['Cost_od_exog'] = {}
    exog_mode_params = {
        'R': {'access_hr': 15 / 60, 'speed_kmh': 35, 'wait_hr': 2.5 / 60},
        'W': {'access_hr': 0, 'speed_kmh': 5, 'wait_hr': 0},
    }
    for o, d in params['K']:
        o_coords = (G.nodes[o]['y'], G.nodes[o]['x'])
        d_coords = (G.nodes[d]['y'], G.nodes[d]['x'])
        dist_km = geopy.distance.great_circle(o_coords, d_coords).km
        for m in ['R', 'W']:
            p = exog_mode_params[m]
            travel_time_hr = dist_km / p['speed_kmh']
            total_time_hr = p['access_hr'] + travel_time_hr + p['wait_hr']
            params['Time_od_exog'][(o, d, m)] = total_time_hr
            cost = 0
            if m == 'R':
                if dist_km <= 6:
                    cost = 3
                elif dist_km <= 12:
                    cost = 4
                elif dist_km <= 22:
                    cost = 5
                elif dist_km <= 32:
                    cost = 6
                else:
                    cost = 7 + np.floor((dist_km - 32) / 20)
            params['Cost_od_exog'][(o, d, m)] = cost

    # --- 构建外生模式的【基础效用】 V_bar ---
    params['V_bar_odhm'] = {}
    for o, d in params['K']:
        for h in params['H']:
            for m in ['R', 'W', 'O']:
                if m == 'O':
                    params['V_bar_odhm'][(o, d, h, m)] = params['beta_const'][m]
                    continue
                time_hr = params['Time_od_exog'].get((o, d, m), 999)
                cost_val = params['Cost_od_exog'].get((o, d, m), 999)
                beta_tt_specific = params['beta_TT_m'][m]
                utility = beta_tt_specific * time_hr + \
                          params['beta_TC'] * cost_val + \
                          params['beta_const'][m]
                params['V_bar_odhm'][(o, d, h, m)] = utility

    # --- 4. 路网参数 ---
    # nx.get_edge_attributes returns an empty dict if the attribute doesn't exist.
    # We add a default value of 0 for background flow.
    all_edges = G.edges()
    background_flows = nx.get_edge_attributes(G, 'background_flow')
    params['bar_v_a'] = {edge: background_flows.get(edge, 0) for edge in all_edges}
    params['C_B'] = 40
    params['M'] = 1e6


    # --- 5. 线性化断点与参数预计算 ---
    params['B_bpr'] = np.linspace(0, 2.0, 5)
    params['C_log'] = np.logspace(np.log10(1e-6), np.log10(0.99), num=10)
    params['T_ab'], params['T_prime_ab'] = {}, {}
    for u, v, data in G.edges(data=True):
        edge_id = (u, v)
        # Add defaults for safety, although dataload should ensure they exist
        C_a = data.get('capacity', 1)
        t0_a_hr = data.get('free_flow_time', 1) / 3600.0
        alpha = data.get('alpha', 0.15)
        beta = data.get('beta', 4.0)

        for b_idx, b_ratio in enumerate(params['B_bpr']):
            v_ab = b_ratio * C_a
            t_a = t0_a_hr * (1 + alpha * (v_ab / C_a) ** beta)
            t_prime_a = t0_a_hr * (alpha * beta / C_a) * (v_ab / C_a) ** (beta - 1) if v_ab > 0 else 0
            params['T_ab'][(edge_id, b_idx)] = t_a
            params['T_prime_ab'][(edge_id, b_idx)] = t_prime_a

    params['LS_c'], params['LS_prime_c'] = {}, {}
    for c_idx, s_c in enumerate(params['C_log']):
        params['LS_c'][c_idx] = np.log(s_c)
        params['LS_prime_c'][c_idx] = 1 / s_c

    # --- 6. 【逻辑修改】直接使用预生成的小汽车路径 ---
    if 'pregenerated_auto_paths' not in solver_input:
        raise KeyError("错误: 'pregenerated_auto_paths' 未在 solver_input 中找到。请确保 data_load.py 正确运行。")

    params['P_auto_od'] = solver_input['pregenerated_auto_paths']

    # --- 7. 【新增】为麦考密克松弛计算参数界限 ---
    params['U_auto_L'] = solver_input['shortest_free_flow_times']
    params['U_auto_U'] = {od: 30 * t for od, t in params['U_auto_L'].items()}

    # WT 的界限 (单位：小时，与 H_k 一致)
    min_headway = min(params['H_k'].values())
    max_headway = max(params['H_k'].values())
    params['WT_L'] = 0.5 * min_headway
    params['WT_U'] = 0.5 * max_headway

    print("\n模型参数生成完毕。")
    return params


# --- 主执行流程 (用于测试本文件) ---
if __name__ == '__main__':
    from data_load import load_model_data, finalize_data_for_solver

    try:
        import geopy.distance
    except ImportError:
        print("错误: geopy 库未安装。请运行 'pip install geopy'。")
        exit()

    DATA_PICKLE_FILE = '../data/model_input_data_8km.pkl'
    initial_data = load_model_data(DATA_PICKLE_FILE)
    if initial_data:
        solver_input = finalize_data_for_solver(initial_data)
        if solver_input:
            model_params = create_model_parameters(solver_input)

            print("\n--- 参数生成结果示例 ---")
            if not model_params['K']:
                print("没有有效的OD对可供验证。")
            else:
                sample_od = model_params['K'][0]
                o, d = sample_od
                h = model_params['H'][0]
                print(f"示例OD对: {sample_od}")

                v_bar_val = model_params['V_bar_odhm'].get((o, d, h, 'R'))
                print(
                    f"  - 地铁 'R' 的基础效用 (V_bar): {v_bar_val:.2f}" if v_bar_val is not None else "  - 地铁 'R' 的基础效用 (V_bar): N/A")

                auto_paths = model_params['P_auto_od'].get(sample_od, [])
                print(f"  - 为该OD对加载的小汽车路径数量: {len(auto_paths)}")
                if auto_paths:
                    print(f"    - 第一条路径 (前5个节点): {auto_paths[0][:5]}...")

                # --- 【新增】验证新添加的界限参数 ---
                u_l = model_params['U_auto_L'].get(sample_od, 'N/A')
                u_u = model_params['U_auto_U'].get(sample_od, 'N/A')
                print(f"\n  - u_auto 的界限 (单位: 秒):")
                print(f"    - 下限 (U_L): {u_l:.2f}" if isinstance(u_l, float) else f"    - 下限 (U_L): {u_l}")
                print(f"    - 上限 (U_U): {u_u:.2f}" if isinstance(u_u, float) else f"    - 上限 (U_U): {u_u}")

                wt_l_hr = model_params['WT_L']
                wt_u_hr = model_params['WT_U']
                print(f"\n  - WT 的全局界限 (单位: 小时):")
                print(f"    - 下限 (WT_L): {wt_l_hr:.4f}  (约 {(wt_l_hr * 60):.2f} 分钟)")
                print(f"    - 上限 (WT_U): {wt_u_hr:.4f}  (约 {(wt_u_hr * 60):.2f} 分钟)")