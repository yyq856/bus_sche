# dataload.py (最终稳定版 - 回归手动、可控的图转换)
import pickle
import networkx as nx
import time
import numpy as np


def load_model_data(data_pickle_file):
    print(f"正在从 {data_pickle_file} 加载预处理的模型数据...")
    try:
        with open(data_pickle_file, 'rb') as f:
            model_data = pickle.load(f)
        print("数据加载成功！")
        return model_data
    except Exception as e:
        print(f"加载数据时发生错误: {e}")
        return None


def convert_to_digraph_robust(G_multi):
    """
    【最终稳健版】
    手动将 MultiDiGraph 转换为 DiGraph，只保留权重最小的平行边。
    """
    if not G_multi.is_multigraph():
        print("图已是 DiGraph，无需转换。")
        return G_multi

    start_time = time.time()
    print("检测到网络为 MultiDiGraph，正在手动转换为属性完整的 DiGraph...")

    print(f"  - 转换前 (MultiDiGraph): {len(G_multi.nodes())} 个节点, {len(G_multi.edges())} 条边")

    G_digraph = nx.DiGraph()
    G_digraph.add_nodes_from(G_multi.nodes(data=True))
    G_digraph.graph = G_multi.graph

    unique_edges = set(G_multi.edges(keys=False))
    for u, v in unique_edges:
        edges_data = G_multi.get_edge_data(u, v)
        best_edge_data = min(edges_data.values(),
                             key=lambda x: x.get('free_flow_time', float('inf')))
        G_digraph.add_edge(u, v, **best_edge_data)

    end_time = time.time()

    print(f"  - 转换后 (DiGraph):    {len(G_digraph.nodes())} 个节点, {len(G_digraph.edges())} 条边")
    print(f"图转换完成，耗时 {end_time - start_time:.2f} 秒。")
    return G_digraph


def pregenerate_auto_paths_and_times(G_digraph, od_pairs, k=3):  # <--- 【修改】函数名，更清晰
    """
    在 DiGraph 上预生成 k 条最短路径，并计算每个OD对的最短自由流通行时间。
    """
    start_time = time.time()
    print(f"正在为 {len(od_pairs)} 个OD对预生成路径并计算最短自由流时间...")

    auto_paths_od = {}
    # --- 【新增】用于存储最短自由流时间的字典 ---
    shortest_free_flow_times_od = {}

    for i, (o, d) in enumerate(od_pairs):
        if (i + 1) % 100 == 0:  # 增加进度提示
            print(f"  - 正在处理 OD 对 {i + 1}/{len(od_pairs)}: 从节点 {o} 到 {d}...")
        try:
            # --- 【新增】计算最短自由流通行时间 (U_L) ---
            # 使用 networkx.shortest_path_length 直接计算加权最短路径的成本
            shortest_time = nx.shortest_path_length(G_digraph, source=o, target=d, weight='free_flow_time')
            shortest_free_flow_times_od[(o, d)] = shortest_time

            # --- 【保留】原有逻辑：生成 k 条候选路径 ---
            if k == 1:
                path = nx.shortest_path(G_digraph, source=o, target=d, weight='free_flow_time')
                paths = [tuple(path)]
            else:
                paths_generator = nx.shortest_simple_paths(G_digraph, source=o, target=d, weight='free_flow_time')
                from itertools import islice
                paths = [tuple(p) for p in islice(paths_generator, k)]

            if not paths: raise nx.NetworkXNoPath  # 理论上不会执行，因为上面已经算出了长度
            auto_paths_od[(o, d)] = paths

        except nx.NetworkXNoPath:
            # print(f"    - 错误: 无法找到任何路径。") # 错误信息在 finalize_data_for_solver 中统一打印
            auto_paths_od[(o, d)] = []
            shortest_free_flow_times_od[(o, d)] = float('inf')  # 标记为不可达

    end_time = time.time()
    print(f"小汽车路径和时间预计算完成！耗时 {end_time - start_time:.2f} 秒。")
    # --- 【修改】返回两个字典 ---
    return auto_paths_od, shortest_free_flow_times_od


def finalize_data_for_solver(model_data):
    """
    【最终版】
    对加载的数据进行最终处理，并筛选掉无法找到驾车路径的OD对。
    """
    if not model_data:
        return None

    # 1. 转换图
    G_digraph = convert_to_digraph_robust(model_data['graph'])

    # --- 【修改】调用新函数，接收两个返回值 ---
    auto_paths_by_od, shortest_fft_by_od = pregenerate_auto_paths_and_times(G_digraph, model_data['od_pairs'], k=3)

    # --- 3. 【关键修改】筛选OD对 ---

    original_od_pairs = model_data['od_pairs']
    original_demand = model_data['demand']

    valid_od_pairs = []
    filtered_demand = {}
    filtered_auto_paths = {}
    # --- 【新增】用于存储筛选后OD对的最短自由流时间的字典 ---
    filtered_shortest_fft = {}

    removed_count = 0
    for o, d in original_od_pairs:
        # 检查这个OD对是否成功生成了至少一条小汽车路径
        if (o, d) in auto_paths_by_od and auto_paths_by_od[(o, d)]:
            # 如果路径存在，则保留这个OD对
            valid_od_pairs.append((o, d))
            filtered_auto_paths[(o, d)] = auto_paths_by_od[(o, d)]

            # --- 【新增】同时保留其最短自由流时间 ---
            filtered_shortest_fft[(o, d)] = shortest_fft_by_od[(o, d)]

            # 并保留它的需求
            for h in model_data['population_segments']:
                key = (o, d, h)
                if key in original_demand:
                    filtered_demand[key] = original_demand[key]
        else:
            # 如果路径列表为空，则移除这个OD对
            print(f"  - 移除OD对 ({o}, {d})，因为找不到驾车路径。")
            removed_count += 1

    print(f"筛选完成。共移除了 {removed_count} 个OD对。")
    print(f"剩余有效OD对数量: {len(valid_od_pairs)}")

    # --- 4. 更新 model_data ---
    model_data['graph'] = G_digraph
    model_data['od_pairs'] = valid_od_pairs
    model_data['demand'] = filtered_demand
    model_data['pregenerated_auto_paths'] = filtered_auto_paths
    # --- 【新增】将最终计算好的最短自由流时间添加到模型数据中 ---
    model_data['shortest_free_flow_times'] = filtered_shortest_fft

    print("所有数据已准备就绪。")
    return model_data

def calculate_path_length(G, path):
    """
    计算给定路径（节点列表）的总物理长度。

    Args:
        G (nx.DiGraph): 网络图。
        path (list or tuple): 节点序列。

    Returns:
        float: 路径的总长度（单位：米）。
    """
    total_length = 0.0
    # 遍历路径中的每条边 (u, v)
    for i in range(len(path) - 1):
        u, v = path[i], path[i+1]
        edge_data = G.get_edge_data(u, v)
        # 累加边的'length'属性
        if edge_data and 'length' in edge_data:
            total_length += edge_data['length']
    return total_length

# --- 主执行流程 (用于测试本文件) ---
if __name__ == '__main__':
    import pandas as pd
    from pathlib import Path
    
    DATA_PICKLE_FILE = '../data/model_input_data_10km.pkl'
    initial_model_data = load_model_data(DATA_PICKLE_FILE)
    if initial_model_data:
        solver_input = finalize_data_for_solver(initial_model_data)
        if solver_input:
            print("\n" + "="*80)
            print("--- 数据加载与准备结果示例 ---")
            print("="*80)
            final_G = solver_input['graph']
            print(f"最终图的类型: {type(final_G)}")
            print(f"是否是多重图? {final_G.is_multigraph()}")
            print(f"总节点数: {len(final_G.nodes())}")
            print(f"总边数: {len(final_G.edges())}")
            
            sample_od = list(solver_input['pregenerated_auto_paths'].keys())[0]
            paths = solver_input['pregenerated_auto_paths'][sample_od]

            # --- 【修改】测试新添加的数据 ---
            shortest_time = solver_input['shortest_free_flow_times'][sample_od]

            print(f"\n示例OD对: {sample_od}")
            print(f"  - 预生成的 {len(paths)} 条小汽车路径:")
            for i, p in enumerate(paths):
                print(f"    - 路径 {i + 1} (节点数 {len(p)}): {str(p[:5])}...{str(p[-3:])}")

            print(f"  - 最短自由流通行时间 (U_L): {shortest_time:.2f} 秒")
            
            # --- 【新增】导出所有OD对的路径信息为CSV ---
            print("\n" + "="*80)
            print("导出路径信息为CSV文件...")
            print("="*80)
            
            paths_data = []
            for (o, d), path_list in solver_input['pregenerated_auto_paths'].items():
                for path_idx, path_nodes in enumerate(path_list):
                    # 计算路径长度和时间
                    path_length = calculate_path_length(final_G, path_nodes)
                    path_time = solver_input['shortest_free_flow_times'].get((o, d), 0)
                    
                    paths_data.append({
                        'origin_node': o,
                        'destination_node': d,
                        'path_index': path_idx,
                        'num_nodes': len(path_nodes),
                        'num_edges': len(path_nodes) - 1,
                        'path_length_m': path_length,
                        'free_flow_time_sec': path_time,
                        'node_sequence': ' -> '.join([str(n) for n in path_nodes])
                    })
            
            paths_df = pd.DataFrame(paths_data)
            data_dir = Path('../data')
            output_path = data_dir / 'auto_paths_10km.csv'
            paths_df.to_csv(output_path, index=False, encoding='utf-8-sig')
            print(f"✅ 路径数据已保存到: {output_path}")
            print(f"   共导出 {len(paths_df)} 条路径记录")
            
            print("\n路径数据示例（前5条）:")
            print(paths_df[['origin_node', 'destination_node', 'path_index', 'num_nodes', 'free_flow_time_sec']].head())
            
            print("\n" + "="*80)
            print("路径统计信息:")
            print("="*80)
            print(f"总OD对数: {len(solver_input['pregenerated_auto_paths'])}")
            print(f"总路径数: {len(paths_df)}")
            print(f"平均路径节点数: {paths_df['num_nodes'].mean():.1f}")
            print(f"平均自由流时间: {paths_df['free_flow_time_sec'].mean():.2f} 秒 ({paths_df['free_flow_time_sec'].mean()/60:.2f} 分钟)")
            print(f"最短路径时间: {paths_df['free_flow_time_sec'].min():.2f} 秒")
            print(f"最长路径时间: {paths_df['free_flow_time_sec'].max():.2f} 秒")