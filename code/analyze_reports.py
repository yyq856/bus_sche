# analyze_reports.py

import os
import re
import pandas as pd
import openpyxl
from datetime import datetime

# --- 1. 配置 ---

# 指定存放报告的文件夹路径
REPORTS_DIR = '../reports'

# 指定最终生成的 Excel 文件路径
OUTPUT_EXCEL_FILE = '../analysis/experiment_summary.xlsx'


# --- 2. 核心功能函数 ---

def parse_report_file(filepath):
    """
    【v2 增强版】
    解析单个报告文件，提取所有关键指标，返回一个字典。
    增加了更多指标的捕获，并使正则表达式更健壮。
    """
    print(f"--> Parsing file: {os.path.basename(filepath)}")

    report_data = {'filename': os.path.basename(filepath)}

    # 【修改】全面扩充正则表达式字典
    patterns = {
        # Section 1: Inputs
        'total_od_pairs': re.compile(r"Total OD Pairs\s*:\s*(\d+)"),
        'total_demand': re.compile(r"Total Demand\s*:\s*([\d,]+)"),
        'total_candidate_routes': re.compile(r"Total Candidate Bus Routes\s*:\s*(\d+)"),
        'total_generated_paths': re.compile(r"Total Generated Bus Paths\s*:\s*(\d+)"),

        # Section 2: Costs
        'total_cost': re.compile(r"TOTAL SOCIETAL COST\s*:\s*([\d,.-]+)"),
        'system_operator_cost': re.compile(r"System_Operator_Cost\s*:\s*([\d,.-]+)"),
        'user_cost': re.compile(r"User_Cost\s*:\s*([\d,.-]+)"),
        'background_traffic_cost': re.compile(r"Background_Traffic_Cost\s*:\s*([\d,.-]+)"),
        'slack_penalty': re.compile(r"Slack_Penalty\s*:\s*([\d,.-]+)"),
        'other_mode_penalty': re.compile(r"Other_Mode_Penalty\s*:\s*([\d,.-]+)"),

        # Section 3: Bus Decisions
        'activated_routes': re.compile(r"Activated Routes\s*:\s*(\d+)"),

        # Section 4: Demand Distribution - 捕获乘客数和份额
        # 使用更通用的正则表达式来匹配表格格式
        'passengers_B': re.compile(r"^\s*B\s*\|([\d,.\s]+)\|"),
        'share_B': re.compile(r"^\s*B\s*\|[\d,.\s]+\|([\d,.\s]+)%"),
        'passengers_D': re.compile(r"^\s*D\s*\|([\d,.\s]+)\|"),
        'share_D': re.compile(r"^\s*D\s*\|[\d,.\s]+\|([\d,.\s]+)%"),
        'passengers_X': re.compile(r"^\s*X\s*\|([\d,.\s]+)\|"),
        'share_X': re.compile(r"^\s*X\s*\|[\d,.\s]+\|([\d,.\s]+)%"),
        'passengers_R': re.compile(r"^\s*R\s*\|([\d,.\s]+)\|"),
        'share_R': re.compile(r"^\s*R\s*\|[\d,.\s]+\|([\d,.\s]+)%"),
        'passengers_K': re.compile(r"^\s*K\s*\|([\d,.\s]+)\|"),
        'share_K': re.compile(r"^\s*K\s*\|[\d,.\s]+\|([\d,.\s]+)%"),
        'passengers_W': re.compile(r"^\s*W\s*\|([\d,.\s]+)\|"),
        'share_W': re.compile(r"^\s*W\s*\|[\d,.\s]+\|([\d,.\s]+)%"),
        'passengers_O': re.compile(r"^\s*O\s*\|([\d,.\s]+)\|"),
        'share_O': re.compile(r"^\s*O\s*\|[\d,.\s]+\|([\d,.\s]+)%"),

        # Section 5: Congestion
        'congested_links': re.compile(r"Congested Links \(>1s delay\)\s*:\s*(\d+)"),
        'avg_delay': re.compile(r"Average Delay on Congested Links\s*:\s*([\d,.-]+)"),
        'max_delay': re.compile(r"Maximum Delay on a Single Link\s*:\s*([\d,.-]+)"),
    }

    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()  # 一次性读取整个文件内容，方便多行匹配

            for key, pattern in patterns.items():
                match = pattern.search(content, re.MULTILINE)  # 使用多行模式
                if match:
                    value_str = match.group(1).strip()
                    try:
                        value_float = float(value_str.replace(',', ''))
                        report_data[key] = value_float
                    except ValueError:
                        print(
                            f"    - Warning: Could not convert value '{value_str}' to float for key '{key}'. Skipping.")

    except FileNotFoundError:
        print(f"    - Error: File not found.")
        return None
    except Exception as e:
        print(f"    - Error: An unexpected error occurred: {e}")
        return None

    return report_data


def main():
    """
    主函数，执行批量读取、解析和导出到Excel的流程。
    """
    print(f"Starting to process reports from directory: '{REPORTS_DIR}'")

    # 检查报告目录是否存在
    if not os.path.isdir(REPORTS_DIR):
        print(f"Error: Reports directory '{REPORTS_DIR}' not found. Please check the path.")
        return

    # 找到所有 .txt 报告文件
    report_files = [f for f in os.listdir(REPORTS_DIR) if f.endswith('.txt')]

    if not report_files:
        print("No report (.txt) files found in the directory.")
        return

    print(f"Found {len(report_files)} report files to process.")

    # 逐个解析文件，将结果存储在列表中
    all_results = []
    for filename in report_files:
        filepath = os.path.join(REPORTS_DIR, filename)
        data = parse_report_file(filepath)
        if data:
            all_results.append(data)

    if not all_results:
        print("Failed to parse any report files.")
        return

    # 将结果列表转换为 pandas DataFrame
    df = pd.DataFrame(all_results)

    # --- 数据后处理 (可选，但推荐) ---
    # 1. 设置 'filename' 为索引，方便查看
    df.set_index('filename', inplace=True)

    # 2. 对列进行排序，使输出更有条理
    # 可以自定义你想要的列顺序
    desired_column_order = [
        # === Inputs ===
        'total_od_pairs',
        'total_demand',
        'total_candidate_routes',
        'total_generated_paths',

        # === Overall Performance (Costs) ===
        'total_cost',
        'system_operator_cost',
        'user_cost',
        'background_traffic_cost',
        'slack_penalty',
        'other_mode_penalty',

        # === System Decisions (Bus) ===
        'activated_routes',

        # === Behavioral Outcomes (Shares %) ===
        'share_B',
        'share_D',
        'share_X',
        'share_R',
        'share_K',
        'share_W',
        'share_O',

        # === Behavioral Outcomes (Passengers) ===
        'passengers_B',
        'passengers_D',
        'passengers_X',
        'passengers_R',
        'passengers_K',
        'passengers_W',
        'passengers_O',

        # === Network Performance (Congestion) ===
        'congested_links',
        'avg_delay',
        'max_delay'
    ]
    # 只保留存在的列，避免因为某些文件解析失败而报错
    existing_columns = [col for col in desired_column_order if col in df.columns]
    df = df[existing_columns]

    print("\n--- Data successfully parsed into DataFrame ---")
    print(df.head())  # 打印前5行以供预览

    # 确保输出目录存在
    output_dir = os.path.dirname(OUTPUT_EXCEL_FILE)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 将 DataFrame 导出到 Excel
    try:
        df.to_excel(OUTPUT_EXCEL_FILE)
        print(f"\nSuccessfully exported summary to Excel file: '{OUTPUT_EXCEL_FILE}'")
    except Exception as e:
        print(f"\nError: Failed to export to Excel. Error: {e}")


# --- 3. 主执行入口 ---

if __name__ == '__main__':
    main()