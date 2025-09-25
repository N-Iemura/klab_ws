"""
ステップ応答解析専用スクリプト
既存のCSVファイルからステップ応答を解析・表示

使用方法:
python step_response_analyzer.py [CSVファイル名]
"""

# 標準ライブラリ
import os
import sys

# サードパーティライブラリ
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# 日本語フォント設定
plt.rcParams['font.family'] = 'DejaVu Sans'  # デフォルトフォント
# 利用可能な日本語フォントを設定（システムにより異なる）
japanese_fonts = ['Noto Sans CJK JP', 'Hiragino Sans', 'Yu Gothic', 'Meiryo', 'Takao', 'IPAexGothic', 'IPAPGothic', 'VL PGothic']
for font in japanese_fonts:
    if font in [f.name for f in fm.fontManager.ttflist]:
        plt.rcParams['font.family'] = font
        break
else:
    # 日本語フォントが見つからない場合は、文字化けを避けるため英語表記に変更
    print("Warning: Japanese fonts not found. Using English labels.")

def analyze_and_plot_step_response(csv_filename):
    """CSVデータからステップ応答を解析してグラフを表示"""
    print(f"ステップ応答解析を開始: {csv_filename}")
    
    # CSVファイル読み込み
    if not os.path.exists(csv_filename):
        print(f"エラー: ファイルが見つかりません: {csv_filename}")
        return
    
    try:
        df = pd.read_csv(csv_filename)
    except Exception as e:
        print(f"CSVファイル読み込みエラー: {e}")
        return
    
    # 必要な列が存在するか確認
    required_cols = ['time', 'motor0_setpoint_pos', 'motor0_pos', 'motor0_torque', 'motor0_error_pos']
    if not all(col in df.columns for col in required_cols):
        print("エラー: 必要な列が見つかりません")
        print("必要な列:", required_cols)
        print("実際の列:", list(df.columns))
        return
    
    # グラフ作成
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle(f'Step Response Analysis - {os.path.basename(csv_filename)}', fontsize=16)
    
    # Motor 0 の解析
    analyze_motor_response(df, 0, axes[:, 0])
    
    # Motor 1 の解析（データが存在する場合）
    if 'motor1_setpoint_pos' in df.columns:
        analyze_motor_response(df, 1, axes[:, 1])
    else:
        # Motor 1 データがない場合は空白
        for ax in axes[:, 1]:
            ax.text(0.5, 0.5, 'Motor 1 データなし', ha='center', va='center', transform=ax.transAxes)
    
    plt.tight_layout()
    
    # 一時的にグラフファイルを作成（表示用）
    os.makedirs('fig', exist_ok=True)
    base_filename = os.path.basename(csv_filename).replace('.csv', '_step_response.png')
    temp_graph_filename = os.path.join('fig', base_filename)
    plt.savefig(temp_graph_filename, dpi=300, bbox_inches='tight')
    
    # グラフを表示
    plt.show()
    
    # ユーザーに保存/破棄の選択を求める（既存CSVファイルの場合）
    print("\n" + "="*50)
    print("グラフファイルの処理を選択してください:")
    print("  [1] グラフを保存")
    print("  [2] グラフを破棄")
    print("="*50)
    
    while True:
        try:
            choice = input("選択 (1-2): ").strip()
            if choice in ['1', '2']:
                break
            else:
                print("1 または 2 を入力してください。")
        except KeyboardInterrupt:
            print("\n処理をキャンセルします。グラフを破棄します。")
            choice = '2'
            break
    
    # 選択に応じてファイル処理
    if choice == '1':
        # グラフを保存
        print(f"✅ グラフファイルを保存: {temp_graph_filename}")
        final_graph_path = temp_graph_filename
    else:
        # グラフを破棄
        try:
            os.remove(temp_graph_filename)
            print(f"🗑️  グラフファイルを削除: {temp_graph_filename}")
            final_graph_path = None
        except Exception as e:
            print(f"⚠️  グラフ削除エラー: {e}")
            final_graph_path = None
    
    plt.close('all')  # メモリリークを防ぐためにプロットを閉じる
    
    return final_graph_path

def analyze_motor_response(df, motor_id, axes):
    """個別モータのステップ応答解析"""
    target_col = f'motor{motor_id}_setpoint_pos'
    actual_col = f'motor{motor_id}_pos'
    error_col = f'motor{motor_id}_error_pos'
    torque_col = f'motor{motor_id}_torque'
    
    # 列が存在しない場合はスキップ
    if not all(col in df.columns for col in [target_col, actual_col, error_col, torque_col]):
        for ax in axes:
            ax.text(0.5, 0.5, f'Motor {motor_id} データ不足', ha='center', va='center', transform=ax.transAxes)
        return
    
    time_data = df['time'].values
    target_data = df[target_col].values
    actual_data = df[actual_col].values
    error_data = df[error_col].values
    torque_data = df[torque_col].values
    
    # 1. Position Step Response
    axes[0].plot(time_data, target_data, 'r--', label='Target', linewidth=2)
    axes[0].plot(time_data, actual_data, 'b-', label='Actual', linewidth=1)
    axes[0].set_title(f'Motor {motor_id} - Position Step Response')
    axes[0].set_ylabel('Position [turn]')
    axes[0].grid(True)
    axes[0].legend()
    
    # 2. Torque Output
    axes[1].plot(time_data, torque_data, 'g-', linewidth=1)
    axes[1].set_title(f'Motor {motor_id} - Torque Output')
    axes[1].set_ylabel('Torque [Nm]')
    axes[1].grid(True)
    
    # 3. Position Error
    axes[2].plot(time_data, error_data, 'r-', linewidth=1)
    axes[2].set_title(f'Motor {motor_id} - Position Error')
    axes[2].set_ylabel('Error [turn]')
    axes[2].set_xlabel('Time [s]')
    axes[2].grid(True)
    
    # ステップ応答特性の数値解析
    print(f"\n=== Motor {motor_id} ステップ応答特性 ===")
    
    # ステップ変化点を検出
    target_diff = np.diff(target_data)
    step_indices = np.where(np.abs(target_diff) > 0.05)[0]
    
    if len(step_indices) == 0:
        print("  ステップ変化が検出されませんでした")
        return
    
    for i, step_idx in enumerate(step_indices[:4]):  # 最初の4つのステップを解析
        step_start = step_idx + 1
        step_end = min(step_start + 1000, len(df))  # 応答解析範囲
        
        target_val = target_data[step_start]
        prev_target = target_data[step_idx]
        
        # 0への変化や小さな変化はスキップ
        if abs(target_val - prev_target) < 0.05:
            continue
            
        actual_vals = actual_data[step_start:step_end]
        time_vals = time_data[step_start:step_end] - time_data[step_start]
        
        print(f"\nステップ {i+1}: {prev_target:.3f} → {target_val:.3f} turn")
        
        # 立ち上がり時間（10%-90%）
        step_size = target_val - prev_target
        val_10 = prev_target + step_size * 0.1
        val_90 = prev_target + step_size * 0.9
        
        if step_size > 0:
            rise_start_idx = np.where(actual_vals >= val_10)[0]
            rise_end_idx = np.where(actual_vals >= val_90)[0]
        else:
            rise_start_idx = np.where(actual_vals <= val_10)[0]
            rise_end_idx = np.where(actual_vals <= val_90)[0]
        
        if len(rise_start_idx) > 0 and len(rise_end_idx) > 0:
            rise_time = time_vals[rise_end_idx[0]] - time_vals[rise_start_idx[0]]
            print(f"  立ち上がり時間: {rise_time:.3f}秒")
        
        # オーバーシュート
        if step_size > 0:
            max_val = np.max(actual_vals)
            overshoot = ((max_val - target_val) / abs(step_size)) * 100
        else:
            max_val = np.min(actual_vals)
            overshoot = ((target_val - max_val) / abs(step_size)) * 100
        print(f"  オーバーシュート: {overshoot:.1f}%")
        
        # 整定時間（5%以内）
        settle_threshold = 0.05 * abs(step_size)
        settle_indices = np.where(np.abs(actual_vals - target_val) <= settle_threshold)[0]
        if len(settle_indices) > 50:  # 連続して条件を満たす
            settle_time = time_vals[settle_indices[50]]
            print(f"  整定時間(5%): {settle_time:.3f}秒")
        
        # 定常偏差
        if step_end > step_start + 100:
            steady_error = np.mean(actual_data[step_end-100:step_end] - target_val)
            print(f"  定常偏差: {steady_error:.4f} turn")

def main():
    if len(sys.argv) > 1:
        csv_filename = sys.argv[1]
    else:
        # 最新のCSVファイルを自動検索
        csv_files = [f for f in os.listdir('.') if f.startswith('integrated_pid_torque_') and f.endswith('.csv')]
        if not csv_files:
            print("CSVファイルが見つかりません")
            print("使用方法: python step_response_analyzer.py [CSVファイル名]")
            return
        
        csv_filename = max(csv_files, key=os.path.getmtime)  # 最新ファイル
        print(f"最新のCSVファイルを使用: {csv_filename}")
    
    analyze_and_plot_step_response(csv_filename)

if __name__ == "__main__":
    main()