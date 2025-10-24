"""
トルク制御PID環境 - 最小ノルム配分・ヌル空間安定化 版（コピペ即実行）

目的:
- 出力角θのみを外側PIDで追従
- 出力トルク τ_out* を最小ノルムで各モータへ配分 (τ_cmd = A^T τ_out*)
- ヌル空間ダンピング / 姿勢戻しで内部ドリフトを抑制
- 既存のODriveトルクモードにそのまま繋ぐ

使い方:
1) 本ファイルを保存して `python integrated_pid_torque_control_min_norm.py` を実行。
2) 終了時に CSV とグラフの保存/破棄を選べます。

注意:
- ODriveのシリアル番号やトルク定数は環境に合わせて変更してください。
- 出力角θは独立エンコーダ (odrv2) で計測する想定です。
"""

# ===================== 標準ライブラリ =====================
import csv
import json
import math
import os
import threading
import time
from datetime import datetime

# ===================== サードパーティ =====================
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ===================== ODrive =====================
import odrive
from odrive.enums import *

# ===================== 日本語フォント設定 =====================
plt.rcParams['font.family'] = 'DejaVu Sans'
japanese_fonts = ['Noto Sans CJK JP', 'Hiragino Sans', 'Yu Gothic', 'Meiryo', 'Takao', 'IPAexGothic', 'IPAPGothic', 'VL PGothic']
for font in japanese_fonts:
    if font in [f.name for f in fm.fontManager.ttflist]:
        plt.rcParams['font.family'] = font
        break
else:
    print("警告: 日本語フォントが見つかりません。グラフは英語表記になります。")

# ==================================================================================
# 設定
# ==================================================================================
REFERENCE_PROFILE = {
    'active_profile': 'ramp_step',  # 'step', 'sine', 'chirp'
    'file_label': None,        # CSV/グラフの識別子に使う任意文字列
    'step': {
        'initial_wait': 1.0,        # 最初のステップまでの待機[秒]
        'step_duration': 20.0,      # 各ステップの持続[秒]
        'output_amplitude': 0.2,    # 出力θ振幅[turn]
        'offset': 0.0,
    },
    'sine': {
        'initial_wait': 1.0,        # 開始前の待機[秒]
        'output_amplitude': 0.1,   # 正弦波振幅[turn]
        'frequency_hz': 0.1,       # 周波数[Hz]
        'offset': 0.0,              # バイアス
    },
    'chirp': {
        'initial_wait': 1.0,
        'output_amplitude': 0.2,
        'start_frequency_hz': 0.02,
        'end_frequency_hz': 0.2,
        'duration': 60.0,           # 開始からこの秒数で終端周波数へ
        'offset': 0.0,
    },
    'ramp': {
        'initial_wait': 1.0,
        'start_value': 0.0,
        'end_value': 2.0,
        'ramp_duration': 2.0,      # [s] start -> end の時間
        'hold_duration': 5.0,       # [s] end_value で保持
        'return_duration': 0.0,     # [s] >0 のとき start_value へ線形で戻す
        'repeat': True,             # True なら周期的に繰り返す
    },
    'ramp_step': {
        'initial_wait': 1.0,
        'start_value': 0.0,
        'end_value': 0.5,
        'ramp_duration': 2.0,
        'hold_duration': 5.0,
        'return_duration': 0.0,
        'repeat': True,
        'step': {
            'amplitude': 0.1,       # 追加ステップ振幅[turn]
            'duration': 1.0,        # ステップ保持時間[秒]
            'start_after': 0.0,     # 初期待機後、この秒数経過で最初のステップ
            'period': 3.0,          # 周期的に繰り返す場合の周期[秒]
            'repeat': False,        # Falseで一度だけステップ
            'offset_in_cycle': 0.0, # 周期内でステップを入れる位置[秒]
            'align_to_ramp_end': True,  # Trueならランプ完了直後に挿入
        },
    },
}

# 後方互換性用のエイリアス
STEP_CONFIG = REFERENCE_PROFILE['step']

# 制御モード: 'output_pid' は既存の単一PID, 'per_motor_pid' はモータ毎PID
CONTROL_MODE = 'per_motor_pid'

# 出力θのPIDゲイン（外側）※CONTROL_MODE='output_pid' のとき使用
OUTPUT_PID = {'kp': 1.0, 'ki': 0.8, 'kd': 0.00, 'max_output': 200.0}

# per_motor_pid モードでも外側PIDを併用する場合は True
ENABLE_OUTER_PID_IN_PER_MOTOR = False

# 各モータ用PIDゲイン（CONTROL_MODE='per_motor_pid' のとき使用）
MOTOR_PID = {
    'motor0': {'kp': 0.31, 'ki': 0.2, 'kd': 0.02, 'max_output': 5.5},   # T-motor
    'motor1': {'kp': 0.1, 'ki': 0.02, 'kd': 0.002, 'max_output': 0.2}     # Maxon
}

# ヌル空間安定化ゲイン
NULLSPACE_CONFIG = {
    'Knu_diag': [0.0, 0.0],   # 粘性ダンピング（対角）
    'Kq_diag':  [0.0, 0.0],   # 姿勢戻し（まずは0）
    'q_ref':    [0.0, 0.0],   # 望ましい関節姿勢
}

# 片側モータを固定する場合の設定（'motor0' / 'motor1' / None）
FREEZE_CONFIG = {
    'motor_to_freeze': None,
    'kp': 1.0,
    'kd': 0.05,
}

# 安全制限
SAFETY_CONFIG = {
    'max_torque0': 6.0,      # T-motor 最大トルク[Nm]
    'max_torque1': 0.2,      # Maxon 最大トルク[Nm]
}

# ODrive接続設定（必要に応じて変更）
ODRIVE_SERIAL = {
    'motor0': '3856345D3539',  # T-motor 側
    'motor1': '384D346F3539',  # Maxon 側
    'output': '3849346F3539',  # 出力エンコーダ
}
ODRIVE_TORQUE_CONSTANT = {
    'motor0': 0.106,  # Nm/A
    'motor1': 0.091,  # Nm/A
}

# 制御周期 [Hz]
CONTROL_FREQUENCY = 200

# 出力ファイル関連
CSV_DIR = 'csv'
FIG_DIR = 'fig'
DATA_FILENAME_PREFIX = 'norm2_hw'
PLOT_FILENAME_SUFFIX = '_plot.pdf'

# ==================================================================================
# ユーティリティ: 機構行列 / 最小ノルム配分 / ヌル空間射影
# ==================================================================================

def get_A(q=None):
    """機構の出力写像 A = [a1 a2] (1x2)
    姿勢依存の場合は q から計算する。まずは定数でOK。
    例: a1 = 1/20, a2 = 163/2000
    """
    return np.array([[1/20, 163/2000]])


def project_null(A):
    """ヌル空間射影 P_N = I - A^T (A A^T)^(-1) A (2x2)
    Aは1x2、AA^Tは1x1スカラーなので安定化のためにダンピングを入れる。
    """
    At = A.T
    s = float(A @ At)   # = a1^2 + a2^2
    eps = 1e-6
    inv = 1.0 / max(s, eps)
    PN = np.eye(2) - At * inv @ A
    return PN


def min_norm_torque_split(A, tau_out):
    """最小ノルムのトルク配分: τ* = A^T (A A^T)^(-1) τ_out"""
    At = A.T
    s = float(A @ At)  # = a1^2 + a2^2
    if s < 1e-8:
        raise ValueError("Mechanism matrix A is near-singular.")
    scale = float(tau_out) / s
    return (At * scale).reshape(2)


MOTOR_OUTPUT_GAINS = np.array([20.0, 2000.0 / 163.0], dtype=float)


def motor_torque_to_output(tau_vec):
    """モータトルクから出力トルクへ変換: τ_out = τ0*20 + τ1*(2000/163)"""
    tau_vec = np.asarray(tau_vec, dtype=float).reshape(2)
    return float(np.dot(MOTOR_OUTPUT_GAINS, tau_vec))


def _solve_torque_with_limits(A, tau_desired, torque_limits, tau_preferred=None, tol=1e-9):
    """
    A:              1x2 行列
    tau_desired:    望ましい出力トルク (スカラー)
    torque_limits:  [limit0, limit1]
    tau_preferred:  ヌル空間成分など、可能なら近づけたい候補
    戻り値: (tau_solution[2], 実現された出力トルク)
    """
    A = np.asarray(A, dtype=float).reshape(1, 2)
    limits = np.asarray(torque_limits, dtype=float)
    a1, a2 = A[0]

    # フィージビリティチェック: 角の値から達成可能な出力トルク範囲を把握
    corners = np.array([
        [ limits[0],  limits[1]],
        [ limits[0], -limits[1]],
        [-limits[0],  limits[1]],
        [-limits[0], -limits[1]],
    ], dtype=float)
    tau_corner_vals = corners @ A.T  # shape (4,1)
    tau_min = float(np.min(tau_corner_vals))
    tau_max = float(np.max(tau_corner_vals))
    tau_target = float(np.clip(tau_desired, tau_min, tau_max))

    # 最小ノルム解（等式を満たす）
    tau_base = min_norm_torque_split(A, tau_target)
    if tau_preferred is None:
        tau_preferred = tau_base.copy()
    tau_preferred = np.asarray(tau_preferred, dtype=float).reshape(2)

    # 既に制限内なら終了
    if np.all(np.abs(tau_base) <= limits + 1e-9):
        return tau_base, tau_target

    # 等式を維持したままボックスへ射影（ヌル空間方向を利用）
    n = np.array([a2, -a1], dtype=float)  # ヌル空間基底
    n_norm_sq = float(np.dot(n, n))

    def project_with_preference(pref):
        if n_norm_sq < tol:
            return None
        alpha_opt = float(np.dot(n, pref - tau_base) / n_norm_sq)
        alpha_low, alpha_high = -np.inf, np.inf
        for i in range(2):
            n_i = n[i]
            if abs(n_i) < tol:
                # この軸では調整できない -> ベースが制限を超えるなら不可
                if abs(tau_base[i]) <= limits[i] + 1e-9:
                    continue
                return None
            low = (-limits[i] - tau_base[i]) / n_i
            high = (limits[i] - tau_base[i]) / n_i
            if low > high:
                low, high = high, low
            alpha_low = max(alpha_low, low)
            alpha_high = min(alpha_high, high)
            if alpha_low > alpha_high:
                return None
        alpha = float(min(max(alpha_opt, alpha_low), alpha_high))
        tau_candidate = tau_base + alpha * n
        if np.all(np.abs(tau_candidate) <= limits + 1e-8):
            return tau_candidate
        return None

    candidate = project_with_preference(tau_preferred)
    if candidate is not None:
        return candidate, tau_target

    candidate = project_with_preference(tau_base)
    if candidate is not None:
        return candidate, tau_target

    # それでも見つからない場合は、片方のモータを限界に固定して求める
    best = None
    eps = tol

    def evaluate_candidate(tau_vec):
        nonlocal best
        if tau_vec is None:
            return
        if not np.all(np.abs(tau_vec) <= limits + 1e-6):
            return
        tau_out = float(A @ tau_vec.reshape(2, 1))
        err_out = abs(tau_out - tau_target)
        pref_err = np.linalg.norm(tau_vec - tau_preferred)
        score = (err_out, pref_err)
        if best is None or score < best[0]:
            best = (score, tau_vec, tau_out)

    # motor0 を限界に貼り付け
    if abs(a2) > eps:
        for s0 in (-1, 1):
            t0 = s0 * limits[0]
            t1 = (tau_target - a1 * t0) / a2
            if abs(t1) <= limits[1] + 1e-6:
                evaluate_candidate(np.array([t0, t1], dtype=float))

    # motor1 を限界に貼り付け
    if abs(a1) > eps:
        for s1 in (-1, 1):
            t1 = s1 * limits[1]
            t0 = (tau_target - a2 * t1) / a1
            if abs(t0) <= limits[0] + 1e-6:
                evaluate_candidate(np.array([t0, t1], dtype=float))

    # それでも不可なら角の中で最も出力が近いものを採用
    if best is None:
        for vec in corners:
            evaluate_candidate(vec)

    if best is None:
        # 理論上ここには来ないはずだが、最悪は単純クリップ
        tau_fallback = np.clip(tau_base, -limits, limits)
        tau_out = float(A @ tau_fallback.reshape(2, 1))
        return tau_fallback, tau_out

    _, tau_vec, tau_out = best
    return tau_vec, tau_out


def project_torque_to_limits(A, tau_candidate, torque_limits):
    """候補トルクを、出力トルクを極力維持しつつ安全範囲へ投影"""
    tau_candidate = np.asarray(tau_candidate, dtype=float).reshape(2)
    desired_output = float((A @ tau_candidate.reshape(2, 1)).item())
    tau_res, _ = _solve_torque_with_limits(
        A, desired_output, torque_limits, tau_preferred=tau_candidate
    )
    return tau_res

# ==================================================================================
# 目標生成
# ==================================================================================

def get_active_profile_name():
    profile = REFERENCE_PROFILE.get('active_profile', 'step')
    if profile not in REFERENCE_PROFILE or not isinstance(REFERENCE_PROFILE[profile], dict):
        raise ValueError(f"未定義の目標プロファイル: {profile}")
    return profile


def get_profile_label():
    label = REFERENCE_PROFILE.get('file_label')
    if label:
        return str(label)
    return get_active_profile_name()


def sanitize_label_for_filename(label):
    safe = ''.join(ch if (ch.isalnum() or ch in ('-', '_')) else '_' for ch in label)
    return safe or 'profile'


def _compute_ramp_profile(cfg, elapsed_time):
    initial_wait = float(cfg.get('initial_wait', 0.0))
    start_value = float(cfg.get('start_value', 0.0))
    end_value = float(cfg.get('end_value', start_value))
    ramp_duration = max(float(cfg.get('ramp_duration', 0.0)), 1e-6)
    hold_duration = max(float(cfg.get('hold_duration', 0.0)), 0.0)
    return_duration = max(float(cfg.get('return_duration', 0.0)), 0.0)
    repeat = bool(cfg.get('repeat', True))

    cycle = ramp_duration + hold_duration + return_duration
    if elapsed_time < initial_wait:
        return start_value, 0.0, cycle, repeat, ramp_duration

    def ramp_value(phase, repeat_mode):
        if phase < ramp_duration:
            ratio = phase / ramp_duration
            return start_value + (end_value - start_value) * ratio
        phase -= ramp_duration
        if phase < hold_duration:
            return end_value
        phase -= hold_duration
        if return_duration > 0.0:
            ratio = min(max(phase / return_duration, 0.0), 1.0)
            return end_value + (start_value - end_value) * ratio
        return start_value if repeat_mode else end_value

    t_after_wait = elapsed_time - initial_wait
    if cycle <= 0.0:
        return end_value, max(t_after_wait, 0.0), cycle, repeat, ramp_duration

    if repeat:
        phase = t_after_wait % cycle
        value = ramp_value(phase, True)
    else:
        if t_after_wait >= cycle:
            final_value = start_value if return_duration > 0.0 else end_value
            value = final_value
        else:
            value = ramp_value(t_after_wait, False)

    return value, max(t_after_wait, 0.0), cycle, repeat, ramp_duration


def generate_output_reference(elapsed_time):
    """出力θの目標値をプロファイルに応じて生成"""
    profile = get_active_profile_name()
    cfg = REFERENCE_PROFILE[profile]

    if profile == 'step':
        initial_wait = cfg['initial_wait']
        step_duration = cfg['step_duration']
        amp = cfg['output_amplitude']
        offset = cfg.get('offset', 0.0)

        if elapsed_time < initial_wait:
            return offset

        period = step_duration * 4.0
        if period <= 0.0:
            return offset
        cyc = (elapsed_time - initial_wait) % period
        if cyc < step_duration:
            return offset + amp
        else:
            return offset

    if profile == 'sine':
        initial_wait = cfg['initial_wait']
        amp = cfg['output_amplitude']
        freq = cfg['frequency_hz']
        offset = cfg.get('offset', 0.0)
        if elapsed_time < initial_wait:
            return offset
        t = elapsed_time - initial_wait
        return offset + amp * math.sin(2.0 * math.pi * freq * t)

    if profile == 'chirp':
        initial_wait = cfg['initial_wait']
        amp = cfg['output_amplitude']
        f0 = cfg['start_frequency_hz']
        f1 = cfg['end_frequency_hz']
        duration = max(cfg['duration'], 1e-6)
        offset = cfg.get('offset', 0.0)
        if elapsed_time < initial_wait:
            return offset
        t = elapsed_time - initial_wait
        k = (f1 - f0) / duration
        if t > duration:
            phase = 2.0 * math.pi * (f0 * duration + 0.5 * k * duration ** 2) + 2.0 * math.pi * f1 * (t - duration)
        else:
            phase = 2.0 * math.pi * (f0 * t + 0.5 * k * t ** 2)
        return offset + amp * math.sin(phase)

    if profile == 'ramp':
        value, _, _, _, _ = _compute_ramp_profile(cfg, elapsed_time)
        return value

    if profile == 'ramp_step':
        base_value, t_after_wait, cycle, repeat_ramp, ramp_duration = _compute_ramp_profile(cfg, elapsed_time)
        step_cfg = cfg.get('step', {})
        amplitude = float(step_cfg.get('amplitude', 0.0))
        if amplitude == 0.0:
            return base_value

        align_to_ramp_end = bool(step_cfg.get('align_to_ramp_end', False))
        start_after_base = max(float(step_cfg.get('start_after', 0.0)), 0.0)
        start_after = start_after_base + (ramp_duration if align_to_ramp_end else 0.0)
        duration = max(float(step_cfg.get('duration', 0.0)), 0.0)
        repeat_step = bool(step_cfg.get('repeat', True))

        if t_after_wait < start_after:
            return base_value

        elapsed_since_start = t_after_wait - start_after
        step_active = False

        if repeat_step:
            period_default = cycle if (cycle > 0.0 and repeat_ramp) else duration if duration > 0.0 else 1.0
            period = float(step_cfg.get('period', period_default))
            if period <= 0.0:
                period = max(period_default, 1e-6)
            offset = float(step_cfg.get('offset_in_cycle', 0.0))
            if align_to_ramp_end:
                offset = (offset + ramp_duration) % period
            phase = (elapsed_since_start - offset) % period
            if duration <= 0.0:
                step_active = phase < 1e-6
            else:
                step_active = phase < duration
        else:
            if duration <= 0.0:
                step_active = elapsed_since_start >= 0.0 and elapsed_since_start < 1e-6
            else:
                step_active = 0.0 <= elapsed_since_start < duration

        if step_active:
            return base_value + amplitude
        return base_value

    raise ValueError(f"未対応の目標プロファイル: {profile}")

# ==================================================================================
# PID コントローラ（スレッドセーフ）
# ==================================================================================

class PIDController:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, max_output=10.0, min_output=-10.0):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.max_output = max_output; self.min_output = min_output
        self.prev_error = 0.0; self.integral = 0.0; self.prev_time = time.time()
        self.lock = threading.Lock()

    def update(self, setpoint, feedback):
        with self.lock:
            t = time.time(); dt = t - self.prev_time
            if dt <= 0.0: dt = 1e-6
            e = setpoint - feedback
            P = self.kp * e
            self.integral += e * dt
            if self.ki > 0:
                i_lim = self.max_output / self.ki
                self.integral = max(min(self.integral, i_lim), -i_lim)
            I = self.ki * self.integral
            D = self.kd * (e - self.prev_error) / dt
            u = P + I + D
            u = max(min(u, self.max_output), self.min_output)
            self.prev_error = e; self.prev_time = t
            return u, e, P, I, D

# ==================================================================================
# 可視化/解析
# ==================================================================================

def analyze_and_plot_response(csv_filename):
    profile_label = get_profile_label()
    print(f"応答解析を開始 ({profile_label}) : {csv_filename}")
    df = pd.read_csv(csv_filename)


    fig, axes = plt.subplots(4, 1, figsize=(12, 12))
    fig.suptitle(f'Output Response (θ) and Torques - {profile_label}')

    t = df['time'].values
    theta_ref_turn = df['theta_ref'].values
    theta_turn = df['output_pos'].values
    tau_out_calc = df['tau_out'].values if 'tau_out' in df.columns else None
    tau0 = df['motor0_torque'].values
    tau1 = df['motor1_torque'].values
    theta1_turn = df['motor0_pos'].values
    theta2_turn = df['motor1_pos'].values

    turn_to_deg = lambda arr: np.asarray(arr, dtype=float) * 360.0
    theta_ref_deg = turn_to_deg(theta_ref_turn)
    theta_deg = turn_to_deg(theta_turn)
    theta1_deg = turn_to_deg(theta1_turn)
    theta2_deg = turn_to_deg(theta2_turn)

    # θ
    axes[0].plot(t, theta_ref_deg, '--', label='θ_ref')
    axes[0].plot(t, theta_deg, '-', label='θ')
    axes[0].set_ylabel('θ [deg]')
    axes[0].legend()

    # τ_out
    axes[1].set_ylabel('τ_out [Nm]')
    plotted = False
    if tau_out_calc is not None:
        axes[1].plot(t, tau_out_calc, '-', label='τ_out')
        plotted = True
    if plotted:
        axes[1].legend()

    # τ0, τ1
    axes[2].plot(t, tau0, color='red', label='τ1')
    axes[2].plot(t, tau1, color='green', label='τ2')
    axes[2].set_ylabel('Torque [Nm]')
    axes[2].legend()

    # 内部姿勢 θ1, θ2
    axes[3].plot(t, theta1_deg, color='red', label='θ1')
    axes[3].plot(t, theta2_deg, color='green', label='θ2')
    axes[3].set_xlabel('Time [s]')
    axes[3].set_ylabel('Joint [deg]')
    axes[3].legend()

    # グリッド線なし、目盛り内向き
    for ax in axes:
        ax.grid(False)
        ax.tick_params(axis='both', direction='in', length=6, width=0.8)

    plt.tight_layout()

    os.makedirs(FIG_DIR, exist_ok=True)
    base_name = os.path.splitext(os.path.basename(csv_filename))[0]
    fig_filename = f"{base_name}{PLOT_FILENAME_SUFFIX}"
    fig_path = os.path.join(FIG_DIR, fig_filename)
    plt.savefig(fig_path, dpi=300, bbox_inches='tight', format='pdf')
    plt.show()

    print("\n" + "="*60)
    print("データファイルの処理を選択してください:")
    print("  [1] CSVとグラフの両方を保存")
    print("  [2] グラフのみ保存（CSVは破棄）")
    print("  [3] CSVのみ保存（グラフは破棄）")
    print("  [4] 両方とも破棄")
    print("="*60)

    while True:
        try:
            choice = input("選択 (1-4): ").strip()
            if choice in ['1','2','3','4']:
                break
            else:
                print("1,2,3,4 のいずれかを入力してください。")
        except KeyboardInterrupt:
            print("\n処理をキャンセルします。")
            choice = '4'
            break

    final_csv_path = csv_filename
    final_graph_path = fig_path

    try:
        if choice == '1':
            print(f"✅ CSV保存: {csv_filename}")
            print(f"✅ グラフ保存: {fig_path}")
        elif choice == '2':
            os.remove(csv_filename); final_csv_path = None
            print(f"🗑️ CSV削除: {csv_filename}")
            print(f"✅ グラフ保存: {fig_path}")
        elif choice == '3':
            os.remove(fig_path); final_graph_path = None
            print(f"✅ CSV保存: {csv_filename}")
            print(f"🗑️ グラフ削除: {fig_path}")
        elif choice == '4':
            os.remove(csv_filename); os.remove(fig_path)
            final_csv_path = None; final_graph_path = None
            print(f"🗑️ 両方削除")
    except Exception as e:
        print(f"⚠️ ファイル削除エラー: {e}")

    plt.close('all')
    return final_graph_path, final_csv_path

# ==================================================================================
# メイン
# ==================================================================================

def main():
    print("=== トルク制御PID環境 - 最小ノルム配分 版 ===")
    print(f"使用する応答プロファイル: {get_profile_label()} (type={get_active_profile_name()})")

    # ---- ODrive 接続 ----
    print("ODriveを検索中...")
    try:
        odrv0 = odrive.find_any(serial_number=ODRIVE_SERIAL['motor0'])
        odrv1 = odrive.find_any(serial_number=ODRIVE_SERIAL['motor1'])
        odrv2 = odrive.find_any(serial_number=ODRIVE_SERIAL['output'])
        print("ODrive接続完了")
    except Exception as e:
        print(f"ODrive接続エラー: {e}")
        return

    # 初期位置
    initial_position0 = odrv0.axis0.pos_vel_mapper.pos_rel
    initial_position1 = odrv1.axis0.pos_vel_mapper.pos_rel
    initial_position2 = odrv2.axis0.pos_vel_mapper.pos_rel

    # ---- モータをトルク制御モードへ ----
    print("モータをトルク制御モードに設定中...")
    odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    odrv0.axis0.config.motor.torque_constant = ODRIVE_TORQUE_CONSTANT['motor0']

    odrv1.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    odrv1.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    odrv1.axis0.config.motor.torque_constant = ODRIVE_TORQUE_CONSTANT['motor1']
    print("モータ設定完了")

    # ---- コントローラ ----
    output_pid = None
    motor_pid = {}
    if CONTROL_MODE == 'output_pid':
        output_pid = PIDController(
            kp=OUTPUT_PID['kp'],
            ki=OUTPUT_PID['ki'],
            kd=OUTPUT_PID['kd'],
            max_output=OUTPUT_PID['max_output'],
            min_output=-OUTPUT_PID['max_output'],
        )
    elif CONTROL_MODE == 'per_motor_pid':
        if ENABLE_OUTER_PID_IN_PER_MOTOR:
            output_pid = PIDController(
                kp=OUTPUT_PID['kp'],
                ki=OUTPUT_PID['ki'],
                kd=OUTPUT_PID['kd'],
                max_output=OUTPUT_PID['max_output'],
                min_output=-OUTPUT_PID['max_output'],
            )
        for key, cfg in MOTOR_PID.items():
            motor_pid[key] = PIDController(
                kp=cfg['kp'],
                ki=cfg['ki'],
                kd=cfg['kd'],
                max_output=cfg['max_output'],
                min_output=-cfg['max_output'],
            )
    else:
        raise ValueError(f"未知のCONTROL_MODE: {CONTROL_MODE}")

    # ---- ログ ----
    data_lock = threading.Lock()
    data_log = {
        'time': [],
        'motor0': {'pos': [], 'vel': [], 'torque': []},
        'motor1': {'pos': [], 'vel': [], 'torque': []},
        'output': {'pos': [], 'vel': []},
        'theta_ref': [],
        'theta_ctrl': [],
        'tau_out': [],
    }

    # ---- ヌル空間ゲイン ----
    Knu = np.diag(NULLSPACE_CONFIG['Knu_diag'])
    Kq  = np.diag(NULLSPACE_CONFIG['Kq_diag'])
    q_ref = np.array(NULLSPACE_CONFIG['q_ref'])
    freeze_idx = {'motor0': 0, 'motor1': 1}.get(FREEZE_CONFIG['motor_to_freeze'])
    freeze_kp = FREEZE_CONFIG['kp']
    freeze_kd = FREEZE_CONFIG['kd']
    torque_limits = np.array([
        SAFETY_CONFIG['max_torque0'],
        SAFETY_CONFIG['max_torque1'],
    ], dtype=float)

    start_time = time.time()
    dt_target = 1.0 / CONTROL_FREQUENCY

    print("=== 制御開始 (Ctrl+Cで停止) ===")

    try:
        while True:
            t0 = time.time()
            elapsed = t0 - start_time

            # ---- 目標 ----
            theta_ref = generate_output_reference(elapsed)

            # ---- 計測 ----
            q0 = odrv0.axis0.pos_vel_mapper.pos_rel - initial_position0
            q1 = odrv1.axis0.pos_vel_mapper.pos_rel - initial_position1
            qout = odrv2.axis0.pos_vel_mapper.pos_rel - initial_position2
            dq0 = odrv0.axis0.pos_vel_mapper.vel
            dq1 = odrv1.axis0.pos_vel_mapper.vel
            dqout = odrv2.axis0.pos_vel_mapper.vel  # 参考

            q = np.array([q0, q1])
            qdot = np.array([dq0, dq1])

            # ---- 機構行列 ----
            A = get_A(q)
            tau_cmd_prelimit = np.zeros(2)
            tau_cmd = np.zeros(2)
            theta_ctrl_cmd = theta_ref - qout

            if CONTROL_MODE == 'output_pid':
                tau_out_desired, _, _, _, _ = output_pid.update(theta_ref, qout)
                if freeze_idx is None:
                    PN = project_null(A)
                    tau_min = min_norm_torque_split(A, tau_out_desired)
                    tau_null = (-Knu @ (PN @ qdot) - Kq @ (PN @ (q - q_ref)))
                    tau_cmd_prelimit = tau_min + tau_null
                else:
                    active_idx = 1 - freeze_idx
                    a_active = float(A[0, active_idx])
                    tau_cmd_prelimit = np.zeros(2)
                    if abs(a_active) > 1e-8:
                        tau_cmd_prelimit[active_idx] = float(tau_out_desired / a_active)
                    else:
                        tau_cmd_prelimit[active_idx] = 0.0
                    tau_cmd_prelimit[freeze_idx] = float(-freeze_kp * q[freeze_idx] - freeze_kd * qdot[freeze_idx])

            elif CONTROL_MODE == 'per_motor_pid':
                theta_err_raw = theta_ref - qout
                theta_ctrl = theta_err_raw
                if output_pid is not None:
                    theta_ctrl, _, _, _, _ = output_pid.update(theta_ref, qout)
                At = A.T
                s = float(A @ At)
                if s < 1e-8:
                    raise ValueError("Mechanism matrix A is near-singular.")

                delta_q = np.zeros(2)
                if freeze_idx is None:
                    delta_q = (At / s).flatten() * theta_ctrl
                else:
                    active_idx = 1 - freeze_idx
                    a_active = float(A[0, active_idx])
                    if abs(a_active) > 1e-8:
                        delta_q[active_idx] = theta_ctrl / a_active
                    else:
                        delta_q[active_idx] = 0.0

                q_des = q_ref + delta_q

                keys = ['motor0', 'motor1']
                for idx, key in enumerate(keys):
                    if freeze_idx is not None and idx == freeze_idx:
                        tau_cmd_prelimit[idx] = float(-freeze_kp * q[idx] - freeze_kd * qdot[idx])
                    else:
                        pid = motor_pid[key]
                        u, _, _, _, _ = pid.update(q_des[idx], q[idx])
                        tau_cmd_prelimit[idx] = float(u)

                if freeze_idx is None:
                    PN = project_null(A)
                    tau_null = (-Knu @ (PN @ qdot) - Kq @ (PN @ (q - q_ref)))
                    tau_cmd_prelimit += tau_null

                theta_ctrl_cmd = float(theta_ctrl)

            else:
                raise RuntimeError(f"未対応のCONTROL_MODE: {CONTROL_MODE}")

            # ---- 飽和を考慮した再割り当て ----
            tau_cmd = project_torque_to_limits(A, tau_cmd_prelimit, torque_limits)
            tau_out_disp = motor_torque_to_output(tau_cmd)

            # ---- 出力 ----
            odrv0.axis0.controller.input_torque = tau_cmd[0]
            odrv1.axis0.controller.input_torque = tau_cmd[1]

            # ---- ログ ----
            with data_lock:
                data_log['time'].append(elapsed)
                data_log['motor0']['pos'].append(q0)
                data_log['motor0']['vel'].append(dq0)
                data_log['motor0']['torque'].append(float(tau_cmd[0]))
                data_log['motor1']['pos'].append(q1)
                data_log['motor1']['vel'].append(dq1)
                data_log['motor1']['torque'].append(float(tau_cmd[1]))
                data_log['output']['pos'].append(qout)
                data_log['output']['vel'].append(dqout)
                data_log['theta_ref'].append(theta_ref)
                data_log['theta_ctrl'].append(float(theta_ctrl_cmd))
                data_log['tau_out'].append(float(tau_out_disp))

            # ---- タイミング調整 ----
            dt = time.time() - t0
            sleep = dt_target - dt
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("\n制御を停止中...")
    finally:
        try:
            odrv0.axis0.controller.input_torque = 0.0
            odrv1.axis0.controller.input_torque = 0.0
            odrv0.axis0.requested_state = AxisState.IDLE
            odrv1.axis0.requested_state = AxisState.IDLE
        except Exception:
            pass

        # ---- CSV保存 ----
        os.makedirs(CSV_DIR, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        profile_slug = sanitize_label_for_filename(get_profile_label())
        csv_filename = os.path.join(CSV_DIR, f"{DATA_FILENAME_PREFIX}_{profile_slug}_{timestamp}.csv")
        with open(csv_filename, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                'time',
                'motor0_pos','motor0_vel','motor0_torque',
                'motor1_pos','motor1_vel','motor1_torque',
                'output_pos','output_vel',
                'theta_ref','theta_ctrl','tau_out'
            ])
            for i in range(len(data_log['time'])):
                w.writerow([
                    data_log['time'][i],
                    data_log['motor0']['pos'][i], data_log['motor0']['vel'][i], data_log['motor0']['torque'][i],
                    data_log['motor1']['pos'][i], data_log['motor1']['vel'][i], data_log['motor1']['torque'][i],
                    data_log['output']['pos'][i], data_log['output']['vel'][i],
                    data_log['theta_ref'][i], data_log['theta_ctrl'][i], data_log['tau_out'][i]
                ])
        print(f"データ保存完了: {csv_filename}")

        # ---- 可視化 ----
        try:
            print("\n=== 応答解析とグラフ表示 ===")
            final_graph_path, final_csv_path = analyze_and_plot_response(csv_filename)
            print("\n=== 最終的なファイル状況 ===")
            print(f"CSV: {final_csv_path if final_csv_path else '削除済み'}")
            print(f"FIG: {final_graph_path if final_graph_path else '削除済み'}")
        except Exception as e:
            print(f"応答解析エラー: {e}")
            print("手動でCSVファイルを確認してください。")
        print("制御終了")


if __name__ == '__main__':
    main()
