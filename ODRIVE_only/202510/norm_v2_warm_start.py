"""
トルク制御PID環境 - ウォームスタート版
既存の norm_v2 を利用しつつ、ステップ前に小さいプリロード動作を入れて
静止摩擦を越えた状態からステップ応答を計測するためのラッパースクリプト。

使い方:
    python norm_v2_warm_start.py
"""

from __future__ import annotations

import copy

import norm_v2 as base_control

# ウォームスタート用パラメータ
WARM_STEP_CONFIG = {
    "initial_wait": 0.5,       # 制御開始直後の待機[秒]
    "pre_ramp": 0.4,           # 0 -> preload_target へのランプ時間[秒]
    "pre_hold": 0.3,           # プリロード値で維持する時間[秒]
    "step_ramp": 0.2,          # プリロード -> ステップ目標へのランプ時間[秒]
    "step_hold": 2.0,          # ステップを保持する時間[秒]
    "return_ramp": 0.4,        # ステップ値 -> 0 へのランプ時間[秒]
    "rest_hold": 0.6,          # 0で休止する時間[秒]
    "preload_target": 0.02,    # プリロード時の目標角[turn]
    "step_target": 0.05,       # ステップ目標角[turn]
}

# base_control のプロファイル設定を情報用に更新（解析時の参照など）
base_control.REFERENCE_PROFILE = copy.deepcopy(base_control.REFERENCE_PROFILE)
base_control.REFERENCE_PROFILE["file_label"] = "warm_start"
base_control.REFERENCE_PROFILE["custom"] = {"description": "warm_start profile"}
base_control.REFERENCE_PROFILE["active_profile"] = "custom"
base_control.REFERENCE_PROFILE["warm_start_params"] = copy.deepcopy(WARM_STEP_CONFIG)


def generate_warm_step(elapsed_time: float) -> float:
    """
    プリロード → ステップ → 戻し → 休止 のサイクルを生成。
    """
    cfg = WARM_STEP_CONFIG
    if elapsed_time < cfg["initial_wait"]:
        return 0.0

    t = elapsed_time - cfg["initial_wait"]
    cycle = (
        cfg["pre_ramp"] + cfg["pre_hold"] + cfg["step_ramp"]
        + cfg["step_hold"] + cfg["return_ramp"] + cfg["rest_hold"]
    )
    tau = t % cycle

    if tau < cfg["pre_ramp"]:
        return cfg["preload_target"] * (tau / cfg["pre_ramp"])
    tau -= cfg["pre_ramp"]

    if tau < cfg["pre_hold"]:
        return cfg["preload_target"]
    tau -= cfg["pre_hold"]

    if tau < cfg["step_ramp"]:
        ratio = tau / cfg["step_ramp"]
        return cfg["preload_target"] + (cfg["step_target"] - cfg["preload_target"]) * ratio
    tau -= cfg["step_ramp"]

    if tau < cfg["step_hold"]:
        return cfg["step_target"]
    tau -= cfg["step_hold"]

    if tau < cfg["return_ramp"]:
        return cfg["step_target"] * (1.0 - tau / cfg["return_ramp"])

    return 0.0


# base_control の目標生成関数を差し替え
base_control.generate_output_reference = generate_warm_step  # type: ignore[assignment]


if __name__ == "__main__":
    print("=== ウォームスタート版ステップ応答 ===")
    base_control.main()
