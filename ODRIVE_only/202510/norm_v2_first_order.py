"""
norm_v2 ラッパー: 出力θ目標に一次遅れフィルタを挿入する版。

使い方:
    python norm_v2_first_order.py
"""

from __future__ import annotations

import copy
import math
from typing import Optional

import norm_v2 as base_control


class FirstOrderLag:
    """一次遅れ (1st-order lag) でステップ入力を滑らかにするヘルパー。"""

    def __init__(self, tau: float, source) -> None:
        self.tau = max(float(tau), 0.0)
        self.source = source
        self.prev_time: Optional[float] = None
        self.prev_value: float = 0.0

    def __call__(self, elapsed_time: float) -> float:
        raw = self.source(elapsed_time)

        if self.tau <= 0.0:
            self.prev_value = raw
            self.prev_time = elapsed_time
            return raw

        if self.prev_time is None:
            self.prev_value = raw
        else:
            dt = max(elapsed_time - self.prev_time, 0.0)
            if dt > 0.0:
                alpha = math.exp(-dt / self.tau)
                self.prev_value = alpha * self.prev_value + (1.0 - alpha) * raw

        self.prev_time = elapsed_time
        return self.prev_value


def main() -> None:
    tau = 0.05  # [s] 変更したい場合はここを書き換える

    base_control.STEP_CONFIG = copy.deepcopy(base_control.STEP_CONFIG)
    base_control.STEP_CONFIG["pattern"] = base_control.STEP_CONFIG.get("pattern", "step") + "+lag"
    base_control.STEP_CONFIG["lag_tau"] = tau

    lagged_generator = FirstOrderLag(tau, base_control.generate_output_step)
    base_control.generate_output_step = lagged_generator  # type: ignore[assignment]

    print(f"=== 一次遅れ版 norm_v2 (tau = {tau:.3f} s 固定) ===")
    base_control.main()


if __name__ == "__main__":
    main()
