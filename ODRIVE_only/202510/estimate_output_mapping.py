from pathlib import Path
"""
推定スクリプト
モータ角度 -> 出力角度の線形写像 θ_out ≈ a1 * q0 + a2 * q1 + b を
既存のログCSVから最小二乗で推定する。
"""

import argparse
from typing import Optional, Tuple

import numpy as np
import pandas as pd


def _find_latest_csv(directory):
    dir_path = Path(directory)
    candidates = sorted(dir_path.glob("integrated_pid_torque_min_norm_20251023_220006.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
    return str(candidates[0]) if candidates else None


def _estimate_coefficients(df: pd.DataFrame) -> Tuple[np.ndarray, np.ndarray]:
    """
    θ_out = a1 * q0 + a2 * q1 + b を最小二乗で推定する。
    戻り値: (coef, stats)
        coef = [a1, a2, b]
        stats = [rmse, max_abs_error, r_squared]
    """
    required_cols = {"motor0_pos", "motor1_pos", "output_pos"}
    if not required_cols.issubset(df.columns):
        missing = required_cols - set(df.columns)
        raise ValueError(f"CSVに必要な列が見つかりません: {missing}")

    X = df[["motor0_pos", "motor1_pos"]].to_numpy()
    y = df["output_pos"].to_numpy()

    # NaNを含む行は除外
    mask = np.isfinite(X).all(axis=1) & np.isfinite(y)
    X = X[mask]
    y = y[mask]

    if X.shape[0] == 0:
        raise ValueError("有効なサンプルがありません。")

    # バイアス項を追加
    X_aug = np.column_stack([X, np.ones(len(X))])
    coef, *_ = np.linalg.lstsq(X_aug, y, rcond=None)

    y_pred = X_aug @ coef
    residuals = y - y_pred
    rmse = np.sqrt(np.mean(residuals ** 2))
    max_abs_error = float(np.max(np.abs(residuals)))
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    ss_res = np.sum(residuals ** 2)
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    stats = np.array([rmse, max_abs_error, r_squared], dtype=float)
    return coef, stats


def main() -> None:
    parser = argparse.ArgumentParser(
        description="モータ角 -> 出力角の線形写像をCSVログから推定します。"
    )
    parser.add_argument(
        "--file",
        type=Path,
        help="推定に使用するCSVファイル。指定が無ければ最新ファイルを自動選択。",
    )
    parser.add_argument(
        "--directory",
        type=Path,
        default=Path("csv"),
        help="CSVファイルを探索するディレクトリ（デフォルト: ODRIVE_only/202510/csv）",
    )
    args = parser.parse_args()

    if args.file:
        csv_files = [args.file]
    else:
        csv_files = sorted(Path(args.directory).glob("*.csv"))
        if not csv_files:
            raise FileNotFoundError(f"{args.directory} にCSVが見つかりません。")

    for csv_path in csv_files:
        if not Path(csv_path).exists():
            print(f"指定ファイルが存在しません: {csv_path}")
            continue
        print(f"\nCSV読み込み: {csv_path}")
        try:
            df = pd.read_csv(csv_path)
            coef, stats = _estimate_coefficients(df)
        except Exception as e:
            print(f"  エラー: {e}")
            continue

        print("=== 推定結果 ===")
        print(f"a1 (motor0_pos係数) : {coef[0]: .8f}")
        print(f"a2 (motor1_pos係数) : {coef[1]: .8f}")
        print(f"バイアス項 b        : {coef[2]: .8f}")

        print("=== 誤差指標 ===")
        print(f"RMSE             : {stats[0]: .6e} turn")
        print(f"最大絶対誤差     : {stats[1]: .6e} turn")
        print(f"決定係数 R^2    : {stats[2]: .6f}")


if __name__ == "__main__":
    main()
