#!/usr/bin/env python3
"""
identify_friction_model.py
--------------------------
Estimate a simple friction model from logged motor data.

Model:
    torque = tau_c * sign(velocity) + B * velocity + tau_bias

The script fits the parameters using (optionally robust) linear regression.
Create `identify_friction_model_config.json` next to this script to store default CLI
arguments and avoid long command lines.
"""
import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.linear_model import LinearRegression, RANSACRegressor
from sklearn.metrics import r2_score


DEFAULT_CONFIG_FILE = Path(__file__).with_name("identify_friction_model_config.json")
DEFAULT_CSV_PATTERN = "csv/integrated_pid_torque_min_norm_*.csv"


def _load_config(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Config file must contain a JSON object.")

    normalized = {}
    base_dir = path.parent
    mapping = {
        "csv": ("csv", Path),
        "torque_col": ("torque_col", str),
        "vel_col": ("vel_col", str),
        "vel_threshold": ("vel_threshold", float),
        "flip_torque": ("flip_torque", bool),
        "flip_velocity": ("flip_velocity", bool),
        "no_bias": ("no_bias", bool),
        "no_ransac": ("no_ransac", bool),
        "residual_threshold": ("residual_threshold", float),
        "min_samples": ("min_samples", float),
        "output_json": ("output_json", Path),
        "no_plot": ("no_plot", bool),
    }

    for key, value in data.items():
        if key not in mapping:
            continue
        dest, caster = mapping[key]
        if caster is Path and value is not None:
            candidate = Path(value)
            if not candidate.is_absolute():
                candidate = (base_dir / candidate).resolve()
            normalized[dest] = candidate
        elif caster is bool:
            normalized[dest] = bool(value)
        elif value is not None:
            normalized[dest] = caster(value)
        else:
            normalized[dest] = None
    return normalized


def parse_args(argv=None) -> argparse.Namespace:
    if argv is None:
        argv = sys.argv[1:]

    config_parser = argparse.ArgumentParser(add_help=False)
    config_parser.add_argument(
        "--config",
        type=Path,
        help="Path to JSON config file. Defaults to identify_friction_model_config.json next to this script if present.",
    )
    cfg_args, remaining = config_parser.parse_known_args(argv)
    config_path = cfg_args.config
    if config_path is None and DEFAULT_CONFIG_FILE.exists():
        config_path = DEFAULT_CONFIG_FILE

    config_defaults = {}
    if config_path is not None:
        try:
            config_defaults = _load_config(config_path)
        except Exception as exc:
            config_parser.error(f"Failed to load config file ({config_path}): {exc}")

    parser = argparse.ArgumentParser(
        description="Identify Coulomb + viscous friction parameters from a CSV log."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=config_path,
        help="Path to JSON config file.",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        help="Path to CSV log (must include torque and velocity columns).",
    )
    parser.add_argument(
        "--torque-col",
        default="motor0_torque",
        help="Column name that stores motor torque [Nm].",
    )
    parser.add_argument(
        "--vel-col",
        default="motor0_vel",
        help="Column name that stores motor velocity [turn/s].",
    )
    parser.add_argument(
        "--vel-threshold",
        type=float,
        default=0.02,
        help="Discard samples whose |velocity| is below this value to avoid static friction effects.",
    )
    parser.add_argument(
        "--flip-torque",
        action="store_true",
        help="Flip torque column sign (useful if CSV convention is opposite).",
    )
    parser.add_argument(
        "--no-flip-torque",
        dest="flip_torque",
        action="store_false",
        help="Force-disable torque sign flip even if config enables it.",
    )
    parser.add_argument(
        "--flip-velocity",
        action="store_true",
        help="Flip velocity column sign (useful if CSV convention is opposite).",
    )
    parser.add_argument(
        "--no-flip-velocity",
        dest="flip_velocity",
        action="store_false",
        help="Force-disable velocity sign flip even if config enables it.",
    )
    parser.add_argument(
        "--no-bias",
        action="store_true",
        help="Disable constant bias term (forces tau_bias = 0).",
    )
    parser.add_argument(
        "--bias",
        dest="no_bias",
        action="store_false",
        help="Force-enable bias term even if config disables it.",
    )
    parser.add_argument(
        "--no-ransac",
        action="store_true",
        help="Disable RANSAC; use plain linear regression.",
    )
    parser.add_argument(
        "--ransac",
        dest="no_ransac",
        action="store_false",
        help="Force-enable RANSAC even if config disables it.",
    )
    parser.add_argument(
        "--residual-threshold",
        type=float,
        default=None,
        help="Manual residual threshold for RANSAC [Nm]. If omitted, it is estimated from data.",
    )
    parser.add_argument(
        "--min-samples",
        type=float,
        default=0.6,
        help="Minimum fraction of samples that define a RANSAC model (0 < value <= 1).",
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        default=None,
        help="Optional path to write identified parameters as JSON.",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip plotting (useful for headless runs).",
    )
    parser.add_argument(
        "--plot",
        dest="no_plot",
        action="store_false",
        help="Force-enable plotting even if config disables it.",
    )

    if config_defaults:
        parser.set_defaults(**config_defaults)

    args = parser.parse_args(argv)
    args.config = config_path
    return args


def build_design_matrix(velocity: np.ndarray) -> np.ndarray:
    signs = np.sign(velocity)
    signs[signs == 0.0] = 0.0
    return np.column_stack((signs, velocity))


def main() -> int:
    args = parse_args()

    csv_path = args.csv
    if csv_path is None:
        search_root = Path.cwd()
        candidates = sorted(
            search_root.glob(DEFAULT_CSV_PATTERN),
            key=lambda p: p.stat().st_mtime,
        )
        if not candidates:
            print(
                "No CSV specified and no logs found matching pattern "
                f"{DEFAULT_CSV_PATTERN} in {search_root}",
                file=sys.stderr,
            )
            return 1
        csv_path = candidates[-1]
        print(f"Auto-selected latest CSV: {csv_path}")
    else:
        csv_path = csv_path.expanduser()

    args.csv = csv_path

    if not csv_path.exists():
        print(f"CSV file not found: {csv_path}", file=sys.stderr)
        return 1

    df = pd.read_csv(csv_path)
    if args.torque_col not in df.columns or args.vel_col not in df.columns:
        print("Requested columns are not present in the CSV.", file=sys.stderr)
        print(f"Available columns: {list(df.columns)}", file=sys.stderr)
        return 1

    torque = df[args.torque_col].to_numpy(dtype=float)
    velocity = df[args.vel_col].to_numpy(dtype=float)
    if getattr(args, "flip_torque", False):
        torque = -torque
    if getattr(args, "flip_velocity", False):
        velocity = -velocity

    mask = np.abs(velocity) >= args.vel_threshold
    if not np.any(mask):
        print("No samples exceed the velocity threshold; lower the threshold.", file=sys.stderr)
        return 1

    torque_fit = torque[mask]
    velocity_fit = velocity[mask]
    X = build_design_matrix(velocity_fit)

    base_reg = LinearRegression(fit_intercept=not args.no_bias)
    if args.no_ransac:
        reg = base_reg
        reg.fit(X, torque_fit)
        inlier_mask = np.ones_like(torque_fit, dtype=bool)
    else:
        if not (0.0 < args.min_samples <= 1.0):
            print("min-samples must lie in the interval (0, 1].", file=sys.stderr)
            return 1
        if args.residual_threshold is None:
            residual_guess = max(0.01, np.std(torque_fit) * 0.5)
        else:
            residual_guess = args.residual_threshold
        reg = RANSACRegressor(
            base_reg,
            min_samples=args.min_samples,
            residual_threshold=residual_guess,
            random_state=0,
        )
        reg.fit(X, torque_fit)
        inlier_mask = reg.inlier_mask_

    if hasattr(reg, "estimator_"):
        est = reg.estimator_
    else:
        est = reg

    tau_c = float(est.coef_[0])
    viscous = float(est.coef_[1])
    tau_bias = float(est.intercept_) if not args.no_bias else 0.0

    torque_pred = tau_c * np.sign(velocity_fit) + viscous * velocity_fit + tau_bias
    r2 = r2_score(torque_fit[inlier_mask], torque_pred[inlier_mask])
    inlier_ratio = float(np.mean(inlier_mask))

    print("=== Friction model parameters ===")
    print(f" tau_coulomb : {tau_c:.4f} [Nm]")
    print(f" B_viscous   : {viscous:.4f} [Nm/(turn/s)]")
    print(f" tau_bias    : {tau_bias:.4f} [Nm]")
    print(f" inlier_ratio: {inlier_ratio:.3f}")
    print(f" R^2 (inliers): {r2:.3f}")

    if args.output_json:
        payload = {
            "tau_coulomb": tau_c,
            "B_viscous": viscous,
            "tau_bias": tau_bias,
            "inlier_ratio": inlier_ratio,
            "r2_inliers": r2,
            "csv": str(args.csv),
            "torque_col": args.torque_col,
            "vel_col": args.vel_col,
            "vel_threshold": args.vel_threshold,
        }
        args.output_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print(f"Wrote parameters to: {args.output_json}")

    if not args.no_plot:
        fig, ax = plt.subplots(figsize=(7, 4))
        vel_sorted = np.linspace(
            np.min(velocity_fit), np.max(velocity_fit), num=400
        )
        torque_curve = tau_c * np.sign(vel_sorted) + viscous * vel_sorted + tau_bias
        ax.scatter(
            velocity_fit[inlier_mask],
            torque_fit[inlier_mask],
            s=10,
            label="Inliers",
            alpha=0.7,
        )
        if not np.all(inlier_mask):
            ax.scatter(
                velocity_fit[~inlier_mask],
                torque_fit[~inlier_mask],
                s=10,
                label="Outliers",
                alpha=0.7,
                color="tab:red",
            )
        ax.plot(vel_sorted, torque_curve, "k-", label="Fitted model")
        ax.set_xlabel("Velocity [turn/s]")
        ax.set_ylabel("Torque [Nm]")
        ax.set_title("Friction identification result")
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
