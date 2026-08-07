"""命令行参数解析；硬件工厂与运行编排位于 application。"""

from __future__ import annotations

import argparse
import math
import sys
from collections.abc import Sequence

from .calibration import LOWER_VALVE_LEVELS, CalibrationError
from .can_pump import CanLinkError, CanPumpError
from .config import ConfigError


def _bounded_float(name: str, minimum: float, maximum: float):
    def parse(value: str) -> float:
        try:
            result = float(value)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(f"{name} 必须是数字") from exc
        if not math.isfinite(result) or not minimum <= result <= maximum:
            raise argparse.ArgumentTypeError(
                f"{name} 必须是 {minimum:g}..{maximum:g} 内的有限数字"
            )
        return result

    return parse


def _comfortable_valve(value: str) -> int:
    try:
        parsed = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("舒适阀值必须是整数或 0x 十六进制") from exc
    if parsed not in LOWER_VALVE_LEVELS:
        levels = ", ".join(f"0x{item:02X}" for item in LOWER_VALVE_LEVELS)
        raise argparse.ArgumentTypeError(f"舒适阀值必须来自实测计划: {levels}")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m agv_lift_height_control",
        description="AGV 升降高度控制 SSH 前台工具",
    )
    parser.add_argument("--config", required=True, help="严格 JSON 配置文件路径")
    subparsers = parser.add_subparsers(dest="command", required=True)

    monitor = subparsers.add_parser("monitor", help="只读高度，不打开 CAN")
    monitor.add_argument(
        "--duration-s",
        type=_bounded_float("监视时长", 0.001, 86400.0),
        default=60.0,
    )

    observe = subparsers.add_parser("observe-can", help="只读观察 CAN 0x197")
    observe.add_argument(
        "--duration-s",
        type=_bounded_float("观察时长", 0.001, 86400.0),
        default=60.0,
    )

    zero = subparsers.add_parser("zero-can", help="仅运行 NMT 与 0x217 全零安全窗口")
    zero.add_argument(
        "--duration-s",
        type=_bounded_float("归零时长", 5.0, 86400.0),
        default=5.0,
    )

    lift = subparsers.add_parser("calibrate-lift", help="起升标定并保存草稿")
    lift.add_argument(
        "--temporary-max-mm",
        required=True,
        type=_bounded_float("起升标定临时最大高度", 0.001, 2900.0),
    )
    subparsers.add_parser("calibrate-lower", help="读取起升草稿并完成下降动作测量")
    confirm_lower = subparsers.add_parser(
        "confirm-lower", help="不打开硬件，确认已实测舒适下降阀值"
    )
    confirm_lower.add_argument("--comfortable-valve", required=True, type=_comfortable_valve)

    move = subparsers.add_parser("move", help="仅自动起升到目标高度")
    move.add_argument(
        "--target-mm", required=True, type=_bounded_float("目标高度", 0.0, 2900.0)
    )
    move.add_argument(
        "--temporary-max-mm",
        type=_bounded_float("临时最大高度", 0.001, 2900.0),
    )

    subparsers.add_parser("manual-lower", help="死手授权人工下降")

    survey = subparsers.add_parser("survey-upper", help="测量建议软上限")
    survey.add_argument(
        "--temporary-max-mm",
        required=True,
        type=_bounded_float("临时最大高度", 0.001, 2900.0),
    )
    confirm_upper = subparsers.add_parser(
        "confirm-upper", help="不打开硬件，从测量草稿确认软上限"
    )
    confirm_upper.add_argument(
        "--soft-limit-mm",
        required=True,
        type=_bounded_float("确认软上限", 0.001, 2900.0),
    )

    subparsers.add_parser("show-calibration", help="显示最终标定结果")
    return parser


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    return build_parser().parse_args(argv)


def main(argv: Sequence[str] | None = None, *, dependencies=None) -> int:
    from .application import run_application

    try:
        return run_application(parse_args(argv), dependencies=dependencies)
    except (
        CalibrationError,
        CanLinkError,
        CanPumpError,
        ConfigError,
        OSError,
        RuntimeError,
        ValueError,
    ) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 2
