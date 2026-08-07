import argparse

import pytest

from agv_lift_height_control.cli import build_parser, main
from agv_lift_height_control.config import ConfigError


def parse(*args):
    return build_parser().parse_args(["--config", "config.json", *args])


@pytest.mark.parametrize(
    "command",
    [
        ("monitor",),
        ("observe-can",),
        ("zero-can",),
        ("calibrate-lift",),
        ("calibrate-lower", "--comfortable-valve", "0x50"),
        ("move", "--target-mm", "120", "--temporary-max-mm", "500"),
        ("manual-lower",),
        ("survey-upper", "--temporary-max-mm", "1000"),
        ("show-calibration",),
    ],
)
def test_cli_exposes_required_commands(command) -> None:
    assert parse(*command).command == command[0]


@pytest.mark.parametrize(
    "args",
    [
        ("move", "--target-mm", "nan"),
        ("move", "--target-mm", "2901"),
        ("survey-upper", "--temporary-max-mm", "inf"),
        ("calibrate-lower", "--comfortable-valve", "0x51"),
        ("monitor", "--duration-s", "0"),
    ],
)
def test_cli_rejects_nonfinite_or_out_of_range_parameters(args) -> None:
    with pytest.raises(SystemExit):
        parse(*args)


def test_move_temporary_limit_is_optional_at_parse_time_for_persistent_bundle() -> None:
    args = parse("move", "--target-mm", "100")
    assert args.temporary_max_mm is None


def test_survey_requires_temporary_limit() -> None:
    with pytest.raises(SystemExit):
        parse("survey-upper")


def test_cli_main_reports_runtime_errors_in_chinese_without_traceback(monkeypatch, capsys) -> None:
    monkeypatch.setattr(
        "agv_lift_height_control.application.run_application",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(ConfigError("配置损坏")),
    )

    exit_code = main(["--config", "missing.json", "monitor"])

    assert exit_code == 2
    assert "错误: 配置损坏" in capsys.readouterr().err
