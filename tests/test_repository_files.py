from pathlib import Path
from subprocess import run


REPOSITORY_ROOT = Path(__file__).parents[1]


def is_ignored(path: str) -> bool:
    result = run(
        ["git", "check-ignore", "-q", "--no-index", path],
        cwd=REPOSITORY_ROOT,
        check=False,
    )
    return result.returncode == 0


def test_gitignore_keeps_example_but_excludes_runtime_data() -> None:
    assert is_ignored("config/runtime.json")
    assert not is_ignored("config/example.json")
    assert is_ignored("calibration-lift.json")
    assert is_ignored("state-height.json")
    assert is_ignored("lift-calibration-draft.json")
    assert is_ignored("lower-calibration-draft.json")
    assert is_ignored("upper-survey-draft.json")
    assert is_ignored("agv-lift-height-control.lock")
    assert is_ignored("height-log.csv")


def test_runtime_dependencies_include_modbus_serial_transport() -> None:
    pyproject = (REPOSITORY_ROOT / "pyproject.toml").read_text(encoding="utf-8")

    assert '"pymodbus[serial]>=3.6,<4"' in pyproject


def test_readme_describes_precharged_limited_travel_lift_calibration() -> None:
    readme = (REPOSITORY_ROOT / "README.md").read_text(encoding="utf-8")

    assert "1 次预充压" in readme
    assert "随后执行 3 次正式测量" in readme
    assert "总共敲击 4 次 `u`" in readme
    assert "每次通电 100 ms" in readme
    assert "全零观察 700 ms" in readme
    assert "观察期可以松开" in readme
    assert "观察结束净位移至少 1 mm" in readme
    assert "响应延迟不得超过 300 ms" in readme
    assert "相对起升行程" in readme
    assert "schema v3" in readme
    assert "以 5% 递增到 80%" not in readme


def test_maintenance_map_links_new_timing_and_tui_to_tests() -> None:
    maintenance = (REPOSITORY_ROOT / "docs" / "维护地图.md").read_text(
        encoding="utf-8"
    )

    assert "`LIFT_CALIBRATION_PWM`" in maintenance
    assert "`LIFT_PRECHARGE_REPEATS`" in maintenance
    assert "`LIFT_PULSE_S`" in maintenance
    assert "`LIFT_SETTLE_S`" in maintenance
    assert "`render_period_s`" in maintenance
    assert "完整 800 ms" in maintenance
    assert "schema v3" in maintenance
    assert (
        "test_lift_session_precharges_then_records_three_delayed_measurements"
        in maintenance
    )
    assert "test_foreground_runtime_controls_at_50hz_but_renders_about_5hz" in maintenance
    assert "单次非阻塞完整帧" in maintenance
