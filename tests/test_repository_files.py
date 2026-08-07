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


def test_readme_describes_limited_travel_lift_calibration() -> None:
    readme = (REPOSITORY_ROOT / "README.md").read_text(encoding="utf-8")

    assert "只执行 3 次固定 40% PWM" in readme
    assert "每次通电 100 ms" in readme
    assert "全零观察 700 ms" in readme
    assert "观察期可以松开" in readme
    assert "相对起升行程" in readme
    assert "旧版 schema v1 起升草稿" in readme
    assert "以 5% 递增到 80%" not in readme


def test_maintenance_map_links_new_timing_and_tui_to_tests() -> None:
    maintenance = (REPOSITORY_ROOT / "docs" / "维护地图.md").read_text(
        encoding="utf-8"
    )

    assert "`LIFT_CALIBRATION_PWM`" in maintenance
    assert "`LIFT_PULSE_S`" in maintenance
    assert "`LIFT_SETTLE_S`" in maintenance
    assert "`render_period_s`" in maintenance
    assert "test_lift_session_runs_three_100ms_pulses_with_700ms_settle" in maintenance
    assert "test_foreground_runtime_controls_at_50hz_but_renders_about_5hz" in maintenance
    assert "单次非阻塞完整帧" in maintenance
