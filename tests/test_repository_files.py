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
