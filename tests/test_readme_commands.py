from pathlib import Path


README = Path(__file__).parents[1] / "README.md"


def test_readme_documents_autonomous_move_and_emergency_stop_contract() -> None:
    content = README.read_text(encoding="utf-8")

    assert "`move` 模式不需要持续按 `u` 或 `d`" in content
    assert "`EMERGENCY_STOP`" in content
    assert "解除急停后必须重新下发目标" in content
    assert "`0x217` 的唯一发送者" in content
