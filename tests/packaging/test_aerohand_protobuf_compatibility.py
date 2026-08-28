"""Dependency compatibility checks for Aero Hand and RynnBot."""

from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:  # Python 3.10
    import tomli as tomllib  # type: ignore[no-redef]


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def _dependencies(package: str) -> list[str]:
    with (PROJECT_ROOT / package / "pyproject.toml").open("rb") as file:
        return tomllib.load(file)["project"]["dependencies"]


def test_aero_hand_and_rynnbot_share_a_protobuf_4_runtime() -> None:
    setup_script = (
        PROJECT_ROOT / "robots" / "tetheria_aerohand" / "setup_aero_hand.sh"
    ).read_text(encoding="utf-8")

    assert 'pip_install --no-deps "mediapipe==0.10.18"' in setup_script
    assert '"protobuf>=4.25.3,<5"' in setup_script
    assert "protobuf>=4.25.3,<5" in _dependencies("apps/rynnbot")
