import json
import os
from dataclasses import asdict, dataclass
from pathlib import Path

from .motors_bus import MotorCalibration

ROBOTS = "robots"
TELEOPERATORS = "teleoperators"
_hf_home = Path(os.getenv("HF_HOME", Path.home() / ".cache" / "huggingface")).expanduser()
HF_LEROBOT_CALIBRATION = Path(
    os.getenv("HF_LEROBOT_CALIBRATION", _hf_home / "lerobot" / "calibration")
).expanduser()


@dataclass(kw_only=True)
class DeviceConfig:
    id: str | None = None
    calibration_dir: Path | None = None


class CalibratedDevice:
    name: str
    calibration_group: str

    def __init__(self, config: DeviceConfig):
        self.id = config.id
        self.calibration_dir = (
            config.calibration_dir
            if config.calibration_dir
            else HF_LEROBOT_CALIBRATION / self.calibration_group / self.name
        )
        self.calibration_dir.mkdir(parents=True, exist_ok=True)
        self.calibration_fpath = self.calibration_dir / f"{self.id}.json"
        self.calibration: dict[str, MotorCalibration] = {}
        if self.calibration_fpath.is_file():
            self._load_calibration()

    def __str__(self) -> str:
        return f"{self.id} {self.__class__.__name__}"

    def _load_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        with open(fpath) as f:
            self.calibration = {
                motor: MotorCalibration(**values) for motor, values in json.load(f).items()
            }

    def _save_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        with open(fpath, "w") as f:
            json.dump({motor: asdict(cal) for motor, cal in self.calibration.items()}, f, indent=4)

