"""
Common utilities and shared components for the robot motion project.
"""

from typing import List, Optional, Dict, Any
from dataclasses import dataclass, asdict, field, is_dataclass
from datetime import datetime
from copy import deepcopy
from rcp_motion.core.data.basic_data import Pose, Wrench, Twist


class RobotState:
    """
    robot data structure for storing comprehensive robot state.
    Designed for use in control, logging, visualization, and communication.
    """

    def __init__(self):
        # time
        self.sec: int = 0
        self.nanosec: int = 0
        self.utime: int = 0  # microseconds

        # joints
        self._num_joints: int = 0
        self.joint_pos: List[float] = []  # q
        self.joint_vel: List[float] = []  # dq
        self.joint_torque: List[float] = []  # torque

        # End-Effector
        self._num_end_effectors: int = 0
        self.ee_pose: List[Pose] = []
        self.ee_vel: List[Twist] = []

        # grippers
        self._num_grippers: int = 0
        self.gripper_pos: List[float] = []
        self.gripper_vel: List[float] = []
        self.gripper_force: List[float] = []

        # FT Sensor
        self._num_ft_sensors: int = 0
        self.ft_sensor_wrench: List[Wrench] = []

        # Sites, e.g., markers, tools
        self._num_sites: int = 0
        self.site_pose: List[Pose] = []  # pose × N
        self.site_vel: List[Twist] = []  # vel × N

        # additional custom data
        self.custom: Dict[str, Any] = {}

    # helper resize used by setters
    def _resize(self, lst: Optional[List[Any]], new_len: int, fill: Any):
        if lst is None:
            lst = []
        cur = len(lst)
        if cur < new_len:
            if callable(fill):
                lst.extend([fill() for _ in range(new_len - cur)])
            else:
                lst.extend([fill] * (new_len - cur))
        elif cur > new_len:
            del lst[new_len:]
        return lst

    # add property after __init__
    @property
    def num_joints(self) -> int:
        return int(self._num_joints)

    @num_joints.setter
    def num_joints(self, n: int):
        """Set number of joints and resize joint lists (pos/vel/torque)."""
        n = int(n)
        if n < 0:
            raise ValueError("num_joints must be >= 0")
        self._num_joints = n
        self.joint_pos = self._resize(getattr(self, "joint_pos", []), n, 0.0)
        self.joint_vel = self._resize(getattr(self, "joint_vel", []), n, 0.0)
        self.joint_torque = self._resize(getattr(self, "joint_torque", []), n, 0.0)

    # end effectors
    @property
    def num_end_effectors(self) -> int:
        return int(self._num_end_effectors)

    @num_end_effectors.setter
    def num_end_effectors(self, n: int):
        n = int(n)
        if n < 0:
            raise ValueError("num_end_effectors must be >= 0")
        self._num_end_effectors = n
        self.ee_pose = self._resize(getattr(self, "ee_pose", []), n, Pose)
        self.ee_vel = self._resize(getattr(self, "ee_vel", []), n, Twist)

    # grippers
    @property
    def num_grippers(self) -> int:
        return int(self._num_grippers)

    @num_grippers.setter
    def num_grippers(self, n: int):
        n = int(n)
        if n < 0:
            raise ValueError("num_grippers must be >= 0")
        self._num_grippers = n
        self.gripper_pos = self._resize(getattr(self, "gripper_pos", []), n, 0.0)
        self.gripper_vel = self._resize(getattr(self, "gripper_vel", []), n, 0.0)
        self.gripper_force = self._resize(getattr(self, "gripper_force", []), n, 0.0)

    # force/torque sensors
    @property
    def num_ft_sensors(self) -> int:
        return int(self._num_ft_sensors)

    @num_ft_sensors.setter
    def num_ft_sensors(self, n: int):
        n = int(n)
        if n < 0:
            raise ValueError("num_ft_sensors must be >= 0")
        self._num_ft_sensors = n
        self.ft_sensor_wrench = self._resize(
            getattr(self, "ft_sensor_wrench", []), n, Wrench
        )

    # sites
    @property
    def num_sites(self) -> int:
        return int(self._num_sites)

    @num_sites.setter
    def num_sites(self, n: int):
        n = int(n)
        if n < 0:
            raise ValueError("num_sites must be >= 0")
        self._num_sites = n
        self.site_pose = self._resize(getattr(self, "site_pose", []), n, Pose)
        self.site_vel = self._resize(getattr(self, "site_vel", []), n, Twist)

    def now(self):
        """Set timestamp to current time (utime in microseconds)"""
        dt = datetime.now()
        ts = dt.timestamp()
        self.sec = int(ts)
        # nanoseconds as integer
        self.nanosec = int((ts - self.sec) * 1e9)
        self.utime = int(ts * 1e6)
        return self

    def from_dict(self, d: Dict[str, Any]):
        """get data from dict"""
        for k, v in d.items():
            if not hasattr(self, k):
                continue
            if k in ("ee_pose", "site_pose") and isinstance(v, list):
                converted = []
                for item in v:
                    if isinstance(item, dict):
                        converted.append(Pose(**item))
                    else:
                        converted.append(item)
                setattr(self, k, converted)
            elif k in ("ee_vel", "site_vel") and isinstance(v, list):
                converted = []
                for item in v:
                    if isinstance(item, dict):
                        converted.append(Twist(**item))
                    else:
                        converted.append(item)
                setattr(self, k, converted)
            elif k in ("ft_sensor_wrench",) and isinstance(v, list):
                converted = []
                for item in v:
                    if isinstance(item, dict):
                        converted.append(Wrench(**item))
                    else:
                        converted.append(item)
                setattr(self, k, converted)
            else:
                setattr(self, k, v)
        try:
            self._update_counts()
        except Exception:
            pass
        return self

    def to_dict(self) -> Dict[str, Any]:
        """transform to dict"""
        out: Dict[str, Any] = {}
        for k, v in self.__dict__.items():
            if isinstance(v, list):
                lst = []
                for item in v:
                    if is_dataclass(item):
                        lst.append(asdict(item))
                    else:
                        lst.append(item)
                out[k] = lst
            elif isinstance(v, dict):
                out[k] = v.copy()
            else:
                if is_dataclass(v):
                    out[k] = asdict(v)
                else:
                    out[k] = v
        out["num_joints"] = self.num_joints
        out["num_end_effectors"] = self.num_end_effectors
        out["num_grippers"] = self.num_grippers
        out["num_ft_sensors"] = self.num_ft_sensors
        out["num_sites"] = self.num_sites
        return out

    def _update_counts(self):
        """update counts"""
        try:
            self._num_joints = len(self.joint_pos) if self.joint_pos is not None else 0
        except Exception:
            self._num_joints = 0
        self._num_end_effectors = len(self.ee_pose) if self.ee_pose is not None else 0
        self._num_grippers = (
            len(self.gripper_pos) if self.gripper_pos is not None else 0
        )
        self._num_ft_sensors = (
            len(self.ft_sensor_wrench) if self.ft_sensor_wrench is not None else 0
        )
        self._num_sites = len(self.site_pose) if self.site_pose is not None else 0

    def copy(self):
        """deep copy"""
        import copy as _copy

        return _copy.deepcopy(self)

    def __repr__(self):
        return (
            f"RobotState(joints={self.num_joints}, "
            f"eef={self.num_end_effectors}, "
            f"grippers={self.num_grippers}, "
            f"ft={self.num_ft_sensors}, "
            f"sites={self.num_sites}, "
            f"utime={self.utime})"
        )


class RobotCommand:
    """
    robot command structure for sending commands to the robot.
    Designed for use in control and communication.

    Supports template usage: keep an immutable template (dress) and call
    instantiate()/copy() to produce runtime-modifiable instances.
    """

    def __init__(self):
        # squence number
        self.seq: int = 0
        # private chunk counter
        self._chunk_size: int = 0
        # trajectory: list of RobotState points
        self.trajectory: List[RobotState] = []

        self.work_mode: int = 0  # robot workmode

        # template counts used when creating/resizing trajectory
        self._template_counts: Dict[str, int] = {
            "num_joints": 0,
            "num_grippers": 0,
            "num_end_effectors": 0,
            "num_ft_sensors": 0,
            "num_sites": 0,
        }

    # helper resize used by setters (same logic as RobotState._resize)
    def _resize(self, lst: Optional[List[Any]], new_len: int, fill: Any):
        if lst is None:
            lst = []
        cur = len(lst)
        if cur < new_len:
            if callable(fill):
                lst.extend([fill() for _ in range(new_len - cur)])
            else:
                lst.extend([fill] * (new_len - cur))
        elif cur > new_len:
            del lst[new_len:]
        return lst

    @property
    def chunk_size(self) -> int:
        return int(self._chunk_size)

    @chunk_size.setter
    def chunk_size(self, n: int):
        """Set chunk size and resize trajectory list (filled with RobotState)."""
        n = int(n)
        if n < 0:
            raise ValueError("chunk_size must be >= 0")
        self._chunk_size = n
        # prefer creating based on template counts when available
        template = None
        if any(v > 0 for v in self._template_counts.values()):
            template = self._make_template_state()
        if template:
            # fill with deep copies of template
            self.trajectory = [deepcopy(template) for _ in range(n)]
        else:
            self.trajectory = self._resize(
                getattr(self, "trajectory", []), n, RobotState
            )

    def init_with_template(self, template: RobotState):
        """
        Replace each trajectory slot with a deep copy of template.
        Call after setting chunk_size if you want prefilled templates.
        """
        for i in range(len(self.trajectory)):
            self.trajectory[i] = deepcopy(template)

    def set_trajectory_list(self, points: List[RobotState]):
        """
        Replace trajectory with provided list and update chunk_size accordingly.
        """
        self.trajectory = list(points)
        self._chunk_size = len(self.trajectory)

    def _make_template_state(self) -> RobotState:
        """Create a RobotState instance initialized to the stored template counts."""
        st = RobotState()
        # attempt to set via properties, fall back to direct attrs if properties missing
        try:
            st.num_joints = int(self._template_counts.get("num_joints", 0))
        except Exception:
            st._num_joints = int(self._template_counts.get("num_joints", 0))
            st.joint_pos = [0.0] * st._num_joints
            st.joint_vel = [0.0] * st._num_joints
            st.joint_torque = [0.0] * st._num_joints
        try:
            st.num_grippers = int(self._template_counts.get("num_grippers", 0))
        except Exception:
            st._num_grippers = int(self._template_counts.get("num_grippers", 0))
            st.gripper_pos = [0.0] * st._num_grippers
            st.gripper_vel = [0.0] * st._num_grippers
            st.gripper_force = [0.0] * st._num_grippers
        try:
            st.num_end_effectors = int(
                self._template_counts.get("num_end_effectors", 0)
            )
        except Exception:
            st._num_end_effectors = int(
                self._template_counts.get("num_end_effectors", 0)
            )
            st.ee_pose = [Pose() for _ in range(st._num_end_effectors)]
            st.ee_vel = [Twist() for _ in range(st._num_end_effectors)]
        try:
            st.num_ft_sensors = int(self._template_counts.get("num_ft_sensors", 0))
        except Exception:
            st._num_ft_sensors = int(self._template_counts.get("num_ft_sensors", 0))
            st.ft_sensor_wrench = [Wrench() for _ in range(st._num_ft_sensors)]
        try:
            st.num_sites = int(self._template_counts.get("num_sites", 0))
        except Exception:
            st._num_sites = int(self._template_counts.get("num_sites", 0))
            st.site_pose = [Pose() for _ in range(st._num_sites)]
            st.site_vel = [Twist() for _ in range(st._num_sites)]
        return st

    def init_from_dofs(
        self,
        num_joints: int,
        num_grippers: int = 0,
        num_end_effectors: int = 0,
        num_ft_sensors: int = 0,
        num_sites: int = 0,
        chunk_size: int = 0,
        template: Optional[RobotState] = None,
    ):
        """
        Initialize internal template counts and create trajectory of length chunk_size.

        - Stores template counts used later when changing chunk_size.
        - If template is provided, deep-copy it into each trajectory slot.
          Otherwise a fresh RobotState with appropriate counts is created.
        - Ensures each RobotState in trajectory has its counts set (num_joints, num_grippers, ...).

        Returns self for chaining.
        """
        # normalize ints
        num_j = int(num_joints)
        num_g = int(num_grippers)
        num_ee = int(num_end_effectors)
        num_ft = int(num_ft_sensors)
        num_s = int(num_sites)
        chunk_n = int(chunk_size)

        # store template counts
        self._template_counts = {
            "num_joints": num_j,
            "num_grippers": num_g,
            "num_end_effectors": num_ee,
            "num_ft_sensors": num_ft,
            "num_sites": num_s,
        }

        # prepare a base template RobotState with requested sizes
        if template is None:
            base = RobotState()
            # set counts via properties if available, otherwise set attributes directly
            try:
                base.num_joints = num_j
            except Exception:
                base._num_joints = num_j
                base.joint_pos = [0.0] * num_j
                base.joint_vel = [0.0] * num_j
                base.joint_torque = [0.0] * num_j
            try:
                base.num_grippers = num_g
            except Exception:
                base._num_grippers = num_g
                base.gripper_pos = [0.0] * num_g
                base.gripper_vel = [0.0] * num_g
                base.gripper_force = [0.0] * num_g
            try:
                base.num_end_effectors = num_ee
            except Exception:
                base._num_end_effectors = num_ee
                base.ee_pose = [Pose() for _ in range(num_ee)]
                base.ee_vel = [Twist() for _ in range(num_ee)]
            try:
                base.num_ft_sensors = num_ft
            except Exception:
                base._num_ft_sensors = num_ft
                base.ft_sensor_wrench = [Wrench() for _ in range(num_ft)]
            try:
                base.num_sites = num_s
            except Exception:
                base._num_sites = num_s
                base.site_pose = [Pose() for _ in range(num_s)]
                base.site_vel = [Twist() for _ in range(num_s)]
        else:
            # use provided template but ensure counts match requested counts
            base = deepcopy(template)
            try:
                base.num_joints = num_j
            except Exception:
                pass
            try:
                base.num_grippers = num_g
            except Exception:
                pass
            try:
                base.num_end_effectors = num_ee
            except Exception:
                pass
            try:
                base.num_ft_sensors = num_ft
            except Exception:
                pass
            try:
                base.num_sites = num_s
            except Exception:
                pass

        # fill trajectory with deep copies of base template
        self.trajectory = [deepcopy(base) for _ in range(chunk_n)]
        self._chunk_size = len(self.trajectory)
        return self

    def to_dict(self) -> Dict[str, Any]:
        """Serialize RobotCommand (trajectory items serialized via RobotState.to_dict when available)."""
        out: Dict[str, Any] = {}
        out["chunk_size"] = self.chunk_size
        out["work_mode"] = int(self.work_mode)
        out["template_counts"] = dict(self._template_counts)
        traj_list = []
        for item in self.trajectory:
            if hasattr(item, "to_dict") and callable(getattr(item, "to_dict")):
                traj_list.append(item.to_dict())
            elif is_dataclass(item):
                traj_list.append(asdict(item))
            else:
                traj_list.append(item)
        out["trajectory"] = traj_list
        return out

    def from_dict(self, d: Dict[str, Any]):
        """Deserialize RobotCommand. Expects trajectory entries as dicts convertible by RobotState.from_dict."""
        # restore template counts if present
        if "template_counts" in d:
            self._template_counts = dict(d.get("template_counts", {}))
        # set work_mode if present
        if "work_mode" in d:
            self.work_mode = int(d.get("work_mode", 0))
        # build trajectory
        traj = []
        for item in d.get("trajectory", []):
            if isinstance(item, dict):
                traj.append(RobotState().from_dict(item))
            else:
                traj.append(item)
        self.trajectory = traj
        # set chunk size property to keep consistent (will resize if needed)
        if "chunk_size" in d:
            self.chunk_size = int(d.get("chunk_size", len(self.trajectory)))
        else:
            # ensure private counter matches actual list
            self._chunk_size = len(self.trajectory)
        return self

    def instantiate(self) -> "RobotCommand":
        """Return a deep-copy instance for runtime use (template -> runtime instance)."""
        return deepcopy(self)

    @classmethod
    def from_template(cls, template: "RobotCommand") -> "RobotCommand":
        """Create a new instance from an existing template (alias)."""
        return deepcopy(template)

    def validate(self) -> bool:
        """Basic sanity checks. Extend as needed."""
        if not isinstance(self._chunk_size, int) or self._chunk_size < 0:
            return False
        if self.trajectory is None:
            return False
        if self._chunk_size != len(self.trajectory):
            return False
        # further checks could verify RobotState sizes if desired
        return True

    def copy(self):
        """deep copy (alias)"""
        return deepcopy(self)

    def __repr__(self):
        return f"RobotCommand(chunk_size={self.chunk_size}, work_mode={self.work_mode}, traj_len={len(self.trajectory)})"
