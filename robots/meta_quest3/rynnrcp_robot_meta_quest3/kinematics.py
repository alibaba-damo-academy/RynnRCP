"""Small dependency-free URDF kinematics and damped least-squares IK."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]
Matrix4 = tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]


@dataclass(frozen=True)
class UrdfJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: Vector3
    origin_rpy: Vector3
    axis: Vector3
    lower: float
    upper: float


@dataclass(frozen=True)
class ForwardKinematics:
    transform: Matrix4
    link_points: tuple[Vector3, ...]
    joint_origins: tuple[Vector3, ...]
    joint_axes: tuple[Vector3, ...]
    link_transforms: tuple[tuple[str, Matrix4], ...]

    @property
    def position(self) -> Vector3:
        return _translation(self.transform)

    @property
    def rotation(self) -> Matrix3:
        return _rotation(self.transform)


@dataclass(frozen=True)
class IkResult:
    joint_positions: tuple[float, ...]
    converged: bool
    iterations: int
    position_error_m: float
    rotation_error_rad: float


class UrdfKinematicChain:
    """Kinematic chain from a URDF base link to one tip link."""

    def __init__(
        self,
        urdf_path: str | Path,
        *,
        base_link: str = "fr3_link0",
        tip_link: str = "fr3_hand_tcp",
    ) -> None:
        self.urdf_path = Path(urdf_path)
        root = ET.parse(self.urdf_path).getroot()
        joints = [_parse_joint(element) for element in root.findall("joint")]
        self.joints = tuple(joints)
        by_child = {joint.child: joint for joint in joints}
        chain: list[UrdfJoint] = []
        current = tip_link
        while current != base_link:
            joint = by_child.get(current)
            if joint is None:
                raise ValueError(
                    f"URDF has no chain from {base_link} to {tip_link}"
                )
            chain.append(joint)
            current = joint.parent
        chain.reverse()
        self.base_link = base_link
        self.tip_link = tip_link
        self.chain = tuple(chain)
        self.active_joints = tuple(
            joint
            for joint in self.chain
            if joint.joint_type in {"revolute", "continuous"}
        )
        if not self.active_joints:
            raise ValueError("URDF chain has no active joints")

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.active_joints)

    @property
    def lower_limits(self) -> tuple[float, ...]:
        return tuple(joint.lower for joint in self.active_joints)

    @property
    def upper_limits(self) -> tuple[float, ...]:
        return tuple(joint.upper for joint in self.active_joints)

    def clamp(self, joint_positions: Sequence[float]) -> tuple[float, ...]:
        values = _vector(joint_positions, len(self.active_joints), "joint_positions")
        return tuple(
            max(low, min(high, value))
            for value, low, high in zip(
                values, self.lower_limits, self.upper_limits
            )
        )

    def forward(
        self,
        joint_positions: Sequence[float],
        auxiliary_joint_positions: dict[str, float] | None = None,
    ) -> ForwardKinematics:
        q = _vector(joint_positions, len(self.active_joints), "joint_positions")
        transform = _identity4()
        points: list[Vector3] = [_translation(transform)]
        joint_origins: list[Vector3] = []
        joint_axes: list[Vector3] = []
        active_index = 0
        for joint in self.chain:
            transform = _mat4_mul(
                transform,
                _transform(
                    _rotation_from_rpy(joint.origin_rpy),
                    joint.origin_xyz,
                ),
            )
            if joint.joint_type in {"revolute", "continuous"}:
                joint_origins.append(_translation(transform))
                joint_axes.append(
                    _mat3_vector(_rotation(transform), joint.axis)
                )
                transform = _mat4_mul(
                    transform,
                    _transform(
                        _rotation_from_axis_angle(
                            joint.axis, q[active_index]
                        ),
                        (0.0, 0.0, 0.0),
                    ),
                )
                active_index += 1
            points.append(_translation(transform))
        return ForwardKinematics(
            transform=transform,
            link_points=tuple(points),
            joint_origins=tuple(joint_origins),
            joint_axes=tuple(joint_axes),
            link_transforms=self._all_link_transforms(
                q, auxiliary_joint_positions
            ),
        )

    def _all_link_transforms(
        self,
        joint_positions: Sequence[float],
        auxiliary_joint_positions: dict[str, float] | None = None,
    ) -> tuple[tuple[str, Matrix4], ...]:
        positions = {
            joint.name: value
            for joint, value in zip(self.active_joints, joint_positions)
        }
        positions.update(auxiliary_joint_positions or {})
        transforms: dict[str, Matrix4] = {self.base_link: _identity4()}
        pending = list(self.joints)
        while pending:
            remaining: list[UrdfJoint] = []
            changed = False
            for joint in pending:
                parent = transforms.get(joint.parent)
                if parent is None:
                    remaining.append(joint)
                    continue
                transform = _mat4_mul(
                    parent,
                    _transform(
                        _rotation_from_rpy(joint.origin_rpy),
                        joint.origin_xyz,
                    ),
                )
                value = positions.get(joint.name, 0.0)
                if joint.joint_type in {"revolute", "continuous"}:
                    transform = _mat4_mul(
                        transform,
                        _transform(
                            _rotation_from_axis_angle(joint.axis, value),
                            (0.0, 0.0, 0.0),
                        ),
                    )
                elif joint.joint_type == "prismatic":
                    transform = _mat4_mul(
                        transform,
                        _transform(
                            (
                                (1.0, 0.0, 0.0),
                                (0.0, 1.0, 0.0),
                                (0.0, 0.0, 1.0),
                            ),
                            tuple(value * axis for axis in joint.axis),
                        ),
                    )
                transforms[joint.child] = transform
                changed = True
            if not changed:
                break
            pending = remaining
        return tuple(transforms.items())

    def solve_ik(
        self,
        target_position: Sequence[float],
        target_rotation: Sequence[Sequence[float]],
        seed: Sequence[float],
        *,
        max_iterations: int = 120,
        damping: float = 0.02,
        max_joint_step: float = 0.08,
        position_tolerance_m: float = 0.002,
        rotation_tolerance_rad: float = 0.02,
        position_weight: float = 4.0,
        rotation_weight: float = 1.0,
    ) -> IkResult:
        target_p = _vector(target_position, 3, "target_position")
        target_r = _matrix3(target_rotation, "target_rotation")
        q = list(self.clamp(seed))
        underactuated = len(q) < 6
        position_error = math.inf
        rotation_error = math.inf
        for iteration in range(1, max_iterations + 1):
            state = self.forward(q)
            position_vector = tuple(
                target_p[index] - state.position[index]
                for index in range(3)
            )
            rotation_vector = _rotation_log(
                _mat3_mul(target_r, _mat3_transpose(state.rotation))
            )
            position_error = _norm(position_vector)
            rotation_error = _norm(rotation_vector)
            if (
                not underactuated
                and position_error <= position_tolerance_m
                and rotation_error <= rotation_tolerance_rad
            ):
                return IkResult(
                    tuple(q),
                    True,
                    iteration,
                    position_error,
                    rotation_error,
                )

            jacobian = [[0.0] * len(q) for _ in range(6)]
            for column, (origin, axis) in enumerate(
                zip(state.joint_origins, state.joint_axes)
            ):
                linear = _cross(
                    axis,
                    tuple(
                        state.position[index] - origin[index]
                        for index in range(3)
                    ),
                )
                for row in range(3):
                    jacobian[row][column] = linear[row]
                    jacobian[row + 3][column] = axis[row]

            if underactuated:
                position_jacobian = jacobian[:3]
                rotation_jacobian = jacobian[3:]
                position_inverse = _damped_pseudoinverse(
                    position_jacobian, max(1e-6, damping * 0.05)
                )
                position_delta = _matrix_vector(
                    position_inverse, position_vector
                )
                nullspace = [
                    [
                        (1.0 if row == column else 0.0)
                        - sum(
                            position_inverse[row][axis]
                            * position_jacobian[axis][column]
                            for axis in range(3)
                        )
                        for column in range(len(q))
                    ]
                    for row in range(len(q))
                ]
                nullspace_rotation_jacobian = _matrix_multiply(
                    rotation_jacobian, nullspace
                )
                rotation_after_position = [
                    rotation_vector[row]
                    - sum(
                        rotation_jacobian[row][column]
                        * position_delta[column]
                        for column in range(len(q))
                    )
                    for row in range(3)
                ]
                rotation_delta_raw = _matrix_vector(
                    _damped_pseudoinverse(
                        nullspace_rotation_jacobian, damping
                    ),
                    rotation_after_position,
                )
                rotation_delta = _matrix_vector(
                    nullspace, rotation_delta_raw
                )
                if (
                    position_error <= position_tolerance_m
                    and _norm(rotation_delta) <= 1e-4
                ):
                    return IkResult(
                        tuple(q),
                        True,
                        iteration,
                        position_error,
                        rotation_error,
                    )
                if position_error > position_tolerance_m:
                    delta = position_delta
                else:
                    delta = [
                        position_delta[index]
                        + rotation_weight
                        * max(
                            -0.02,
                            min(0.02, rotation_delta[index]),
                        )
                        for index in range(len(q))
                    ]
            else:
                error = [*position_vector, *rotation_vector]
                normal = [
                    [
                        sum(
                            (
                                position_weight
                                if row < 3
                                else rotation_weight
                            )
                            ** 2
                            * jacobian[row][column_a]
                            * jacobian[row][column_b]
                            for row in range(6)
                        )
                        + (
                            damping * damping
                            if column_a == column_b
                            else 0.0
                        )
                        for column_b in range(len(q))
                    ]
                    for column_a in range(len(q))
                ]
                gradient = [
                    sum(
                        (
                            position_weight
                            if row < 3
                            else rotation_weight
                        )
                        ** 2
                        * jacobian[row][column]
                        * error[row]
                        for row in range(6)
                    )
                    for column in range(len(q))
                ]
                delta = _solve_linear(normal, gradient)
            for index, value in enumerate(delta):
                q[index] += max(-max_joint_step, min(max_joint_step, value))
            q[:] = self.clamp(q)

        return IkResult(
            tuple(q),
            underactuated and position_error <= position_tolerance_m,
            max_iterations,
            position_error,
            rotation_error,
        )


def rotation_from_rpy(rpy: Sequence[float]) -> Matrix3:
    return _rotation_from_rpy(_vector(rpy, 3, "rpy"))


def rpy_from_rotation(rotation: Sequence[Sequence[float]]) -> Vector3:
    matrix = _matrix3(rotation, "rotation")
    pitch = math.asin(max(-1.0, min(1.0, -matrix[2][0])))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        roll = math.atan2(-matrix[1][2], matrix[1][1])
        yaw = 0.0
    return roll, pitch, yaw


def rotation_multiply(left: Matrix3, right: Matrix3) -> Matrix3:
    return _mat3_mul(left, right)


def quaternion_xyzw_from_rotation(rotation: Matrix3) -> tuple[float, float, float, float]:
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        result = (
            (rotation[2][1] - rotation[1][2]) / scale,
            (rotation[0][2] - rotation[2][0]) / scale,
            (rotation[1][0] - rotation[0][1]) / scale,
            0.25 * scale,
        )
    elif rotation[0][0] > rotation[1][1] and rotation[0][0] > rotation[2][2]:
        scale = math.sqrt(
            1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]
        ) * 2.0
        result = (
            0.25 * scale,
            (rotation[0][1] + rotation[1][0]) / scale,
            (rotation[0][2] + rotation[2][0]) / scale,
            (rotation[2][1] - rotation[1][2]) / scale,
        )
    elif rotation[1][1] > rotation[2][2]:
        scale = math.sqrt(
            1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]
        ) * 2.0
        result = (
            (rotation[0][1] + rotation[1][0]) / scale,
            0.25 * scale,
            (rotation[1][2] + rotation[2][1]) / scale,
            (rotation[0][2] - rotation[2][0]) / scale,
        )
    else:
        scale = math.sqrt(
            1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]
        ) * 2.0
        result = (
            (rotation[0][2] + rotation[2][0]) / scale,
            (rotation[1][2] + rotation[2][1]) / scale,
            0.25 * scale,
            (rotation[1][0] - rotation[0][1]) / scale,
        )
    magnitude = math.sqrt(sum(value * value for value in result))
    return tuple(value / magnitude for value in result)  # type: ignore[return-value]


def _parse_joint(element: ET.Element) -> UrdfJoint:
    origin = element.find("origin")
    axis = element.find("axis")
    limit = element.find("limit")
    joint_type = str(element.attrib.get("type") or "")
    return UrdfJoint(
        name=str(element.attrib["name"]),
        joint_type=joint_type,
        parent=str(element.find("parent").attrib["link"]),  # type: ignore[union-attr]
        child=str(element.find("child").attrib["link"]),  # type: ignore[union-attr]
        origin_xyz=_parse_vector(
            None if origin is None else origin.attrib.get("xyz"),
            (0.0, 0.0, 0.0),
        ),
        origin_rpy=_parse_vector(
            None if origin is None else origin.attrib.get("rpy"),
            (0.0, 0.0, 0.0),
        ),
        axis=_parse_vector(
            None if axis is None else axis.attrib.get("xyz"),
            (1.0, 0.0, 0.0),
        ),
        lower=_joint_limit(limit, "lower", -math.inf),
        upper=_joint_limit(limit, "upper", math.inf),
    )


def _joint_limit(
    element: ET.Element | None,
    name: str,
    default: float,
) -> float:
    if element is None:
        return default
    return float(element.attrib.get(name, default))


def _parse_vector(value: str | None, default: Vector3) -> Vector3:
    if not value:
        return default
    return _vector([float(item) for item in value.split()], 3, "URDF vector")


def _vector(value: Sequence[float], length: int, name: str) -> tuple[float, ...]:
    result = tuple(float(item) for item in value)
    if len(result) != length or not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain {length} finite numbers")
    return result


def _matrix3(value: Sequence[Sequence[float]], name: str) -> Matrix3:
    rows = tuple(_vector(row, 3, name) for row in value)
    if len(rows) != 3:
        raise ValueError(f"{name} must be 3x3")
    return rows  # type: ignore[return-value]


def _identity4() -> Matrix4:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _transform(rotation: Matrix3, translation: Vector3) -> Matrix4:
    return (
        (*rotation[0], translation[0]),
        (*rotation[1], translation[1]),
        (*rotation[2], translation[2]),
        (0.0, 0.0, 0.0, 1.0),
    )


def _rotation(transform: Matrix4) -> Matrix3:
    return tuple(tuple(row[:3]) for row in transform[:3])  # type: ignore[return-value]


def _translation(transform: Matrix4) -> Vector3:
    return (
        transform[0][3],
        transform[1][3],
        transform[2][3],
    )


def _mat4_mul(left: Matrix4, right: Matrix4) -> Matrix4:
    return tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(4))
            for column in range(4)
        )
        for row in range(4)
    )  # type: ignore[return-value]


def _mat3_mul(left: Matrix3, right: Matrix3) -> Matrix3:
    return tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )  # type: ignore[return-value]


def _mat3_transpose(value: Matrix3) -> Matrix3:
    return tuple(
        tuple(value[column][row] for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def _mat3_vector(matrix: Matrix3, vector: Vector3) -> Vector3:
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def _rotation_from_rpy(rpy: Vector3) -> Matrix3:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _rotation_from_axis_angle(axis: Vector3, angle: float) -> Matrix3:
    magnitude = _norm(axis)
    x, y, z = (value / magnitude for value in axis)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus = 1.0 - cosine
    return (
        (
            cosine + x * x * one_minus,
            x * y * one_minus - z * sine,
            x * z * one_minus + y * sine,
        ),
        (
            y * x * one_minus + z * sine,
            cosine + y * y * one_minus,
            y * z * one_minus - x * sine,
        ),
        (
            z * x * one_minus - y * sine,
            z * y * one_minus + x * sine,
            cosine + z * z * one_minus,
        ),
    )


def _rotation_log(rotation: Matrix3) -> Vector3:
    cosine = max(
        -1.0,
        min(
            1.0,
            (rotation[0][0] + rotation[1][1] + rotation[2][2] - 1.0)
            / 2.0,
        ),
    )
    angle = math.acos(cosine)
    if angle < 1e-8:
        return (
            (rotation[2][1] - rotation[1][2]) / 2.0,
            (rotation[0][2] - rotation[2][0]) / 2.0,
            (rotation[1][0] - rotation[0][1]) / 2.0,
        )
    denominator = 2.0 * math.sin(angle)
    return (
        angle * (rotation[2][1] - rotation[1][2]) / denominator,
        angle * (rotation[0][2] - rotation[2][0]) / denominator,
        angle * (rotation[1][0] - rotation[0][1]) / denominator,
    )


def _cross(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _norm(value: Sequence[float]) -> float:
    return math.sqrt(sum(item * item for item in value))


def _matrix_vector(
    matrix: Sequence[Sequence[float]],
    vector: Sequence[float],
) -> list[float]:
    return [
        sum(value * vector[column] for column, value in enumerate(row))
        for row in matrix
    ]


def _matrix_multiply(
    left: Sequence[Sequence[float]],
    right: Sequence[Sequence[float]],
) -> list[list[float]]:
    columns = len(right[0])
    return [
        [
            sum(
                left[row][index] * right[index][column]
                for index in range(len(right))
            )
            for column in range(columns)
        ]
        for row in range(len(left))
    ]


def _damped_pseudoinverse(
    jacobian: Sequence[Sequence[float]],
    damping: float,
) -> list[list[float]]:
    rows = len(jacobian)
    columns = len(jacobian[0])
    normal = [
        [
            sum(
                jacobian[row][column_a] * jacobian[row][column_b]
                for row in range(rows)
            )
            + (damping * damping if column_a == column_b else 0.0)
            for column_b in range(columns)
        ]
        for column_a in range(columns)
    ]
    result = [[0.0] * rows for _ in range(columns)]
    for row in range(rows):
        column = _solve_linear(
            normal,
            [jacobian[row][index] for index in range(columns)],
        )
        for index, value in enumerate(column):
            result[index][row] = value
    return result


def _solve_linear(
    matrix: Sequence[Sequence[float]],
    vector: Sequence[float],
) -> list[float]:
    size = len(vector)
    augmented = [
        [float(item) for item in matrix[row]] + [float(vector[row])]
        for row in range(size)
    ]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise ValueError("IK linear system is singular")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        augmented[column] = [item / divisor for item in augmented[column]]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                augmented[row][index] - factor * augmented[column][index]
                for index in range(size + 1)
            ]
    return [augmented[row][-1] for row in range(size)]
