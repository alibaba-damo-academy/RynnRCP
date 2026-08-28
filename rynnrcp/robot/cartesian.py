"""RCP Cartesian pose convention and dependency-free transform helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence


RCP_CARTESIAN_CONVENTION = "rcp_cartesian_v1"


@dataclass(frozen=True)
class Pose3D:
    """TCP pose expressed in an RCP base frame, in metres and xyzw."""

    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]

    @classmethod
    def from_value(cls, value: Any) -> "Pose3D":
        if not isinstance(value, Mapping):
            raise ValueError("ee_pose value must be an object")
        return cls(
            position=number_vector(value.get("position"), 3, "position"),
            orientation_xyzw=normalized_quaternion(
                value.get("orientation_quat_xyzw")
            ),
        )

    def to_value(self) -> dict[str, list[float]]:
        return {
            "position": list(self.position),
            "orientation_quat_xyzw": list(self.orientation_xyzw),
        }


def number_vector(
    value: Any, length: int, name: str
) -> tuple[float, ...]:
    if not isinstance(value, Sequence) or isinstance(
        value, (str, bytes, bytearray)
    ):
        raise ValueError(f"{name} must contain {length} numbers")
    if len(value) != length:
        raise ValueError(f"{name} must contain {length} numbers")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain finite numbers")
    return result


def normalized_quaternion(
    value: Any,
) -> tuple[float, float, float, float]:
    quaternion = number_vector(value, 4, "orientation_quat_xyzw")
    norm = math.sqrt(sum(item * item for item in quaternion))
    if norm <= 1e-12:
        raise ValueError("orientation_quat_xyzw must have non-zero length")
    return tuple(item / norm for item in quaternion)  # type: ignore[return-value]


def quaternion_to_matrix(
    quaternion: Sequence[float],
) -> tuple[tuple[float, float, float], ...]:
    x, y, z, w = normalized_quaternion(quaternion)
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def matrix_to_quaternion(
    matrix: Sequence[Sequence[float]],
) -> tuple[float, float, float, float]:
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (m21 - m12) / scale,
            (m02 - m20) / scale,
            (m10 - m01) / scale,
            0.25 * scale,
        )
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        quaternion = (
            0.25 * scale,
            (m01 + m10) / scale,
            (m02 + m20) / scale,
            (m21 - m12) / scale,
        )
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        quaternion = (
            (m01 + m10) / scale,
            0.25 * scale,
            (m12 + m21) / scale,
            (m02 - m20) / scale,
        )
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        quaternion = (
            (m02 + m20) / scale,
            (m12 + m21) / scale,
            0.25 * scale,
            (m10 - m01) / scale,
        )
    return normalized_quaternion(quaternion)


def matrix_multiply(
    left: Sequence[Sequence[float]],
    right: Sequence[Sequence[float]],
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        tuple(
            sum(left[row][inner] * right[inner][column] for inner in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def matrix_transpose(
    matrix: Sequence[Sequence[float]],
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        tuple(matrix[column][row] for column in range(3))
        for row in range(3)
    )


def transform_rotation_basis(
    rotation: Sequence[Sequence[float]],
    basis: Sequence[Sequence[float]],
) -> tuple[tuple[float, float, float], ...]:
    """Change rotation coordinates, including handedness-changing bases."""
    return matrix_multiply(
        matrix_multiply(basis, rotation),
        matrix_transpose(basis),
    )
