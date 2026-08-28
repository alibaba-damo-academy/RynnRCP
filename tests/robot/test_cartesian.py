"""Tests for dependency-free Cartesian pose helpers."""

from __future__ import annotations

import math

import pytest

from rynnrcp.robot.cartesian import (
    Pose3D,
    matrix_multiply,
    matrix_to_quaternion,
    matrix_transpose,
    normalized_quaternion,
    number_vector,
    quaternion_to_matrix,
    transform_rotation_basis,
)


IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def test_pose3d_round_trip() -> None:
    pose = Pose3D.from_value(
        {"position": [1, 2, 3], "orientation_quat_xyzw": [0, 0, 0, 2]}
    )
    assert pose.position == (1.0, 2.0, 3.0)
    # Quaternion is normalized during parsing.
    assert pose.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)
    assert pose.to_value() == {
        "position": [1.0, 2.0, 3.0],
        "orientation_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


def test_pose3d_rejects_invalid_values() -> None:
    with pytest.raises(ValueError, match="must be an object"):
        Pose3D.from_value([1, 2, 3])
    with pytest.raises(ValueError, match="position must contain 3 numbers"):
        Pose3D.from_value({"position": [1], "orientation_quat_xyzw": [0, 0, 0, 1]})


def test_number_vector_validation() -> None:
    assert number_vector([1, 2], 2, "v") == (1.0, 2.0)
    with pytest.raises(ValueError, match="must contain 2 numbers"):
        number_vector("ab", 2, "v")
    with pytest.raises(ValueError, match="must contain 2 numbers"):
        number_vector([1, 2, 3], 2, "v")
    with pytest.raises(ValueError, match="finite numbers"):
        number_vector([1.0, math.inf], 2, "v")


def test_normalized_quaternion_rejects_zero_length() -> None:
    with pytest.raises(ValueError, match="non-zero length"):
        normalized_quaternion([0, 0, 0, 0])


def test_quaternion_matrix_round_trip_identity() -> None:
    matrix = quaternion_to_matrix([0, 0, 0, 1])
    assert matrix == IDENTITY
    assert matrix_to_quaternion(matrix) == pytest.approx((0.0, 0.0, 0.0, 1.0))


@pytest.mark.parametrize(
    "quaternion",
    [
        # 180-degree rotations exercise the non-trace branches.
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        # Generic rotation exercises the trace branch.
        (0.1830127, 0.1830127, 0.6830127, 0.6830127),
    ],
)
def test_quaternion_matrix_round_trip(quaternion) -> None:
    normalized = normalized_quaternion(quaternion)
    matrix = quaternion_to_matrix(normalized)
    recovered = matrix_to_quaternion(matrix)
    # q and -q describe the same rotation.
    direct = all(a == pytest.approx(b, abs=1e-9) for a, b in zip(recovered, normalized))
    negated = all(a == pytest.approx(-b, abs=1e-9) for a, b in zip(recovered, normalized))
    assert direct or negated


def test_matrix_multiply_and_transpose() -> None:
    rotation = quaternion_to_matrix([0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
    multiplied = matrix_multiply(rotation, IDENTITY)
    for row in range(3):
        assert multiplied[row] == pytest.approx(rotation[row], abs=1e-12)
    # R * R^T = I for rotation matrices.
    product = matrix_multiply(rotation, matrix_transpose(rotation))
    for row in range(3):
        for column in range(3):
            assert product[row][column] == pytest.approx(IDENTITY[row][column], abs=1e-12)


def test_transform_rotation_basis_with_axis_swap() -> None:
    # Swap the x and y axes (handedness-changing basis).
    basis = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    rotation = quaternion_to_matrix([0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
    transformed = transform_rotation_basis(rotation, basis)
    # Rotation about +z becomes rotation about -z in the swapped frame.
    expected = matrix_transpose(rotation)
    for row in range(3):
        for column in range(3):
            assert transformed[row][column] == pytest.approx(
                expected[row][column], abs=1e-12
            )
