import platform
import select
import sys


def encode_sign_magnitude(value: int, sign_bit_index: int):
    max_magnitude = (1 << sign_bit_index) - 1
    magnitude = abs(value)
    if magnitude > max_magnitude:
        raise ValueError(f"Magnitude {magnitude} exceeds {max_magnitude} (max for {sign_bit_index=})")

    direction_bit = 1 if value < 0 else 0
    return (direction_bit << sign_bit_index) | magnitude


def decode_sign_magnitude(encoded_value: int, sign_bit_index: int):
    direction_bit = (encoded_value >> sign_bit_index) & 1
    magnitude_mask = (1 << sign_bit_index) - 1
    magnitude = encoded_value & magnitude_mask
    return -magnitude if direction_bit else magnitude


def enter_pressed() -> bool:
    if platform.system() == "Windows":
        import msvcrt

        if msvcrt.kbhit():
            return msvcrt.getch() in (b"\r", b"\n")
        return False
    return bool(select.select([sys.stdin], [], [], 0)[0]) and sys.stdin.readline().strip() == ""


def move_cursor_up(lines):
    print(f"\033[{lines}A", end="")

