#!/usr/bin/env python3
"""
Platform-aware wrapper for MuJoCo-dependent scripts.
Handles the requirement that MuJoCo viewer scripts need mjpython on macOS.
"""

import os
import sys
import platform
import subprocess


def get_platform_python_command():
    """Get the appropriate Python command for the current platform."""
    if platform.system() == "Darwin":  # macOS
        return "mjpython"
    else:  # Linux/Ubuntu
        return sys.executable


def run_with_platform_python(script_module, script_function="main"):
    """
    Run a script with the appropriate Python interpreter for the platform.

    Args:
        script_module: Module path (e.g., "rcp_motion.robots.so101.controller.so101_motion")
        script_function: Function name to call (default: "main")
    """
    current_platform = platform.system()

    if current_platform == "Darwin":  # macOS
        is_mjpython = "mjpython" in sys.executable or os.path.basename(sys.executable) == "mjpython"

        if not is_mjpython:
            try:
                subprocess.run(["which", "mjpython"], check=True, capture_output=True)
            except subprocess.CalledProcessError:
                print("Error: mjpython not found!")
                print("MuJoCo viewer requires mjpython on macOS.")
                print("Make sure MuJoCo is properly installed and mjpython is in PATH.")
                sys.exit(1)

            # Re-execute with mjpython, preserving sys.argv arguments
            cmd = ["mjpython", "-c",
                   f"import sys; sys.argv = {repr(sys.argv)}; "
                   f"from {script_module} import {script_function}; {script_function}()"]
            try:
                subprocess.run(cmd, check=True)
                sys.exit(0)
            except subprocess.CalledProcessError as e:
                print(f"Error running script with mjpython: {e}")
                sys.exit(e.returncode)
            except KeyboardInterrupt:
                print("\nScript interrupted by user")
                sys.exit(130)
        else:
            # Already running under mjpython
            _run_module(script_module, script_function)
    else:  # Linux/Ubuntu
        _run_module(script_module, script_function)


def _run_module(script_module, script_function):
    """Import and run a module function."""
    try:
        module = __import__(script_module, fromlist=[script_function])
        main_func = getattr(module, script_function)
        main_func()
    except ImportError as e:
        print(f"Error importing {script_module}: {e}")
        sys.exit(1)
    except AttributeError:
        print(f"Error: {script_function} function not found in {script_module}")
        sys.exit(1)
    except Exception as e:
        print(f"Error running script: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


def create_mujoco_wrapper(script_module):
    """Create a wrapper function for MuJoCo-based scripts."""
    def wrapper_main():
        run_with_platform_python(script_module)
    return wrapper_main


# =============================================================================
# Entry Point Wrapper Functions
# =============================================================================

# SO101 robot
so101_motion_main = create_mujoco_wrapper("rcp_motion.robots.so101.controller.so101_motion")
so101_inference_main = create_mujoco_wrapper("rcp_motion.robots.so101.controller.so101_inference")

# FR3 robot
fr3_motion_main = create_mujoco_wrapper("rcp_motion.robots.franka.controller.fr3_motion")

# RM75 robot
rm75_motion_main = create_mujoco_wrapper("rcp_motion.robots.realman.controller.rm75_motion")


# Entry point functions for pyproject.toml
def so101_motion():
    """Entry point for so101-motion command."""
    so101_motion_main()


def so101_inference():
    """Entry point for so101-inference command."""
    so101_inference_main()


def fr3_motion():
    """Entry point for fr3-motion command."""
    fr3_motion_main()


def rm75_motion():
    """Entry point for rm75-motion command."""
    rm75_motion_main()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        script_name = sys.argv[1]
        if script_name == "so101-motion":
            so101_motion()
        elif script_name == "so101-inference":
            so101_inference()
        elif script_name == "fr3-motion":
            fr3_motion()
        elif script_name == "rm75-motion":
            rm75_motion()
        else:
            print(f"Unknown script: {script_name}")
            sys.exit(1)
    else:
        print("Usage: python mjpython_wrapper.py [so101-motion|so101-inference|fr3-motion|rm75-motion]")
        sys.exit(1)
