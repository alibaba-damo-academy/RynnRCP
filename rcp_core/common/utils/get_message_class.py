# rcp_core/common/get_message_class.py

"""
Dynamic message class loader.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module provides :func:`~rcp_core.common.get_message_class.get_message_class`, a
small utility that resolves a fully-qualified class path string into the corresponding
Python class object.

Given a type string such as ``"sensor_msgs.msg.JointState"``, it:
- splits the string into a module path (``sensor_msgs.msg``) and class name (``JointState``)
- imports the module via :mod:`importlib`
- returns the class attribute from that module

It is used throughout the codebase to turn YAML-configured message type strings into
concrete message classes for ROS 2/LCM adapters.
"""

from __future__ import annotations

import importlib


def get_message_class(type_str: str):
    """Import and return a message class from its fully-qualified name. 'sensor_msgs.msg.JointState'"""
    parts = type_str.split(".")
    module_path = ".".join(parts[:-1])
    class_name = parts[-1]
    module = importlib.import_module(module_path)
    return getattr(module, class_name)
