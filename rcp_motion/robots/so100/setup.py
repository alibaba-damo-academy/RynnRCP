#!/usr/bin/env python3
"""
Setup script for RynnMotion lerobot package
"""

from setuptools import setup, find_packages
import os

this_directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(this_directory, "README.md"), encoding="utf-8") as f:
    long_description = f.read()

packages = find_packages(where=".")

common_packages = find_packages(where="../../../", include=["common*"])

all_packages = packages + common_packages

setup(
    name="RynnMotion",
    version="0.1.1",
    description="RynnMotion - lerobot/lekiwi Robot Motion Control",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Robot Motion Team",
    python_requires=">=3.8",
    packages=all_packages,
    package_dir={
        "": ".",
        "common": "../../../common",
    },
    package_data={
        "common.lcm": ["*.lcm"],
        "common.lcm.lcm_types": ["*.lcm"],
    },
    include_package_data=True,
    install_requires=[
        "numpy",
        "PyYAML",
        "matplotlib",
        "scipy",
        "lcm",
    ],
    extras_require={
        "dev": [
            "black",
            "mypy",
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "lerobot-controller=controller.lerobot:main",
            "lekiwi-controller=controller.lekiwi:main",
        ],
    },
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
)
