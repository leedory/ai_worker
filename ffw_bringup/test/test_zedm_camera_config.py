"""Regression checks for the common ZED-M acquisition path."""

import ast
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def test_zedm_acquires_vga_at_15_hz_without_launching_the_remap():
    config = (PACKAGE / "config" / "common" / "zedm.yaml").read_text()
    assert "grab_resolution: 'VGA'" in config
    assert "grab_frame_rate: 15" in config
    assert "pub_frame_rate: 15.0" in config

    launch = (PACKAGE / "launch" / "camera_zed.launch.py").read_text()
    ast.parse(launch)
    assert "head_camera_remap" not in launch


def test_head_camera_remap_remains_available_as_an_entrypoint():
    assert (PACKAGE / "ffw_bringup" / "head_camera_remap.py").is_file()
    setup = (PACKAGE / "setup.py").read_text()
    assert "head_camera_remap = ffw_bringup.head_camera_remap:main" in setup
