#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Apply head_joint delta_pulse from YAML into ros2_control xacro Homing Offset."""

from __future__ import annotations

import argparse
import re
import shutil
import sys
from pathlib import Path
from typing import Dict

try:
    import yaml
except ImportError as e:  # pragma: no cover
    print('This script requires PyYAML (e.g. apt install python3-yaml).', file=sys.stderr)
    raise SystemExit(1) from e


DEFAULT_JOINT_TO_GPIO: Dict[str, str] = {
    'head_joint1': 'dxl61',
    'head_joint2': 'dxl62',
}

def _default_xacro() -> Path | None:
    """Default xacro when running from source tree (same layout as ai_worker workspace)."""
    inner = Path(__file__).resolve().parent
    rep = inner.parent
    if not (rep / 'setup.py').exists():
        return None
    p = (
        rep.parent
        / 'ffw_description'
        / 'ros2_control'
        / 'ffw_bg2_rev4_follower'
        / 'ffw_bg2_follower.ros2_control.xacro'
    )
    return p if p.is_file() else None


def _default_xacro_from_share() -> Path | None:
    """Installed ffw_description share path (runtime / overlay)."""
    try:
        from ament_index_python.packages import (  # type: ignore[import-untyped]
            PackageNotFoundError,
            get_package_share_directory,
        )
    except ImportError:
        return None
    try:
        share = Path(get_package_share_directory('ffw_description'))
    except PackageNotFoundError:
        return None
    p = (
        share
        / 'ros2_control'
        / 'ffw_bg2_rev4_follower'
        / 'ffw_bg2_follower.ros2_control.xacro'
    )
    return p if p.is_file() else None


def default_ros2_control_xacro_path() -> Path | None:
    """Resolve default BG2 follower ros2_control xacro (source tree or install)."""
    return _default_xacro() or _default_xacro_from_share()


def resolve_ros2_control_xacro_path(param_value: str | None) -> Path | None:
    """Non-empty param: must exist as a file. Empty: same as default_ros2_control_xacro_path()."""
    s = str(param_value or '').strip()
    if s:
        p = Path(s).expanduser()
        return p if p.is_file() else None
    return default_ros2_control_xacro_path()


def _default_delta_yaml() -> Path | None:
    inner = Path(__file__).resolve().parent
    rep = inner.parent
    candidates = [
        rep / 'config' / 'ffw_head_joint_pulse_delta.yaml',
        inner / 'config' / 'ffw_head_joint_pulse_delta.yaml',
    ]
    for c in candidates:
        if c.is_file():
            return c
    return None


def load_delta_pulse(path: Path) -> Dict[str, int]:
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'{path}: root must be a mapping')
    dp = data.get('delta_pulse')
    if not isinstance(dp, dict):
        raise ValueError(f'{path}: missing or invalid delta_pulse mapping')
    out: Dict[str, int] = {}
    for k, v in dp.items():
        out[str(k)] = int(v)
    return out


def _gpio_inner_param_indent(inner: str) -> str:
    indent = '        '
    for line in inner.splitlines():
        m = re.match(r'^(\s+)<param\b', line)
        if m:
            indent = m.group(1)
    return indent


def patch_gpio_homing_offset(
    text: str, gpio_name: str, value: int,
) -> tuple[str, bool, str]:
    """Set Homing Offset inside a <gpio name="..."> block.

    Replaces an existing Homing Offset param; if missing, appends one before </gpio>.
    Returns (new_text, ok, action) with action 'replaced', 'added', or ''.
    """
    gpio_re = re.compile(
        rf'(<gpio name="{re.escape(gpio_name)}">)(.*?)(</gpio>)',
        re.DOTALL,
    )
    m = gpio_re.search(text)
    if not m:
        return text, False, ''
    inner = m.group(2)
    inner_new, n = re.subn(
        r'(<param name="Homing Offset">)-?\d+(</param>)',
        rf'\g<1>{int(value)}\g<2>',
        inner,
        count=1,
    )
    action = ''
    if n == 1:
        action = 'replaced'
    else:
        inner_new, n2 = re.subn(
            r'(<param name="Homing Offset">)[^<]*(</param>)',
            rf'\g<1>{int(value)}\g<2>',
            inner,
            count=1,
        )
        if n2 == 1:
            action = 'replaced'
        else:
            indent = _gpio_inner_param_indent(inner)
            line = f'{indent}<param name="Homing Offset">{int(value)}</param>'
            inner_new = inner.rstrip() + '\n' + line + '\n'
            action = 'added'

    new_text = text[: m.start()] + m.group(1) + inner_new + m.group(3) + text[m.end() :]
    return new_text, True, action


def run(
    delta_path: Path,
    xacro_path: Path,
    joint_gpio: Dict[str, str],
    dry_run: bool,
    backup: bool,
) -> int:
    deltas = load_delta_pulse(delta_path)
    text = xacro_path.read_text(encoding='utf-8')
    original = text
    applied: list[tuple[str, str, int, str]] = []

    for joint, gpio in joint_gpio.items():
        if joint not in deltas:
            print(f'Error: delta_pulse has no key {joint!r} in {delta_path}', file=sys.stderr)
            return 2
        val = deltas[joint]
        text, ok, how = patch_gpio_homing_offset(text, gpio, val)
        if not ok:
            print(
                f'Error: could not find <gpio name="{gpio}"> in {xacro_path} '
                f'(joint {joint})',
                file=sys.stderr,
            )
            return 3
        applied.append((joint, gpio, val, how))

    if text == original:
        print('No changes (unexpected).', file=sys.stderr)
        return 4

    for joint, gpio, val, how in applied:
        print(f'{joint} -> {gpio}: Homing Offset = {val} ({how})')

    if dry_run:
        print('Dry run: not writing file.')
        return 0

    if backup:
        bak = xacro_path.with_suffix(xacro_path.suffix + '.bak')
        shutil.copy2(xacro_path, bak)
        print(f'Backup: {bak}')

    xacro_path.write_text(text, encoding='utf-8')
    print(f'Updated {xacro_path}')
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description=(
            'Read delta_pulse from ffw_head_joint_pulse_delta.yaml and write '
            'those values into Homing Offset for the mapped DXL gpio blocks '
            'in ffw_bg2_follower.ros2_control.xacro (default paths). '
            'Override paths with --delta-yaml / --xacro or use '
            'apply_head_homing_offset.launch.py (ros2_control_xacro_path, '
            'delta_yaml_path).'
        ),
    )
    p.add_argument(
        '--delta-yaml',
        type=Path,
        default=None,
        help='YAML from save_delta (default: package config/ffw_head_joint_pulse_delta.yaml)',
    )
    p.add_argument(
        '--xacro',
        type=Path,
        default=None,
        help=(
            'Target ros2_control xacro '
            '(default: ffw_description/ros2_control/ffw_bg2_rev4_follower/'
            'ffw_bg2_follower.ros2_control.xacro under ai_worker source tree)'
        ),
    )
    p.add_argument(
        '--gpio-map',
        metavar='JOINT:GPIO',
        nargs='*',
        default=[],
        help='Override joint→gpio (default: head_joint1:dxl61 head_joint2:dxl62)',
    )
    p.add_argument(
        '--dry-run',
        action='store_true',
        help='Print planned Homing Offset values only; do not write.',
    )
    p.add_argument(
        '--backup',
        action='store_true',
        help='Write a .bak copy of the xacro before overwriting.',
    )
    args = p.parse_args(argv)

    delta_path = args.delta_yaml or _default_delta_yaml()
    if delta_path is None:
        print(
            'Error: could not find ffw_head_joint_pulse_delta.yaml; '
            'pass --delta-yaml',
            file=sys.stderr,
        )
        return 1

    xacro_path = args.xacro or _default_xacro()
    if xacro_path is None:
        print(
            'Error: could not find default xacro (source tree); pass --xacro',
            file=sys.stderr,
        )
        return 1

    joint_gpio = dict(DEFAULT_JOINT_TO_GPIO)
    for item in args.gpio_map:
        if ':' not in item:
            print(f'Error: bad --gpio-map entry {item!r} (expected JOINT:GPIO)', file=sys.stderr)
            return 1
        j, g = item.split(':', 1)
        joint_gpio[j.strip()] = g.strip()

    return run(delta_path, xacro_path, joint_gpio, args.dry_run, args.backup)


if __name__ == '__main__':
    raise SystemExit(main())
