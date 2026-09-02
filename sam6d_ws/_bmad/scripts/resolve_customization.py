#!/usr/bin/env python3
"""Resolve BMad skill customization with a three-layer structural TOML merge.

Layers are merged from defaults to highest priority:

1. ``{skill-root}/customize.toml``
2. ``{project-root}/_bmad/custom/{skill-name}.toml``
3. ``{project-root}/_bmad/custom/{skill-name}.user.toml``

Tables deep-merge, scalar overrides replace their base value, keyed arrays of
tables merge by a shared ``code`` or ``id``, and every other array appends.
Python 3.11+ uses stdlib ``tomllib``; Python 3.10 falls back to ``tomli``.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

try:
    import tomllib
except ImportError:  # Python 3.10
    try:
        import tomli as tomllib
    except ImportError:
        sys.stderr.write(
            "error: TOML support is unavailable. Use Python 3.11+ or install tomli.\n"
        )
        sys.exit(3)


_MISSING = object()
_KEYED_MERGE_FIELDS = ("code", "id")


def find_project_root(start: Path):
    current = start.resolve()
    while True:
        if (current / "_bmad").exists() or (current / ".git").exists():
            return current
        parent = current.parent
        if parent == current:
            return None
        current = parent


def load_toml(file_path: Path, required: bool = False) -> dict:
    if not file_path.exists():
        if required:
            sys.stderr.write(f"error: required customization file not found: {file_path}\n")
            sys.exit(1)
        return {}
    try:
        with file_path.open("rb") as handle:
            parsed = tomllib.load(handle)
        if not isinstance(parsed, dict):
            if required:
                sys.stderr.write(f"error: {file_path} did not parse to a table\n")
                sys.exit(1)
            return {}
        return parsed
    except tomllib.TOMLDecodeError as error:
        level = "error" if required else "warning"
        sys.stderr.write(f"{level}: failed to parse {file_path}: {error}\n")
        if required:
            sys.exit(1)
        return {}
    except OSError as error:
        level = "error" if required else "warning"
        sys.stderr.write(f"{level}: failed to read {file_path}: {error}\n")
        if required:
            sys.exit(1)
        return {}


def _detect_keyed_merge_field(items):
    if not items or not all(isinstance(item, dict) for item in items):
        return None
    for candidate in _KEYED_MERGE_FIELDS:
        if all(item.get(candidate) is not None for item in items):
            return candidate
    return None


def _merge_by_key(base, override, key_name):
    result = []
    index_by_key = {}
    for item in base:
        if not isinstance(item, dict):
            continue
        if item.get(key_name) is not None:
            index_by_key[item[key_name]] = len(result)
        result.append(dict(item))
    for item in override:
        if not isinstance(item, dict):
            result.append(item)
            continue
        key = item.get(key_name)
        if key is not None and key in index_by_key:
            result[index_by_key[key]] = dict(item)
        else:
            if key is not None:
                index_by_key[key] = len(result)
            result.append(dict(item))
    return result


def _merge_arrays(base, override):
    base_array = base if isinstance(base, list) else []
    override_array = override if isinstance(override, list) else []
    keyed_field = _detect_keyed_merge_field(base_array + override_array)
    if keyed_field:
        return _merge_by_key(base_array, override_array, keyed_field)
    return base_array + override_array


def deep_merge(base, override):
    if isinstance(base, dict) and isinstance(override, dict):
        result = dict(base)
        for key, override_value in override.items():
            result[key] = (
                deep_merge(result[key], override_value)
                if key in result else override_value
            )
        return result
    if isinstance(base, list) and isinstance(override, list):
        return _merge_arrays(base, override)
    return override


def extract_key(data, dotted_key: str):
    current = data
    for part in dotted_key.split("."):
        if isinstance(current, dict) and part in current:
            current = current[part]
        else:
            return _MISSING
    return current


def write_json_stdout(output):
    reconfigure = getattr(sys.stdout, "reconfigure", None)
    if reconfigure is not None:
        reconfigure(encoding="utf-8")
    sys.stdout.write(json.dumps(output, indent=2, ensure_ascii=False) + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Resolve customization for a BMad skill using three-layer TOML merge."
    )
    parser.add_argument(
        "--skill", "-s", required=True,
        help="Absolute path to the skill directory (must contain customize.toml)",
    )
    parser.add_argument(
        "--key", "-k", action="append", default=[],
        help="Dotted field path to resolve (repeatable). Omit for full dump.",
    )
    args = parser.parse_args()

    skill_dir = Path(args.skill).resolve()
    defaults = load_toml(skill_dir / "customize.toml", required=True)
    project_root = find_project_root(skill_dir) or find_project_root(Path.cwd())
    team = {}
    user = {}
    if project_root:
        custom_dir = project_root / "_bmad" / "custom"
        team = load_toml(custom_dir / f"{skill_dir.name}.toml")
        user = load_toml(custom_dir / f"{skill_dir.name}.user.toml")
    merged = deep_merge(deep_merge(defaults, team), user)

    if args.key:
        output = {}
        for key in args.key:
            value = extract_key(merged, key)
            if value is not _MISSING:
                output[key] = value
    else:
        output = merged
    write_json_stdout(output)


if __name__ == "__main__":
    main()
