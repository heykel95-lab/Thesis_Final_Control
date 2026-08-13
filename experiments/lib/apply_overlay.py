#!/usr/bin/env python3
"""Apply an experiment overlay onto a copy of the controller parameter files.

usage: apply_overlay.py <overlay.txt> <params_dir>

The overlay lists only the keys that differ from the nominal set. Every key
must already exist in exactly one of the parameter files: the controller's
files are disjoint by design, so a key appearing twice, or not at all, means
the overlay is wrong and we stop rather than run a silently wrong experiment.
"""

import os
import re
import sys

def param_files(params_dir):
    """Every parameter file in the directory, whatever they are called.

    The controller splits its parameters by topic and renames/adds files as the
    layout evolves; the duplicate check below is what actually enforces
    disjointness, so listing the directory is both simpler and safer than
    keeping a copy of the file list here.
    """
    return sorted(
        name for name in os.listdir(params_dir) if name.endswith(".conf")
    )


def read_overlay(path):
    pairs = []
    with open(path) as f:
        for raw in f:
            line = raw.split("#")[0].strip()
            if not line:
                continue
            if "=" not in line:
                sys.exit(f"overlay line is not 'key = value': {raw.rstrip()}")
            key, value = line.split("=", 1)
            pairs.append((key.strip(), value.strip()))
    return pairs


def key_line_re(key):
    # Matches 'key = value' with optional surrounding spaces, ignoring
    # comments that follow on the same line.
    return re.compile(rf"^(\s*{re.escape(key)}\s*=\s*)([^#\n]*)(.*)$")


def apply_overlay(overlay_path, params_dir):
    pairs = read_overlay(overlay_path)
    if not pairs:
        return 0

    contents = {}
    for name in param_files(params_dir):
        with open(os.path.join(params_dir, name)) as f:
            contents[name] = f.readlines()

    applied = 0
    for key, value in pairs:
        pattern = key_line_re(key)
        hits = []
        for name, lines in contents.items():
            for i, line in enumerate(lines):
                if pattern.match(line):
                    hits.append((name, i))

        if len(hits) == 0:
            sys.exit(f"ERROR: key '{key}' not found in any parameter file. "
                     f"Check the spelling against params/.")
        if len(hits) > 1:
            where = ", ".join(f"{n}:{i + 1}" for n, i in hits)
            sys.exit(f"ERROR: key '{key}' found in several places ({where}). "
                     f"The parameter files are meant to be disjoint.")

        name, i = hits[0]
        m = pattern.match(contents[name][i])
        contents[name][i] = f"{m.group(1)}{value}{m.group(3)}\n"
        applied += 1

    for name, lines in contents.items():
        with open(os.path.join(params_dir, name), "w") as f:
            f.writelines(lines)

    return applied


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    n = apply_overlay(sys.argv[1], sys.argv[2])
    print(f"applied {n} overrides")
