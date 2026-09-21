#!/usr/bin/env python3
"""Regenerate the thesis/presentation CoC position plot from archived reports.

Requires three complete trials at every requested position. The default
includes +/-90 and +/-100 mm; no missing point is estimated or interpolated.
Uses only the Python standard library plus pdflatex/pdftocairo for rendering.
"""

import argparse
import csv
import hashlib
import json
import math
import re
import statistics
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
POSITIONS = [-100, -90, -80, -40, -20, -10, 0, 10, 20, 40, 80, 90, 100]


def settings(directory):
    result = {}
    for path in sorted(directory.glob("*.conf")):
        for raw in path.read_text().splitlines():
            line = raw.split("#", 1)[0].strip()
            if "=" in line:
                key, value = map(str.strip, line.split("=", 1))
                result[key] = value
    return result


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def collect(root, positions):
    rows, provenance = [], []
    for direction in ("pos", "neg"):
        for position in positions:
            suffix = f"{'m' if position < 0 else 'p'}{abs(position):03d}"
            run = f"P2_t1_{direction}_{suffix}"
            for repeat in ("r01", "r02", "r03"):
                trial = root / "experiments" / "results" / run / repeat
                report = trial / "terminal.log"
                if not report.is_file():
                    raise ValueError(f"Missing measurement: {run}/{repeat}")
                text = report.read_text(errors="replace")
                recorded = dict((k.strip(), v.strip()) for line in
                                (trial / "provenance.txt").read_text().splitlines()
                                if ":" in line for k, v in [line.split(":", 1)])
                if recorded.get("exit_status") != "0" or "stop: time | t=5.0 s" not in text:
                    raise ValueError(f"Incomplete contact trial: {run}/{repeat}")
                match = re.search(r"deviation components.*?before=\[(.*?)\].*?after=\[(.*?)\]", text)
                if not match:
                    raise ValueError(f"Missing normal-error vectors: {run}/{repeat}")
                entry = [-float(v) for v in match[1].split(",")]
                final = [-float(v) for v in match[2].split(",")]
                if len(entry) != 3 or len(final) != 3 or not all(map(math.isfinite, entry + final)):
                    raise ValueError(f"Invalid normal-error vectors: {run}/{repeat}")
                params = settings(trial / "params_effective")
                if (not math.isclose(-1000 * float(params["compliance_center_offset_ee_y"]), position, abs_tol=1e-9)
                        or float(params["tool_target_offset_tangent1_deg"]) != (10 if direction == "pos" else -10)):
                    raise ValueError(f"Archived condition differs from label: {run}/{repeat}")
                if position and (params["compliance_center_in_tool_frame"] != "1"
                                 or params["compliance_lever_in_surface_frame"] != "0"):
                    raise ValueError(f"Expected tool-frame centre: {run}/{repeat}")
                rows.append(dict(run_id=run, repeat=repeat, direction=direction,
                                 position_mm=position, entry_t1_deg=entry[0],
                                 final_t1_deg=final[0], source_resolution_deg=0.01))
                provenance.append(dict(run_id=run, repeat=repeat,
                    report=str(report.relative_to(root)), report_sha256=digest(report),
                    acquisition=recorded, parameters=params,
                    parameter_sha256={p.name: digest(p) for p in sorted((trial / "params_effective").glob("*.conf"))}))
    groups = []
    for direction in ("pos", "neg"):
        for position in positions:
            samples = [row for row in rows if row["direction"] == direction and row["position_mm"] == position]
            group = dict(run_id=samples[0]["run_id"], direction=direction, position_mm=position, n=len(samples))
            for key in ("entry_t1_deg", "final_t1_deg"):
                group[key + "_mean"] = statistics.mean(row[key] for row in samples)
                group[key + "_sd"] = statistics.stdev(row[key] for row in samples)
            groups.append(group)
    return rows, groups, provenance


def source(groups, positions, presentation=False):
    limit = max(abs(p) for p in positions) + 8
    width = "13.5" if limit > 88 else "11.5"
    # Keep the original y range unless new measured error bars require more room.
    ymax = max(11, math.ceil(max(abs(g["final_t1_deg_mean"]) + g["final_t1_deg_sd"] for g in groups)) + 1)
    text = r'''% Generated from archived contact-error endpoint reports; three repeats, sample SD.
\begin{tikzpicture}
\begin{axis}[
    width=@WIDTH@cm, height=7.0cm,
    xmin=-@LIMIT@, xmax=@LIMIT@, ymin=-@YMAX@, ymax=@YMAX@,
    xtick={@TICKS@}, ytick={@YTICKS@},
    xlabel={Tangential CoC Position, \(r_{c,t_2}\) [mm]},
    ylabel={Angular Error About \(t_1\), \(\theta_{\mathrm{err},t_1}\) [\(^\circ\)]},
    ymajorgrids=true, xmajorgrids=true,
    grid style={gray!25, very thin},
    x grid style={gray!65, thin, densely dotted},
    tick label style={font=\footnotesize},
    label style={font=\footnotesize},
    xticklabel style={rotate=0, anchor=north, font=\fontsize{8}{10}\selectfont},
    legend style={font=\scriptsize, draw=none, fill=none,
                  cells={anchor=west}, at={(0.5,-0.24)}, anchor=north},
    every axis plot/.append style={thick, mark size=2.3pt},
  ]
'''
    for key, value in {"WIDTH": width, "LIMIT": limit, "YMAX": ymax,
                       "TICKS": ",".join(map(str, positions)),
                       "YTICKS": ",".join(map(str, range(-ymax + ymax % 2, ymax, 2)))}.items():
        text = text.replace(f"@{key}@", str(value))
    for direction, colour, marker in (("pos", "black", "o"), ("neg", "blue!55!black", "square")):
        subset = [row for row in groups if row["direction"] == direction]
        text += (r"\addplot[" + colour + ", mark=" + marker + r''', mark options={fill=white},
         error bars/.cd, y dir=both, y explicit] coordinates {
''')
        text += "\n".join(f"  ({row['position_mm']},{row['final_t1_deg_mean']:.9f}) +- (0,{row['final_t1_deg_sd']:.9f})" for row in subset) + "};\n"
        entry = statistics.mean(row["entry_t1_deg_mean"] for row in subset)
        label = (("Positive" if direction == "pos" else "Negative") + rf" entry tilt \(({entry:+.2f}^\circ)\)"
                 if presentation else rf"Measured Angular Offset, \(\theta_{{\mathrm{{meas}},t_1}}={entry:.2f}^\circ\)")
        text += "\\addlegendentry{" + label + "}\n"
    return text + "\\end{axis}\n\\end{tikzpicture}\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--positions", type=int, nargs="+", default=POSITIONS)
    parser.add_argument("--out-dir", type=Path, default=ROOT / "figures" / "coc_position")
    parser.add_argument("--summary-dir", type=Path, default=ROOT / "experiments" / "derived" / "coc_position")
    parser.add_argument("--no-render", action="store_true")
    args = parser.parse_args()
    positions = sorted(set(args.positions))
    # Validate every source before writing any figure or summary.
    rows, groups, provenance = collect(args.root, positions)
    args.summary_dir.mkdir(parents=True, exist_ok=True)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for name, values in (("per_trial_results.csv", rows), ("grouped_results.csv", groups)):
        with (args.summary_dir / name).open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(values[0]))
            writer.writeheader()
            writer.writerows(values)
    (args.summary_dir / "source_provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    preamble = ("\\documentclass[tikz,border=4pt]{standalone}\n"
                "\\usepackage{amsmath,pgfplots,lmodern}\n\\pgfplotsset{compat=1.16}\n\\begin{document}\n")
    for name, presentation in (("results_case_d_panels", False), ("CoC_position", True)):
        body = source(groups, positions, presentation)
        (args.out_dir / (name + ".tex")).write_text(body if not presentation else preamble + body + "\\end{document}\n")
        if not args.no_render:
            render = name if presentation else "render_" + name
            if not presentation:
                (args.out_dir / (render + ".tex")).write_text(preamble + body + "\\end{document}\n")
            result = subprocess.run(["pdflatex", "-interaction=nonstopmode", "-halt-on-error", render + ".tex"],
                                    cwd=args.out_dir, capture_output=True, text=True)
            if result.returncode:
                raise RuntimeError(result.stdout[-4000:])
            if not presentation:
                (args.out_dir / (name + ".pdf")).write_bytes((args.out_dir / (render + ".pdf")).read_bytes())
            subprocess.run(["pdftocairo", "-png", "-singlefile", "-r", "200", name + ".pdf", name], cwd=args.out_dir, check=True)
            subprocess.run(["pdftocairo", "-svg", name + ".pdf", name + ".svg"], cwd=args.out_dir, check=True)
    print(f"Wrote {len(groups)} positions/entry conditions from {len(rows)} complete trials to {args.out_dir}")


if __name__ == "__main__":
    main()
