# CoC sweep extension for the thesis and presentation

Completed on 2026-09-21: all 24 trials at -100, -90, +90 and +100 mm
finished the five-second contact phase and exited successfully. Each of the
eight conditions has three repeats. The plot now contains 78 trials over
26 position/entry conditions; the full main-contact summary has 93 trials
over 31 settings.

This is a later-session extension of the archived campaign. Saved surface,
tool geometry, initial posture, gripper and safety parameters match the
corresponding +/-80 mm references. Contact, approach and damping settings
were translated from those references to the current parameter names.
Acquisition used controller revision `e23bd03`; each trial records its exact
provenance. Saved calibration equality does not independently establish an
unchanged physical tool mounting or surface placement.

The first attempt was rejected by the robot before motion because its mode
disallowed automatic error recovery. That attempt remains under
`experiments/results_aborted/P2_t1_pos_m090_r01_20260921_mode_rejected/`
and contributes no measurement. Acquisition succeeded after the robot was
unlocked. All 16 active parameter files were restored byte for byte.

The extension adds four CoC positions for each of the +10 and -10 degree
commanded entry tilts about t1: eight settings and 24 trials at three repeats.
Each setup is under `experiments/setups/P2_t1_{pos,neg}_{m090,p090,m100,p100}`.
`run_ids.txt` lists only these eight settings, with 90 mm before 100 mm.
They are separate from the general campaign's `INDEX.txt`.

## Coordinate convention

The existing Case-D plot uses `plot position [mm] = -1000 * offset_ee_y [m]`.
The centre remains attached to the tool, as in the archived +/-80 mm trials.
These settings therefore use `compliance_center_in_tool_frame = 1`.

| Position on the existing plot | `compliance_center_offset_ee_y` | Equivalent live tool-frame command |
|---|---:|---|
| -100 mm | +0.100 m | `pc2 100` |
| -90 mm | +0.090 m | `pc2 90` |
| +90 mm | -0.090 m | `pc2 -90` |
| +100 mm | -0.100 m | `pc2 -100` |

The `rc2`/`r2` commands specify a surface-frame component. Their numbers must
not be substituted directly for the plot positions above.

## Parameters and acquisition

Regenerate these files without running the robot:

```sh
python3 experiments/lib/generate_coc_extension_setups.py
```

Each overlay extends the corresponding same-sign +/-80 mm `r01` archive.
It preserves the archived contact, approach, operator-hold, automatic-damping,
null-space and tool-orientation parameters, translating their historical
names to the current schema. Within those parameters only the centre's
tool-frame y component changes. In particular:

- Contact stiffness: Kp = [2000, 2000, 350] N/m and KR = [5, 5, 50] N m/rad.
- Contact duration: 5 s; virtual penetration: 0.240 m at 0.080 m/s.
- Automatic contact damping enabled, factor 1; manual damping lower bound off.
- Orientation timeout: 8 s, matching the archive rather than the current
  default of 5 s.
- No pre-contact hold; pre-grinding hold enabled; startup mode `s`.
- Null-space mode 3, damping 2 N m s/rad, conditioning gain 1 N m.

The generator does not change the active parameter directory. Surface and
tool-geometry calibration, initial posture, gripper and safety settings remain
those in the active parameter files and must be recorded by the trial runner.
Check them against the archived reference for comparability; any changed
calibration or physical tool mounting makes this a later-session extension.
The current controller revision is also recorded separately from the original
campaign. `manifest.json` records the baseline file hashes and generated
overlay hashes. Its `status_at_preparation` records the original preparation state.
`completed_trials.json` and `measurement_audit.json` record completed acquisition.

When starting acquisition, run each setting with repeat indices 1, 2 and 3.
For example, from the repository root:

```sh
./experiments/run.sh P2_t1_pos_p090 1
```

Select `s` to run the sequence, then `e` once the contact report has printed
and pre-grinding hold is active. `run.sh` archives the terminal report, raw
CSV, effective parameters and provenance, then restores the active parameters.
Keep aborted trials out of the plotted means and retain their records.

The dedicated runner checks progress without commanding the robot:

```sh
python3 experiments/lib/run_coc_extension.py
```

Once the robot is ready, collect the missing trials with:

```sh
python3 experiments/lib/run_coc_extension.py --run
```

It validates and skips completed trials, checks each new archive against its
overlay and five-second raw contact log, and stops on the first failure.
Use `--limit 1` to collect one new trial. Full console output is retained in
`acquisition.log`, and validated trials are listed in `completed_trials.json`.

## Regenerating the plot from a checkout

The archive and this generator live in the same repository. Regenerate both figure variants with:

```sh
git lfs pull
python3 analysis/make_coc_position_figure.py
```

The generator needs Python 3, pdfLaTeX with pgfplots and Latin Modern, and
Poppler's `pdftocairo`. It exports editable LaTeX and PDF/PNG/SVG versions for
the thesis and presentation under `figures/coc_position/`. Per-trial values,
grouped means/sample SD and source hashes are written to
`experiments/derived/coc_position/`. Every requested condition requires three
successful five-second terminal reports with matching saved parameters.
Missing measurements stop generation before outputs are written.

To reproduce the original +/-80 mm plot:

```sh
python3 analysis/make_coc_position_figure.py \
  --positions -80 -40 -20 -10 0 10 20 40 80 \
  --out-dir /tmp/coc_baseline/figures --summary-dir /tmp/coc_baseline/data
```

All 18 existing plotted means and sample standard deviations were checked
against the thesis source and matched at its nine-decimal stored precision.

## Measured extension

Angular error theta_err,t1 is the negative of the controller's normal-alignment
error component. Values below are mean +/- sample SD across three trials,
in degrees, from 0.01-degree terminal endpoints.

| Plot position [mm] | Positive entry | Negative entry |
|---:|---:|---:|
| -100 | 31.77 +/- 1.33 | 2.76 +/- 0.14 |
| -90 | 10.52 +/- 0.39 | 2.48 +/- 0.03 |
| +90 | 1.11 +/- 0.06 | -11.91 +/- 0.57 |
| +100 | 0.83 +/- 0.06 | -33.98 +/- 1.16 |

The smallest positive-entry mean magnitude is 0.83 degrees at +100 mm,
52.6% below the 1.75-degree TCP reference. The negative-entry minimum
remains at +10 mm. The larger opposing errors at 100 mm require a wider
vertical axis; no result is clipped or replaced by a prediction.

Validation checks all 24 effective parameter sets and raw contact durations.
An independent 3D reconstruction from the logged EE rotation agrees with
the logged normal-error vectors, and the report endpoints agree with the
raw endpoints within their reporting precision. Exact residuals and source
hashes are in `measurement_audit.json` and `completed_trials.json`.
All original 23 main-contact setting summaries are unchanged.

The synchronized plot has 13 positions: -100, -90, -80, -40, -20, -10, 0,
10, 20, 40, 80, 90 and 100 mm. The thesis Case-D source and presentation's
Effect of CoC position figure use identical means and error bars. The original
representative wrench traces at -40, 0 and +40 mm remain unchanged.
The plot, summaries, editable sources and raw logs can be regenerated from
this repository without the robot or either document checkout.
