# Redrawing the thesis figures

Every generated plot the thesis includes is written by a script in this
directory, from data tracked in this repository. A clone is enough: nothing
here needs the lab machine or the robot.

## Which figure comes from where

The thesis carries these figures under different names than the scripts write,
because they were renamed by hand after they were generated. Searching the
thesis for a file name will not find it here without this table. Each row was
checked by regenerating the figure and comparing it against the copy the thesis
ships.

| Name in the thesis | Written here as | Script | Reads |
|---|---|---|---|
| `MAIN_B_KR.pdf` | `MAIN_A_KR.pdf` | `make_coc_figures.py` | `derived/metrics.csv` |
| `MAIN_C_KP.pdf` | `MAIN_B_KP.pdf` | `make_coc_figures.py` | `derived/metrics.csv` |
| `MAIN_H_direction.pdf` | `MAIN_C_direction.pdf` | `make_coc_figures.py` | `derived/metrics.csv` |
| `MAIN_D_sign.pdf` | `MAIN_E_sign.pdf` | `make_coc_figures.py` | `derived/metrics.csv` |
| `MAIN_E_frame.pdf` | `MAIN_F_frame.pdf` | `make_coc_figures.py` | `derived/metrics.csv` |
| `MAIN_D_wrench.pdf` | `MAIN_E_wrench.pdf` | `plot_coc_case.py` | `P2_t1_pos_{m040,p000,p040}/r01` |
| `MAIN_D_diagnostics.pdf` | `MAIN_E_diagnostics.pdf` | `plot_setup_diagnostics.py` | `P2_t1_pos_{m040,p040}/r01` |
| `MAIN_DQ_metric_comparison.pdf` | same name | `compare_angle_metrics.py` | `V_best_check/r02`, `derived/metrics.csv` |
| `MAIN_NS_nullspace_automatic.pdf` | same name | `make_nullspace_figure.py` | the twelve `MAIN_NS7`/`MAIN_NS8` runs |

`make_coc_figures.py` also writes `MAIN_D_contact.pdf`, `MAIN_G_toolaxis.pdf`
and `MAIN_H_magnitude.pdf`, and `plot_angle_descent.py` writes
`MAIN_DQ_descent.pdf` from `S1_none_t1_10deg/r01` and `S5_normal_p090/r01`.
All four are in `figures/` and none is included by a chapter any more. They are
kept, and their data with them, so that every generated file in `figures/` has
a generator that still runs.

`figures/MAIN_D_trace.png` is the one exception: no script here writes it and
no chapter includes it.

The four Chapter 5 plots drawn in `pgfplots` have no generator here. They are
`.tex` sources in the thesis repository and are drawn from means already
tabulated in the text.

## Running them

    python3 analysis/make_coc_figures.py
    python3 analysis/plot_setup_diagnostics.py
    python3 analysis/compare_angle_metrics.py
    python3 analysis/plot_angle_descent.py
    python3 analysis/make_nullspace_figure.py
    python3 analysis/plot_coc_case.py \
        "P2_t1_pos_m040/r01=centre -40 mm" \
        "P2_t1_pos_p000/r01=centre 0 mm" \
        "P2_t1_pos_p040/r01=centre +40 mm" \
        --axis t1 --out MAIN_E_wrench

They write into `figures/` unless given `--out-dir`. `plot_coc_case.py` is the
only one that takes its trials on the command line; the three above are the
ones the reported figure uses. The others carry their trials as defaults.

`figure_style.py` holds the drawing conventions and is imported by the rest.
It is not run on its own.

## What data is here, and what is not

Trial archives under `experiments/results/` keep their set-up report, effective
parameters and calibration for every trial. The raw 1 kHz logs are 3.4 GB
across the campaign and are **not** tracked, with one exception: the trials a
figure is drawn from, listed in the table above, carry their full logs. That is
what makes a clone enough to redraw.

The consequence is that `extract_metrics.py` cannot run from a clone. It reads
every archived log to rebuild `experiments/derived/metrics.csv`, and most of
those logs are only on the lab machine. `metrics.csv` is therefore tracked as
data rather than treated as a build product, together with
`derived/MAIN_NS_automatic_summary.csv`. Between them they hold the numbers the
thesis quotes.

If a figure is changed to read a trial not in the table, add that trial's logs
to the exceptions in `.gitignore` in the same commit. Otherwise the figure will
redraw on the machine that happens to hold the archive and nowhere else.
