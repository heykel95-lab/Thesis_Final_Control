# Experiment archive

This directory includes the experiment scripts, setups, derived metrics, raw
trial logs, and older campaigns in `results_prior/`, `results_kr300/`, and
`results_kr180_oldnames/`. Python bytecode caches are excluded.

CSV files are stored with Git LFS. Install Git LFS before cloning, then run:

```sh
git lfs install
git lfs pull
```

Run these commands inside an existing clone as well to download the CSV data.
The complete archive requires several gigabytes of disk space and download.

Archived `params_effective/` files preserve the parameters used for each run,
including historical names such as `setup.conf`. Current controller parameters
are in `surface_grinding_controller/params/` at the repository root.

The prepared +/-90 and +/-100 mm t1 CoC extension is documented in
[`coc_extension/README.md`](coc_extension/README.md). Its eight settings still
need measurements before updating the thesis and presentation figures.
