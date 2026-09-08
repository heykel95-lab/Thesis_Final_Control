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
