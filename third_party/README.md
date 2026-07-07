# third_party

This directory is reserved for large external projects that should not be
committed directly into this repository.

PX4 can be downloaded here with:

```bash
./tools/setup_px4.sh
```

The default target path is:

```text
third_party/PX4-Autopilot
```

That directory is ignored by Git. Keep local PX4 builds there, and pass the
path with `PX4_DIR` or `px4_dir:=...` when launching the simulation.
