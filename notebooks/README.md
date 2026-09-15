# notebooks — analysis and write-ups

These are plain `.py` files in **percent format**, using `# %%` cell markers.
We never commit `.ipynb` files.

Percent-format files run as ordinary scripts with no Jupyter installed. They
also run cell by cell, with inline plots, in VS Code, PyCharm, and Jupyter. And
they diff like code, because they are code. An `.ipynb` file is JSON with
outputs embedded: its diffs cannot be reviewed, and every run creates a merge
conflict.

```python
# %% [markdown]
# # Joint 3 gravity sweep
# What the residual torque looks like across the range.

# %%
import numpy as np, matplotlib.pyplot as plt
...
```

If you want a browser notebook, pair one locally with `jupytext --sync`. The
`.ipynb` stays ignored by git.

**Safety note.** A notebook cell that holds a live arm handle is exactly the
hazard [ADR-0005](../docs/adr/0005-safe-state-and-stop-architecture.md) exists
for. Restarting the kernel drops the heartbeat; the real-time core ramps down
to a compliant hold and stays powered. Do not disable the heartbeat to make a
cell more convenient.
