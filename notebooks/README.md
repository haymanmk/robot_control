# notebooks — analysis and write-ups

Plain `.py` files in **percent format** (`# %%` cell markers), never `.ipynb`.

They run as ordinary scripts with no Jupyter installed, execute cell-by-cell
with inline plots in VS Code, PyCharm, and Jupyter, and diff like code — because
they are code. `.ipynb` is JSON with embedded outputs: unreviewable diffs and a
merge conflict on every execution.

```python
# %% [markdown]
# # Joint 3 gravity sweep
# What the residual torque looks like across the range.

# %%
import numpy as np, matplotlib.pyplot as plt
...
```

If you want a browser notebook, pair one locally with `jupytext --sync`; the
`.ipynb` stays gitignored.

**Safety note:** a notebook cell holding a live arm handle is exactly the hazard
[ADR-0005](../docs/adr/0005-safe-state-and-stop-architecture.md) exists for.
Restarting the kernel drops the heartbeat; the RT core ramps to a compliant hold
and stays powered. Do not disable the heartbeat to make a cell more convenient.
