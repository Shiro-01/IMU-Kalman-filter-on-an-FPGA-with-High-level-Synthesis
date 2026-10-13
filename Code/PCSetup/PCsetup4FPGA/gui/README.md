# IMU/EKF real-time dashboard

Replaces `app2.py`'s console printout with a live PySide6 desktop GUI.
Does not modify any file outside `gui/` — `EKF.py`, `data_preparer.py`,
`sample_converter.py`, `UARTFPGA_reader.py`, `frame_types.py` are imported
as-is from the parent directory.

## Stack, and why

| Piece | Choice | Why |
|---|---|---|
| GUI shell | PySide6 (Qt6) | Native widgets, real `QThread`s with the GIL released during blocking serial I/O and most numpy calls — unlike Streamlit's rerun-the-whole-script model, nothing here re-executes on every packet. |
| 3D view | `pyqtgraph.opengl` (`GLViewWidget`/`GLMeshItem`) | Same Qt/GL context as the rest of the app (one widget toolkit, one event loop) instead of pulling in a second 3D engine. Mesh loading is generic via `trimesh`, so any OBJ/STL/glTF file works. |
| 2D plot | `pyqtgraph.PlotWidget` | Built for scientific real-time data: GPU-backed, supports `setDownsampling`/`setClipToView` so a tens-of-thousands-of-points buffer still redraws cheaply at 60 Hz. |

**Why this beats Streamlit here:** Streamlit's execution model reruns the
whole script on each interaction/update, so at 1125 Hz it either falls
behind or you throttle the *acquisition* rate to match the *render* rate —
exactly the failure mode described. Here acquisition and rendering are two
independent loops on two different clocks, connected only by a small
thread-safe hand-off (`state_bus.py`): a single "latest snapshot" slot plus
a time-bounded rolling deque for the divergence history. The acquisition
`QThread` runs `read → convert → prepare → EKF.step()` as fast as packets
arrive; a `QTimer` in the GUI thread redraws at a fixed ~60 Hz by reading
whatever is currently in the slot. A slow GUI frame just means the next
tick reads a newer snapshot — it can never block or slow the filter loop
(drop-oldest by construction, not by an explicit queue policy).

The one tradeoff worth naming: the divergence history is appended to the
`StateBus` deque at the *full acquisition rate* (not decimated), then
pruned by wall-clock time window rather than sample count, and pyqtgraph's
own `setDownsampling`/`setClipToView` handle keeping the *redraw* of that
buffer cheap. This keeps the plotted curve statistically faithful to every
packet (no aliasing from picking every Nth point) at the cost of the deque
holding up to `history_window_s * 1125` floats (e.g. ~34k points for a 30 s
window) — a few hundred KB, not a memory concern.

## Setup

```bash
cd gui
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
python3 gui_app.py
```

The Source selector defaults to **Mock** — no hardware needed. It
generates synthetic MODE_1 packets from a smooth simulated attitude (using
`EKF`'s own kinematic model, so the software filter tracks it closely),
runs them through the *same* unmodified `SampleConverter` /
`DataPreparer` / `EKF` pipeline as real hardware, and paces itself to
1125 Hz. Click **Start**.

To use the real FPGA: switch Source to **Serial**, set the port (defaults
to app2.py's `/dev/tty.usbserial-210183ACB00B1`) and baud rate, then
**Start**. Real-hardware calibration constants (hard/soft-iron, Hamburg
WMM values) are the same ones already in `app2.py`, centralized in
`constants.py`.

## Swapping the 3D mesh

The orientation view has no hardcoded geometry. Click **Load Mesh...** in
the toolbar and pick any `.obj`/`.stl`/`.glb`/`.gltf`/`.ply` file — loading
goes through `trimesh` in `mesh_loader.py` and is completely decoupled
from `orientation_widget.py`'s rendering code. Until you supply one, a
small procedurally-generated placeholder craft (tapered fuselage, delta
wings, tail fin — no external asset, no IP concerns) is used.

Convention expected of a supplied mesh: body frame with the **nose along
+X**, **right side along +Y**, **belly along +Z** (down) — standard
aerospace body axes — roughly centered on the origin. It's rescaled and
recentered automatically regardless of the file's original units.

Almost no downloaded mesh actually matches that convention out of the box
— there's no way to detect "which way is the nose" from geometry alone, so
after loading a file it will very likely look tipped over or sideways at
rest. Fix it with the toolbar's **Mesh nose axis** / **Mesh belly axis**
dropdowns: pick which of the *mesh's own* local axes points toward the
nose and which points toward the belly (down), based on how it looks by
eye. The third axis ("right") is never a free choice — it's derived
automatically as `belly × nose`, so only valid, orthogonal, correctly
-handed combinations are reachable; the Belly dropdown is filtered live to
just the 4 axes perpendicular to whatever Nose axis is selected. This is a
one-time correction baked into the mesh's vertices, independent of the
live attitude rotation applied on top of it. **Reset alignment** returns
to the identity mapping (nose=+X, belly=+Z). Loading a new file resets
this mapping — a different mesh needs its own.

One case this can't fix: a mesh whose geometry is a genuine mirror image
relative to this convention (different handedness, not just a relabeling)
— no rotation/relabeling can correct that; it would need an explicit
mirror + face-winding fix, which isn't implemented since it's an unusual
case for typical downloaded assets.

For a ready-made "small sleek craft" look, search Sketchfab's CC0 category
(filter by license) rather than any franchise-specific asset — a fan
upload of a copyrighted design (e.g. a Star Wars ship) isn't actually
freely licensed just because the uploader didn't restrict the file; the
underlying design is still someone else's IP regardless of who modeled it.

## Notes

- **Magnetic declination**: the toolbar's "Declination (E)" field defaults
  to +4.2° (Hamburg, current WMM). It only affects the True-North pointer
  in the 3D view (`true_heading = magnetic_heading + declination_east`) —
  it is *not* fed into the EKF's own `mN`/`mD` constants, which are
  unrelated (field magnitude, not declination angle). Declination drifts
  slowly; refresh it periodically from NOAA's calculator.
- **Divergence metric**: RMSE across all 6 state components
  `[phi,theta,psi,bx,by,bz]`, with wraparound-safe differencing on the 3
  angles (`acquisition.combined_state_error`). Angles (rad) and gyro biases
  (rad/s) are on very different scales, so this norm is in practice
  dominated by attitude disagreement — a single combined indicator, not a
  calibrated unit.
- **Verification**: the acquisition/EKF pipeline and full GUI lifecycle
  (start/stop, panel updates, plot updates) were exercised headlessly
  (`QT_QPA_PLATFORM=offscreen`) against the mock source. Actual OpenGL
  rendering of the 3D view could not be visually verified in that sandboxed
  environment (no real display) — run it locally to confirm the 3D view
  looks right; everything else was verified end-to-end.
