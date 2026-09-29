# Backlash analysis

`backlash_analysis.py` is **Chencheng's** backlash-extraction script, bundled here
unchanged so the measurement example is self-contained. It is the single source
of truth for the backlash maths — the recorder
(`../src/start-backlash-measurement-passive-observer.ts`) only produces the CSVs
it reads.

## How it fits together

1. The recorder writes one `DPT-readout-data-<timestamp>.csv` into **this folder**
   per run, with the four columns the script expects:
   `Inner Single`, `Outer Single`, `Inner Speed`, `Outer Speed`
   (`Inner` = output shaft / after the reduction, `Outer` = motor side; positions
   in single-turn degrees, speeds in deg/s). Extra columns are ignored.
2. Run the analysis over every recording in this folder:
   ```console
   python3 backlash_analysis.py           # gear ratio measured from the data
   python3 backlash_analysis.py 25        # force a gear ratio (motor:output)
   ```
   Requires `numpy`, `pandas`, `matplotlib` (`pip install numpy pandas matplotlib`).
3. Outputs (written here):
   - `backlash_summary.csv` — one row per recording.
   - `backlash_report.png` — per-file backlash bars + pooled hysteresis curve.
   - `hysteresis_<file>.png` — per-file discrepancy-vs-speed (branch view).

## Notes

- The script pools **all** `*readout-data*.csv` files in this folder, so repeated
  runs accumulate into the summary. Delete old CSVs to start a fresh batch.
- `ALLOWED_RATIOS = [16, 20, 24, 25]` near the top is the gear-ratio catalogue it
  snaps to for a cross-check. The backlash number itself uses the ratio measured
  from the position slope, so a catalogue mismatch does not corrupt the result —
  but edit the list (or pass the ratio as an argument) to match your actuator.
