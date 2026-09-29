# Gearbox backlash — motor-side profile, shaft-side passive observer

This guide covers `src/start-backlash-measurement-passive-observer.ts`: it drives
the motor through a slow triangular (zig-zag) position profile and records both
encoders over SDO, then hands the recording to **Chencheng's**
`analysis/backlash_analysis.py`, which does all of the backlash maths.

It replaces the manual "turn the shaft by hand" step: the drive commands the
profile and the analysis is unchanged.

> There is a second, older backlash example in this repo
> (`start-gearbox-backlash-measurement.ts`) that uses **dual-loop** control and
> its own TypeScript analysis. This one is different on purpose — single-loop on
> the motor with the shaft as a passive observer, and Chencheng's Python for the
> analysis. Use this one for the passive-observer method.

> ⚠️ **Safety first.** This script *enables the drive and spins the motor* (and,
> with `--offset-detection`, spins it freely first). Before you run it: make sure
> the actuator can rotate freely through the whole range, nobody is near moving
> parts, and an E-stop is within reach. Start with a small `--sweep-total` and a
> low `--velocity`. `Ctrl-C` triggers a quick stop.

---

## 1. The idea

Backlash is the lost motion across the gearbox: when the motor reverses, its
teeth travel across the gap before re-engaging on the other flank, so the motor
moves a little while the output shaft stays put.

To see that cleanly:

- **Control loop on the motor (internal) encoder.** The drive follows the
  commanded profile using the motor encoder, so the commanded motion is clean and
  the position controller never tries to servo out the play.
- **Output (external) encoder as a passive observer.** It is read but is not in
  any control loop. Since nothing is servoing it, the lost motion appears in it
  directly — exactly what we measure.

The zig-zag profile ("two steps forward, one step back") walks around the whole
output revolution and plants a clean reversal every `--step` degrees
(30, 60, 90 … 360) from both directions, giving a backlash-vs-angle map.

**Why SDO, not PDO/monitoring:** each sample is one batched SDO read
(`uploadMany`) of the device timestamp + both encoder positions, so the two
readings in a row are coherent and each row carries the device's own timestamp
(0x20F0). Rate is whatever the round-trips allow (~50–150 Hz), which is plenty
for Chencheng's fit.

## 2. What you need physically

| Item | Notes |
|------|-------|
| A SOMANET servo drive | SOMANET Node / Circulo, software **5.x**. |
| A motor **with a gearbox** | The unit under test. |
| **Two encoders spanning the gearbox** | **Internal** on the **motor** shaft, **external** on the **output** shaft. Mandatory — backlash is the difference of the two. |
| EtherCAT wiring + PC running Motion Master | via OBLAC Drives. |
| A safe mechanical range | The output shaft must turn freely through the full `--sweep-total` (360° by default). Remove end stops / loads, or reduce the range. |
| Power supply + E-stop | Standard bring-up. |

## 3. What you need in software

1. **OBLAC Drives** installed and connected (it runs the Motion Master server this
   script talks to). <https://doc.synapticon.com/>
2. **Node.js + npm**, then in this repo: `npm install`.
3. **Python 3** with `numpy pandas matplotlib` for the analysis:
   `pip install numpy pandas matplotlib`.
4. **Point the script at Motion Master** — a `.env` file in the repo root:
   ```sh
   MOTION_MASTER_HOSTNAME=192.168.1.112
   ```

## 4. Prepare the drive (in OBLAC Drives) — single loop + observer

Do this once, before running. **This is the key difference from the dual-loop
example.**

1. **Commission the motor** normally (motor + both encoders detected, commutation
   done, drive enables without a fault).
2. **Set single-loop control on the MOTOR encoder:**
   - Motor-shaft (internal) encoder → **Commutation**, **Velocity** *and*
     **Position** (it is the sole control-loop feedback).
   - Output-shaft (external) encoder → configured as a **feedback / secondary
     sensor only** (so it is read and reported on `0x230A`), assigned to **no**
     control function. It must NOT be the position sensor — that would be the
     dual-loop setup we are avoiding.
     > You can offset-detect the motor from here, or let the script do it with
     > `--offset-detection` (Motion Master, not OBLAC).
3. **Set the gear ratio** in **`0x6091`** (`:1` motor revolutions, `:2` shaft
   revolutions). The script reads this to size the sweep and as a cross-check.
4. **Confirm it moves:** command a small move in Profile Position Mode and check
   both encoders respond (motor on `0x6064`, observer on `0x230A`) and no fault.

## 5. Object dictionary entries the script uses

| Object | Meaning | Role here |
|--------|---------|-----------|
| `0x6064:00` | Position actual value | **Control** encoder = **motor** (position feedback) |
| `0x230A:00` | Secondary position value | **Observer** encoder = **output** shaft |
| `0x20F0:00` | Device timestamp | Per-sample timestamp (SDO) |
| `0x6091:01/02` | Gear ratio (motor rev : shaft rev) | Sweep sizing + cross-check |
| `0x2110:03`, `0x2112:03` | Encoder 1 / 2 resolution | Verify `--*-counts-per-rev` |
| `0x607A / 0x6081 / 0x6083 / 0x6084` | Target pos / profile vel / accel / decel | Commanded in **motor** counts |

The identification move (below) confirms which physical encoder is on `0x6064`
vs `0x230A` and assigns the CSV columns correctly even if they are swapped.

## 6. Run it

```console
# First run — small and slow to prove it out:
npx ts-node ./src/start-backlash-measurement-passive-observer.ts -d 0 \
    --sweep-total 90 --step 30 --velocity 8 --accel 40

# Optional: offset-detect through Motion Master first (spins the motor):
npx ts-node ./src/start-backlash-measurement-passive-observer.ts -d 0 --offset-detection

# Full revolution once you are happy, and run the analysis automatically:
npx ts-node ./src/start-backlash-measurement-passive-observer.ts -d 0 \
    --sweep-total 360 --step 30 --velocity 8 --accel 40 --analyze
```

Options (all have sensible defaults):

| Option | Default | What it does |
|--------|---------|--------------|
| `-d, --device-ref` | `0` | Device position / address / serial. `0` = first device. |
| `--sweep-total <deg>` | `360` | Output angle to cover. **Start smaller.** |
| `--step <deg>` | `30` | Reversal spacing (30, 60, 90 …). |
| `--velocity <deg_s>` | `8` | Output speed during ramps. Lower = cleaner + denser sampling. |
| `--accel <deg_s2>` | `40` | Output accel/decel. **Keep low — high inertia.** |
| `--dwell <ms>` | `400` | Pause at each reversal (lets the branches settle). |
| `--period <ms>` | `0` | Min SDO sampling period; `0` = as fast as round-trips. |
| `--offset-detection` | off | Run offset detection (spins!) before measuring. |
| `--analyze` | off | Run `analysis/backlash_analysis.py` after recording. |
| `--out-dir <path>` | `analysis` | Where the CSV is written (next to the Python). |
| `--motor-counts-per-rev <n>` | `524288` | Motor encoder counts/rev — **verify** via `0x2110:03`/`0x2112:03`. |
| `--shaft-counts-per-rev <n>` | `524288` | Output encoder counts/rev — **verify**. |

What happens:

1. (optional) offset detection.
2. Prints **diagnostics** — gear ratio, encoder resolutions, start positions.
3. **Identification move** (small, slow): reports each encoder's travel and which
   one is the motor (the one that turns more revolutions — "output speed is
   reduced"), cross-checked against `0x6091`.
4. Runs the zig-zag sweep while SDO-sampling both encoders.
5. Writes `analysis/DPT-readout-data-<timestamp>.csv`.
6. Tells you the command to analyse it (or runs it with `--analyze`).

`Ctrl-C` at any time → quick stop and exit.

## 7. Analyse

```console
cd analysis
python3 backlash_analysis.py            # gear ratio measured from the data
python3 backlash_analysis.py 25         # force a gear ratio (motor:output)
```

Outputs (in `analysis/`): `backlash_summary.csv`, `backlash_report.png`, and a
`hysteresis_<file>.png` per recording. See `analysis/README.md`. The script pools
**all** `DPT-readout-data-*.csv` files in that folder — delete old ones to start a
fresh batch, or keep them to pool multiple runs.

## 8. Troubleshooting

| Symptom | Likely cause / fix |
|---------|--------------------|
| "control loop appears to run on the OUTPUT encoder" warning | The drive is in dual-loop / output-position mode. Reconfigure to single-loop on the motor (section 4). The CSV is still assigned correctly, but this is not the intended setup. |
| Measured ratio differs >25% from `0x6091` | Wrong `--motor-counts-per-rev` / `--shaft-counts-per-rev`. Set them from `0x2110:03` / `0x2112:03`. |
| Very few samples collected | SDO round-trips slow/blocked — check the Motion Master connection; lower the move count (`--sweep-total`). |
| Backlash values look 360°-ish or jump wildly | Wrong counts-per-rev, or the observer encoder (`0x230A`) is not being reported — confirm it is configured as a secondary sensor in OBLAC. |
| Drive faults on enable | Commission/enable it in OBLAC first; clear faults; confirm it runs in Profile Position Mode. |
| `MOTION_MASTER_HOSTNAME … is not defined` | Create the `.env` file (section 3). |
| Analysis: `ModuleNotFoundError` | `pip install numpy pandas matplotlib`. |

## References

- [Dual Loop Cascaded Position Control — Synapticon](https://doc.synapticon.com/circulo/sw5.0/motion_control/control_loops/dual_loop_cascaded_position_control.html) (what we are *not* using here)
- [0x6064 Position actual value](https://doc.synapticon.com/node/sw5.4/objects_html/6xxx/6064.html)
- [0x6091 Gear ratio](https://doc.synapticon.com/node/sw5.4/objects_html/6xxx/6091.html)
- [SOMANET Object Dictionary (all objects)](https://doc.synapticon.com/node/sw5.4/object_dict/all_objects.html)
