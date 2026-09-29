/**
 * ============================================================================
 *  Gearbox backlash measurement
 *  — motor-side position profile, shaft-side PASSIVE observer, SDO sampling
 * ============================================================================
 *
 * WHAT THIS SCRIPT DOES
 * ---------------------
 * It drives the motor through a slow, triangular (zig-zag) position profile and
 * records BOTH encoders while it moves, so that the recorded CSV can be fed
 * straight into Chencheng's `backlash_analysis.py` (bundled under ./analysis).
 * This script does NOT compute the backlash itself — all of the backlash logic
 * lives in that Python script. This script's only job is to command a clean
 * motion and produce a faithful recording.
 *
 * CONTROL ARCHITECTURE (important — differs from the dual-loop example)
 * --------------------------------------------------------------------
 *   - The position control loop runs on the MOTOR-side (internal) encoder.
 *     The drive follows the commanded profile using the motor encoder as
 *     feedback, so the commanded motion is clean and the controller never
 *     "fights" the gearbox play.
 *   - The OUTPUT-shaft (external) encoder is a PASSIVE OBSERVER: it is read but
 *     is NOT part of any control loop (no dual-loop cascade). Because nothing is
 *     trying to servo it, the lost motion across the gearbox shows up in it
 *     directly — which is exactly what we want to measure.
 *
 * You must set this up in OBLAC Drives BEFORE running (single-loop control on
 * the motor encoder, external encoder configured as a feedback/monitoring
 * sensor only). See PASSIVE_OBSERVER_BACKLASH.md for the full setup.
 *
 * WHY SDO, NOT PDO/MONITORING
 * ---------------------------
 * Each sample is a single batched SDO read (`uploadMany`) of the device
 * timestamp plus both encoder positions. All three values come back from one
 * request, so the two encoder readings in a row are coherent (read essentially
 * together) and each row carries the device's own timestamp (0x20F0). The
 * sampling rate is whatever the SDO round-trips allow (typically ~50–150 Hz),
 * which is plenty: Chencheng's fit measures the step between the two direction
 * branches, and its `c*v` term explicitly absorbs any small read-timing offset.
 *
 * WHICH ENCODER IS WHICH (motor vs output)
 * ----------------------------------------
 * Before the sweep, the script makes one small, slow move and watches how far
 * each encoder travels. In a reduction gearbox the MOTOR turns many more
 * revolutions than the OUTPUT shaft, so the encoder with the larger travel is
 * the motor and the other is the output — the "output speed is reduced" rule.
 * The measured ratio is cross-checked against 0x6091. The CSV columns are then
 * assigned accordingly (Chencheng expects inner = output side, outer = motor
 * side), so they are correct even if the two OD objects are wired the other way.
 *
 * !!  SAFETY  !!  This script ENABLES the drive and MOVES the motor (and, with
 * --offset-detection, spins it freely first). Before running: make sure the
 * actuator can rotate freely through the whole range you configured, nobody is
 * near moving parts, and you can reach an E-stop. Start with a small
 * --sweep-total and a low --velocity. Ctrl-C performs a quick stop.
 *
 * HOW TO RUN
 *   npx ts-node ./src/start-backlash-measurement-passive-observer.ts --device-ref=0
 *   npx ts-node ./src/start-backlash-measurement-passive-observer.ts -d 0 \
 *       --sweep-total 90 --step 30 --velocity 8 --accel 40 --offset-detection
 * (see PASSIVE_OBSERVER_BACKLASH.md for the full physical setup guide)
 * ============================================================================
 */

import { writeFileSync, mkdirSync, existsSync } from "fs";
import { join } from "path";
import { spawnSync } from "child_process";
import { program } from "commander";
import { Cia402State, makeParameterId, resolveAfter } from "motion-master-client";
import { client, logStatus } from "./init-client";

// ---------------------------------------------------------------------------
//  CLI options  (init-client already registers -d/--device-ref, -t, -m)
// ---------------------------------------------------------------------------
const num = (v: string) => Number(v);
program
  .option("--sweep-total <deg>", "output-shaft angle to cover, degrees", num, 360)
  .option("--step <deg>", "reversal spacing on the output shaft, degrees", num, 30)
  .option("--velocity <deg_s>", "output-shaft speed during ramps, deg/s (low = cleaner)", num, 8)
  .option("--accel <deg_s2>", "output-shaft accel/decel, deg/s^2 (LOW: high inertia)", num, 40)
  .option("--dwell <ms>", "pause at every reversal, ms", num, 400)
  .option("--period <ms>", "min SDO sampling period, ms (0 = as fast as round-trips)", num, 0)
  .option("--offset-detection", "run offset detection (spins the motor!) before measuring", false)
  .option("--analyze", "run analysis/backlash_analysis.py after recording", false)
  .option("--out-dir <path>", "directory for the recorded CSV", "analysis")
  .option(
    "--motor-counts-per-rev <n>",
    "counts/rev of the MOTOR (control) encoder — VERIFY (0x2110:03/0x2112:03)",
    num,
    524288, // 2^19, the common SOMANET single-turn resolution
  )
  .option("--shaft-counts-per-rev <n>", "counts/rev of the OUTPUT (observer) encoder — VERIFY", num, 524288);
program.parse();

const opts = program.opts();
const deviceRef: number = opts.deviceRef ?? 0;

const SWEEP_TOTAL_DEG: number = opts.sweepTotal;
const SWEEP_STEP_DEG: number = opts.step;
const SWEEP_VELOCITY_DEG_S: number = opts.velocity;
const SWEEP_ACCEL_DEG_S2: number = opts.accel;
const REVERSAL_DWELL_MS: number = opts.dwell;
const SAMPLE_MIN_PERIOD_MS: number = opts.period;
const RUN_OFFSET_DETECTION: boolean = !!opts.offsetDetection;
const RUN_ANALYSIS: boolean = !!opts.analyze;
const OUT_DIR: string = opts.outDir;
const MOTOR_COUNTS_PER_REV: number = opts.motorCountsPerRev;
const SHAFT_COUNTS_PER_REV: number = opts.shaftCountsPerRev;

// ---------------------------------------------------------------------------
//  Object dictionary entries
// ---------------------------------------------------------------------------
//  In the requested single-loop-with-observer configuration:
//    0x6064:00  "Position actual value"    -> the CONTROL encoder = MOTOR shaft
//    0x230A:00  "Secondary position value" -> the OBSERVER encoder = OUTPUT shaft
//  The identification move below verifies this and swaps the CSV column mapping
//  automatically if your drive reports them the other way round.
const CONTROL_POSITION = { index: 0x6064, subindex: 0x00 }; // control-loop feedback encoder
const OBSERVER_POSITION = { index: 0x230a, subindex: 0x00 }; // passive observer encoder
const OD = {
  TIMESTAMP: 0x20f0,
  MODES_OF_OPERATION: 0x6060,
  POSITION_WINDOW: 0x6067,
  TARGET_POSITION: 0x607a,
  PROFILE_VELOCITY: 0x6081,
  PROFILE_ACCELERATION: 0x6083,
  PROFILE_DECELERATION: 0x6084,
  GEAR_RATIO: 0x6091, // :1 motor revolutions, :2 shaft revolutions
  ENCODER1_CONFIG: 0x2110, // :3 resolution (counts/rev)
  ENCODER2_CONFIG: 0x2112, // :3 resolution (counts/rev)
};
const MODE_PROFILE_POSITION = 1;

// Batched SDO read layout for each sample (order must match `sampleIds`).
const S_TS = 0; // device timestamp
const S_CTRL = 1; // control encoder (expected: motor)
const S_OBS = 2; // observer encoder (expected: output)
const sampleIds: [number, number, number][] = [
  [deviceRef, OD.TIMESTAMP, 0x00],
  [deviceRef, CONTROL_POSITION.index, CONTROL_POSITION.subindex],
  [deviceRef, OBSERVER_POSITION.index, OBSERVER_POSITION.subindex],
];

// ---------------------------------------------------------------------------
//  Small helpers
// ---------------------------------------------------------------------------
const nowSec = () => Number(process.hrtime.bigint()) / 1e9;
const norm360 = (deg: number) => ((deg % 360) + 360) % 360;
const toNum = (v: unknown) => Number(v as any);

/** Read one object, returning null instead of throwing if it is unavailable. */
async function tryUpload(index: number, subindex: number): Promise<number | null> {
  try {
    return await client.request.upload<number>(deviceRef, index, subindex);
  } catch {
    return null;
  }
}

/** Phase-unwrap single-turn degrees into a continuous curve (for speed calc). */
function unwrapDeg(deg: number[]): number[] {
  const out: number[] = [];
  let offset = 0;
  for (let i = 0; i < deg.length; i++) {
    if (i > 0) {
      const d = deg[i] - deg[i - 1];
      if (d < -180) offset += 360;
      else if (d > 180) offset -= 360;
    }
    out.push(deg[i] + offset);
  }
  return out;
}

// ---------------------------------------------------------------------------
//  Motion profile: the "two forward, one back" zig-zag target list
// ---------------------------------------------------------------------------
/**
 * Output-shaft target angles (deg, relative to start). Up by 2*step then down by
 * 1*step nets +step per cycle, so the motion walks around `SWEEP_TOTAL_DEG` while
 * planting a clean reversal on every `step` grid line (30, 60, 90 ... 360) from
 * BOTH directions of travel.
 */
function buildZigZagTargetsDeg(): number[] {
  const step = SWEEP_STEP_DEG;
  const cycles = Math.max(1, Math.round(SWEEP_TOTAL_DEG / step));
  const targets: number[] = [];
  let pos = 0;
  for (let c = 0; c < cycles; c++) {
    pos += 2 * step;
    targets.push(pos);
    pos -= step;
    targets.push(pos);
  }
  return targets;
}

// ---------------------------------------------------------------------------
//  Sampling loop (concurrent SDO polling)
// ---------------------------------------------------------------------------
interface Sample {
  host: number; // host monotonic seconds
  dev: number; // device timestamp (0x20F0), raw units
  ctrl: number; // control-encoder counts
  obs: number; // observer-encoder counts
}
const samples: Sample[] = [];
let sampling = false;

async function sampleLoop(): Promise<void> {
  while (sampling) {
    const t = nowSec();
    try {
      const vals = await client.request.uploadMany(sampleIds, false, 2000);
      samples.push({ host: nowSec(), dev: toNum(vals[S_TS]), ctrl: toNum(vals[S_CTRL]), obs: toNum(vals[S_OBS]) });
    } catch {
      /* transient read failure — keep sampling */
    }
    if (SAMPLE_MIN_PERIOD_MS > 0) {
      const elapsedMs = (nowSec() - t) * 1000;
      if (elapsedMs < SAMPLE_MIN_PERIOD_MS) await resolveAfter(SAMPLE_MIN_PERIOD_MS - elapsedMs);
    }
  }
}

// ---------------------------------------------------------------------------
//  CSV assembly in Chencheng's format
// ---------------------------------------------------------------------------
//  Chencheng's loader reads, by column name:
//    'Inner Single' = OUTPUT-side single-turn position, deg  (after reduction)
//    'Outer Single' = MOTOR-side single-turn position, deg
//    'Inner Speed'  = OUTPUT-side speed, deg/s
//    'Outer Speed'  = MOTOR-side speed, deg/s
//  Extra columns are ignored by pandas, so we keep raw data for debugging.
function buildCsv(motorIsCtrl: boolean): string {
  // Map the two recorded encoders onto motor/output roles + their resolutions.
  const motorCounts = samples.map((s) => (motorIsCtrl ? s.ctrl : s.obs));
  const outputCounts = samples.map((s) => (motorIsCtrl ? s.obs : s.ctrl));
  const motorRes = motorIsCtrl ? MOTOR_COUNTS_PER_REV : SHAFT_COUNTS_PER_REV;
  const outputRes = motorIsCtrl ? SHAFT_COUNTS_PER_REV : MOTOR_COUNTS_PER_REV;

  // Single-turn degrees (what Chencheng calls "Single").
  const motorSingle = motorCounts.map((c) => norm360((c / motorRes) * 360));
  const outputSingle = outputCounts.map((c) => norm360((c / outputRes) * 360));

  // Time base for speed: prefer the device timestamp; scale it to seconds by
  // comparing its total span to the host clock. Fall back to the host clock.
  const n = samples.length;
  const devSpan = samples[n - 1].dev - samples[0].dev;
  const hostSpan = samples[n - 1].host - samples[0].host;
  const devPerSec = devSpan !== 0 && hostSpan > 0 ? devSpan / hostSpan : NaN;
  const useDev = Number.isFinite(devPerSec) && devPerSec > 0;
  const tSec = samples.map((s) => (useDev ? s.dev / devPerSec : s.host));

  // Speeds from the derivative of the UNWRAPPED single-turn curves. Use CENTERED
  // differences so each speed sample lines up in time with its position sample —
  // a backward difference would sit half a sample (dt/2) behind the position and
  // show up in Chencheng's fit as a spurious read delay that biases the backlash.
  const speed = (single: number[]): number[] => {
    const u = unwrapDeg(single);
    const v: number[] = new Array(n).fill(0);
    for (let i = 1; i < n - 1; i++) {
      const dt = tSec[i + 1] - tSec[i - 1];
      v[i] = dt > 0 ? (u[i + 1] - u[i - 1]) / dt : 0;
    }
    if (n > 2) {
      v[0] = v[1];
      v[n - 1] = v[n - 2];
    }
    return v;
  };
  const motorSpeed = speed(motorSingle);
  const outputSpeed = speed(outputSingle);

  const header = "Inner Single,Outer Single,Inner Speed,Outer Speed,dt_s,device_ts,motor_counts,output_counts";
  const rows = samples.map((s, i) =>
    [
      outputSingle[i].toFixed(6),
      motorSingle[i].toFixed(6),
      outputSpeed[i].toFixed(6),
      motorSpeed[i].toFixed(6),
      (i > 0 ? tSec[i] - tSec[i - 1] : 0).toFixed(6),
      String(s.dev),
      String(motorCounts[i]),
      String(outputCounts[i]),
    ].join(","),
  );
  return [header, ...rows].join("\n") + "\n";
}

// ---------------------------------------------------------------------------
//  Shutdown handling
// ---------------------------------------------------------------------------
let finished = false;

async function safeStop(): Promise<void> {
  sampling = false;
  try {
    await client.request.quickStop(deviceRef);
  } catch {
    /* ignore — shutting down */
  }
}

process.on("SIGINT", async () => {
  console.log("\nSIGINT (Ctrl-C): quick-stopping the drive and closing sockets.");
  await safeStop();
  client.closeSockets();
  process.exit(0);
});

// ---------------------------------------------------------------------------
//  Main sequence
// ---------------------------------------------------------------------------
client
  .whenReady()
  .then(async () => {
    // --- Optional: offset detection (through Motion Master, not OBLAC). Spins!
    if (RUN_OFFSET_DETECTION) {
      console.log("Running offset detection (the motor will spin freely)...");
      // A DeviceRef can be a position, an address, or a serial number; build the
      // right DeviceRefObj so this works however --device-ref was given.
      const props =
        typeof deviceRef === "string"
          ? { deviceSerialNumber: deviceRef }
          : deviceRef > 0xffff
            ? { deviceAddress: deviceRef }
            : { devicePosition: deviceRef };
      await new Promise<void>((resolve, reject) => {
        client.request.startOffsetDetection(props, 120000).subscribe({
          next: (status) => logStatus(status),
          error: reject,
          complete: resolve,
        });
      });
      console.log("Offset detection finished.\n");
    }

    // --- Diagnostics.
    const gearMotorRev = await tryUpload(OD.GEAR_RATIO, 0x01);
    const gearShaftRev = await tryUpload(OD.GEAR_RATIO, 0x02);
    const enc1Res = await tryUpload(OD.ENCODER1_CONFIG, 0x03);
    const enc2Res = await tryUpload(OD.ENCODER2_CONFIG, 0x03);
    const mechGr = gearMotorRev && gearShaftRev && gearShaftRev !== 0 ? gearMotorRev / gearShaftRev : null;

    const startCtrl = await client.request.upload<number>(deviceRef, CONTROL_POSITION.index, CONTROL_POSITION.subindex);
    const startObs = await client.request.upload<number>(deviceRef, OBSERVER_POSITION.index, OBSERVER_POSITION.subindex);

    console.log("--- Device diagnostics (verify these look right) ---------------");
    console.log(`Device ref                     : ${deviceRef}`);
    console.log(`Control-loop encoder object    : ${makeParameterId(CONTROL_POSITION.index, CONTROL_POSITION.subindex)} (expected: MOTOR shaft)`);
    console.log(`Passive-observer encoder object: ${makeParameterId(OBSERVER_POSITION.index, OBSERVER_POSITION.subindex)} (expected: OUTPUT shaft)`);
    console.log(`Gear ratio 0x6091 (1:2)        : ${gearMotorRev ?? "n/a"} : ${gearShaftRev ?? "n/a"}` + (mechGr ? `  = ${mechGr.toFixed(4)} : 1` : ""));
    console.log(`Encoder resolutions 0x2110/12  : ${enc1Res ?? "n/a"} / ${enc2Res ?? "n/a"} counts/rev`);
    console.log(`Configured motor / shaft c/rev : ${MOTOR_COUNTS_PER_REV} / ${SHAFT_COUNTS_PER_REV}`);
    console.log("----------------------------------------------------------------");

    // --- Set up Profile Position Mode. Targets/velocity/accel are in the
    //     CONTROL encoder's counts (the motor, in the requested config). We
    //     convert output-shaft degrees -> motor counts via the gear ratio.
    const grForCommand = mechGr ?? 1;
    if (mechGr === null) {
      console.warn("WARNING: 0x6091 gear ratio unavailable; commanding as if 1:1. Set 0x6091 in OBLAC for correct sweep size.");
    }
    const outDegToCtrlCounts = (deg: number) => Math.round((deg / 360) * grForCommand * MOTOR_COUNTS_PER_REV);
    const positionWindowCounts = Math.max(1, Math.round((0.02 / 360) * grForCommand * MOTOR_COUNTS_PER_REV));

    await client.request.resetTargets(deviceRef);
    await client.request.downloadMany([
      [deviceRef, OD.MODES_OF_OPERATION, 0x00, MODE_PROFILE_POSITION],
      [deviceRef, OD.POSITION_WINDOW, 0x00, positionWindowCounts],
      [deviceRef, OD.PROFILE_VELOCITY, 0x00, outDegToCtrlCounts(SWEEP_VELOCITY_DEG_S)],
      [deviceRef, OD.PROFILE_ACCELERATION, 0x00, outDegToCtrlCounts(SWEEP_ACCEL_DEG_S2)],
      [deviceRef, OD.PROFILE_DECELERATION, 0x00, outDegToCtrlCounts(SWEEP_ACCEL_DEG_S2)],
    ]);

    // --- Enable and settle.
    await client.request.transitionToCia402State(deviceRef, Cia402State.OPERATION_ENABLED);
    await resolveAfter(REVERSAL_DWELL_MS);

    // --- Identification move: which encoder is the motor (larger travel)?
    console.log("Identification move (small, slow) to confirm motor vs output encoder...");
    sampling = true;
    const sampler = sampleLoop();
    const idTargetCounts = startCtrl + outDegToCtrlCounts(Math.min(SWEEP_STEP_DEG, 15));
    await client.request.downloadMany([[deviceRef, OD.TARGET_POSITION, 0x00, idTargetCounts]]);
    await client.request.applySetPoint(deviceRef);
    await client.whenTargetReached(deviceRef);
    await resolveAfter(REVERSAL_DWELL_MS);

    const ctrlTravelRev = Math.abs((toNum(await client.request.upload(deviceRef, CONTROL_POSITION.index, CONTROL_POSITION.subindex)) - startCtrl)) / MOTOR_COUNTS_PER_REV;
    const obsTravelRev = Math.abs((toNum(await client.request.upload(deviceRef, OBSERVER_POSITION.index, OBSERVER_POSITION.subindex)) - startObs)) / SHAFT_COUNTS_PER_REV;
    const motorIsCtrl = ctrlTravelRev >= obsTravelRev; // motor turns more revolutions
    const measuredGr = Math.min(ctrlTravelRev, obsTravelRev) > 0 ? Math.max(ctrlTravelRev, obsTravelRev) / Math.min(ctrlTravelRev, obsTravelRev) : NaN;
    console.log(`  control-encoder travel : ${ctrlTravelRev.toFixed(4)} rev`);
    console.log(`  observer-encoder travel: ${obsTravelRev.toFixed(4)} rev`);
    console.log(`  => MOTOR is the ${motorIsCtrl ? "CONTROL (0x6064)" : "OBSERVER (0x230A)"} encoder; measured ratio ~ ${measuredGr.toFixed(3)} : 1` + (mechGr ? ` (0x6091 says ${mechGr.toFixed(3)})` : ""));
    if (!motorIsCtrl) {
      console.warn("  NOTE: the CONTROL loop appears to run on the OUTPUT encoder, not the motor.");
      console.warn("  That is the opposite of the requested single-loop-on-motor setup. CSV columns");
      console.warn("  are assigned by measured travel so the analysis is still correct, but review your");
      console.warn("  OBLAC encoder configuration if you intended motor-side control.");
    }
    if (mechGr && Number.isFinite(measuredGr) && Math.abs(measuredGr - mechGr) / mechGr > 0.25) {
      console.warn("  WARNING: measured ratio differs >25% from 0x6091 — check *_COUNTS_PER_REV values.");
    }

    // --- Run the zig-zag sweep (sampler keeps running throughout).
    const targetsDeg = buildZigZagTargetsDeg();
    console.log(`\nRunning ${targetsDeg.length} moves covering ~${SWEEP_TOTAL_DEG} deg output, reversing every ${SWEEP_STEP_DEG} deg...`);
    for (let i = 0; i < targetsDeg.length; i++) {
      const targetCounts = startCtrl + outDegToCtrlCounts(targetsDeg[i]);
      await client.request.downloadMany([[deviceRef, OD.TARGET_POSITION, 0x00, targetCounts]]);
      await client.request.applySetPoint(deviceRef);
      await client.whenTargetReached(deviceRef);
      await resolveAfter(REVERSAL_DWELL_MS);
      process.stdout.write(`\r  move ${i + 1}/${targetsDeg.length} -> ${targetsDeg[i].toFixed(0)} deg (samples: ${samples.length})   `);
    }
    process.stdout.write("\n");

    // --- Stop sampling and write the CSV.
    sampling = false;
    await sampler;

    if (samples.length < 20) {
      console.log(`Only ${samples.length} samples collected — not enough to analyse. Check the SDO connection.`);
      finished = true;
      return;
    }

    if (!existsSync(OUT_DIR)) mkdirSync(OUT_DIR, { recursive: true });
    const stamp = new Date().toISOString().replace(/[:.]/g, "-").replace("T", "-").slice(0, 19);
    const csvPath = join(OUT_DIR, `DPT-readout-data-${stamp}.csv`);
    writeFileSync(csvPath, buildCsv(motorIsCtrl));
    console.log(`\nRecorded ${samples.length} samples -> ${csvPath}`);
    console.log(`Analyse with Chencheng's script:\n  cd ${OUT_DIR} && python3 backlash_analysis.py`);

    // --- Optional: run the Python analysis right away (try python3, then python).
    if (RUN_ANALYSIS) {
      console.log("\nRunning analysis/backlash_analysis.py ...");
      let ran = false;
      for (const exe of ["python3", "python"]) {
        const py = spawnSync(exe, ["backlash_analysis.py"], { cwd: OUT_DIR, encoding: "utf8" });
        if (py.error) continue; // interpreter not found — try the next name
        ran = true;
        if (py.status === 0) console.log(py.stdout);
        else console.log(`  ${exe} exited with code ${py.status}.\n${py.stderr ?? ""}`);
        break;
      }
      if (!ran) console.log(`  No Python interpreter found. Run it manually: cd ${OUT_DIR} && python3 backlash_analysis.py (needs numpy/pandas/matplotlib).`);
    }

    finished = true;
  })
  .catch((err) => {
    console.error("\nMeasurement failed:", err);
  })
  .finally(async () => {
    sampling = false;
    if (!finished) await safeStop();
    else {
      try {
        await client.request.quickStop(deviceRef);
      } catch {
        /* ignore */
      }
    }
    client.closeSockets();
  });
