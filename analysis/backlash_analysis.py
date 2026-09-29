"""
Backlash extraction from dual single-turn encoder readouts (inner = output side,
after the reduction; outer = motor side).

    python3 backlash_analysis.py            # uses the ratio measured from the data
    python3 backlash_analysis.py 25         # forces a given gear ratio for the matching

Pipeline per file
  1. Multi-turn reconstruction. Both encoders report one turn in [0, 360) deg. A
     jump of more than 180 deg between two consecutive samples is a wrap; a turn
     counter is incremented / decremented and 360 deg * turns is added.
  2. Gear ratio. Estimated from the speed columns as median |v_outer / v_inner|
     while both move, cross-checked with the slope of unwrapped outer vs inner
     position, and snapped to the nearest catalogue value {16, 20, 24, 25}.
     The position slope is the ratio used for matching (see report).
  3. Matching. Outer position referred to the output side: outer_u / ratio, both
     zeroed at the first sample.
  4. Discrepancy d = inner_u - outer_u / ratio, in output-side arcmin.
  5. Backlash. While moving, d is modelled as
         d = TE(position) + (B/2) * sign(v) + c * v
     where TE is a piecewise-constant transmission-error term (3 deg bins of
     output position, cancels position-dependent gear error), B is the backlash
     (the jump of d between the two directions of motion), and c * v absorbs the
     read-timing offset between the two encoders (a delay of tau shows up as
     -tau * v). Solved by linear least squares over samples with 4..40 deg/s.
     Transition samples near zero speed (backlash being traversed) are excluded.
Outputs
  backlash_summary.csv   one row per file
  backlash_report.png    per-file backlash bar chart and pooled hysteresis curve
  hysteresis_<file>.png  per-file discrepancy vs speed (branch view)
"""
import glob, os, sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ALLOWED_RATIOS = [16, 20, 24, 25]
DT = 0.006          # s; sample period implied by dpos/speed in the files
V_LO, V_HI = 4.0, 40.0   # deg/s output side, speed window used for the fit
BIN_DEG = 3.0       # output-position bin width for the transmission-error term
EXCLUDE = {          # files not used for the pooled result, with reason
    'DPT-readout-data-20260911-143123': 'inner encoder disturbance: +-2.5 deg oscillation near Nyquist',
    'DPT-readout-data-20260911-143855': 'inner encoder disturbance: +-2.5 deg oscillation near Nyquist',
    'DPT-readout-data-20260911-142643': 'byte-identical duplicate of ...-142632',
}
BLUE, ORANGE, AQUA = '#2a78d6', '#eb6834', '#1baf7a'
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e5e4e0'

def unwrap_single_turn(pos_deg):
    d = np.diff(pos_deg)
    turns = np.concatenate([[0], np.cumsum(np.where(d < -180, 1, np.where(d > 180, -1, 0)))])
    return pos_deg + 360.0 * turns

def load(path):
    df = pd.read_csv(path)
    return (df['Inner Single'].to_numpy(float), df['Outer Single'].to_numpy(float),
            df['Inner Speed'].to_numpy(float), df['Outer Speed'].to_numpy(float))

def gear_ratio(vi, vo, inn_u, out_u):
    m = (np.abs(vi) > 1.0) & (np.abs(vo) > 1.0) & (np.abs(vi) < 80)
    r_speed = float(np.median(np.abs(vo[m]) / np.abs(vi[m]))) if m.sum() > 20 else np.nan
    r_pos = float(np.polyfit(inn_u, out_u, 1)[0])
    r_nom = min(ALLOWED_RATIOS, key=lambda r: abs(r - r_speed))
    return r_speed, r_pos, r_nom

def fit_backlash(inn_u, d_arcmin, v):
    m = (np.abs(v) > V_LO) & (np.abs(v) < V_HI)
    p, dd, vv = inn_u[m], d_arcmin[m], v[m]
    b = np.floor((p - p.min()) / BIN_DEG).astype(int); nb = b.max() + 1
    X = np.zeros((m.sum(), nb + 2)); X[np.arange(m.sum()), b] = 1; X[:, nb] = np.sign(vv); X[:, nb + 1] = vv
    coef = np.linalg.lstsq(X, dd, rcond=None)[0]
    resid = dd - X @ coef
    chunks = []
    for ch in np.array_split(np.arange(m.sum()), 4):       # spread over four time chunks
        keep = X[ch].any(0); c2 = np.linalg.lstsq(X[ch][:, keep], dd[ch], rcond=None)[0]
        chunks.append(2 * c2[np.where(keep)[0].tolist().index(nb)])
    te = coef[:nb][X[:, :nb].any(0)]
    return dict(backlash_arcmin=2 * coef[nb], backlash_chunk_std=float(np.std(chunks)),
                read_delay_ms=-coef[nb + 1] / 60 * 1000, te_ptp_arcmin=float(np.ptp(te)),
                resid_rms_arcmin=float(resid.std()), n_fit=int(m.sum()), mask=m)

def analyse(path, ratio_override=None):
    inn, out, vi, vo = load(path)
    inn_u, out_u = unwrap_single_turn(inn), unwrap_single_turn(out)
    r_speed, r_pos, r_nom = gear_ratio(vi, vo, inn_u, out_u)
    ratio = ratio_override or r_pos
    d = ((inn_u - inn_u[0]) - (out_u - out_u[0]) / ratio) * 60      # output-side arcmin
    v = vo / ratio                                                    # output-side deg/s
    fit = fit_backlash(inn_u, d, v)
    stem = os.path.basename(path).split(' ')[0]
    s = dict(file=os.path.basename(path), samples=len(inn),
             inner_wraps=int(np.sum(np.abs(np.diff(inn)) > 180)), outer_wraps=int(np.sum(np.abs(np.diff(out)) > 180)),
             inner_travel_deg=float(np.ptp(inn_u)), ratio_from_speed=r_speed, ratio_from_position=r_pos,
             ratio_nominal=r_nom, ratio_used=ratio, **{k: v_ for k, v_ in fit.items() if k != 'mask'},
             backlash_output_deg=fit['backlash_arcmin'] / 60, backlash_motor_deg=fit['backlash_arcmin'] / 60 * ratio,
             excluded=EXCLUDE.get(stem, ''))
    return s, dict(inn_u=inn_u, d=d, v=v, mask=fit['mask'])

def style(ax):
    for sp in ('top', 'right'): ax.spines[sp].set_visible(False)
    for sp in ('left', 'bottom'): ax.spines[sp].set_color(GRID)
    ax.grid(axis='y', color=GRID, lw=0.8); ax.tick_params(colors=INK2, labelsize=9)

def branch_curve(v, d, mask):
    bins = np.arange(-V_HI, V_HI + 0.1, 2.0); xs, ys = [], []
    for a, b in zip(bins[:-1], bins[1:]):
        m = mask & (v >= a) & (v < b)
        if m.sum() > 30: xs.append(0.5 * (a + b)); ys.append(np.median(d[m]))
    return np.array(xs), np.array(ys)

def plot_file(path, x, s, folder):
    fig, ax = plt.subplots(figsize=(7, 4)); style(ax)
    xs, ys = branch_curve(x['v'], x['d'] - np.median(x['d'][x['mask']]), x['mask'])
    ax.plot(xs[xs < 0], ys[xs < 0], color=BLUE, lw=2, marker='o', ms=4, label='moving negative')
    ax.plot(xs[xs > 0], ys[xs > 0], color=ORANGE, lw=2, marker='o', ms=4, label='moving positive')
    ax.set_xlabel('output speed, deg/s', color=INK2); ax.set_ylabel('inner minus outer/ratio, arcmin (centred)', color=INK2)
    ax.set_title(f"{s['file'][:32]}   backlash {s['backlash_arcmin']:.1f} arcmin", color=INK, fontsize=11, loc='left')
    ax.legend(frameon=False, fontsize=9); fig.tight_layout()
    fig.savefig(os.path.join(folder, 'hysteresis_' + s['file'].split(' ')[0] + '.png'), dpi=110); plt.close(fig)

def plot_report(S, pooled, folder):
    good = S[S.excluded == '']
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4.8)); style(a1); style(a2)
    lab = [f.split(' ')[0].replace('DPT-readout-data-2026','DPT ').replace('readout-data-2026','') for f in good.file]
    a1.bar(range(len(good)), good.backlash_arcmin, color=BLUE, width=0.6)
    a1.errorbar(range(len(good)), good.backlash_arcmin, yerr=good.backlash_chunk_std, fmt='none', ecolor=INK2, lw=1, capsize=2)
    a1.axhline(good.backlash_arcmin.median(), color=ORANGE, lw=1.5, ls='--')
    a1.text(-0.4, good.backlash_arcmin.max() + 2.5, f'median {good.backlash_arcmin.median():.1f} arcmin (dashed)', color=ORANGE, ha='left', fontsize=9)
    a1.set_xticks(range(len(good))); a1.set_xticklabels(lab, rotation=60, ha='right', fontsize=8)
    a1.set_ylabel('backlash, output-side arcmin', color=INK2); a1.set_title('Backlash per recording (16 clean files)', loc='left', color=INK, fontsize=11)
    xs, ys = pooled
    a2.plot(xs[xs < 0], ys[xs < 0], color=BLUE, lw=2, marker='o', ms=4, label='moving negative')
    a2.plot(xs[xs > 0], ys[xs > 0], color=ORANGE, lw=2, marker='o', ms=4, label='moving positive')
    a2.set_xlabel('output speed, deg/s', color=INK2); a2.set_ylabel('inner minus outer/ratio, arcmin (per-file centred)', color=INK2)
    a2.set_title('Pooled discrepancy vs speed: the step at zero is the backlash', loc='left', color=INK, fontsize=11)
    a2.legend(frameon=False, fontsize=9); fig.tight_layout(); fig.savefig(os.path.join(folder, 'backlash_report.png'), dpi=120); plt.close(fig)

if __name__ == '__main__':
    folder = os.path.dirname(os.path.abspath(__file__))
    files = sorted(f for f in glob.glob(os.path.join(folder, '*.csv')) if 'readout-data' in os.path.basename(f))
    override = float(sys.argv[1]) if len(sys.argv) > 1 else None
    rows, pool_v, pool_d, pool_m = [], [], [], []
    for f in files:
        s, x = analyse(f, override); rows.append(s); plot_file(f, x, s, folder)
        if not s['excluded']:
            pool_v.append(x['v']); pool_d.append(x['d'] - np.median(x['d'][x['mask']])); pool_m.append(x['mask'])
    S = pd.DataFrame(rows); S.to_csv(os.path.join(folder, 'backlash_summary.csv'), index=False)
    pooled = branch_curve(np.concatenate(pool_v), np.concatenate(pool_d), np.concatenate(pool_m))
    plot_report(S, pooled, folder)
    pd.set_option('display.width', 250); pd.set_option('display.max_columns', 30)
    cols = ['file', 'inner_wraps', 'outer_wraps', 'inner_travel_deg', 'ratio_from_speed', 'ratio_from_position', 'ratio_nominal',
            'backlash_arcmin', 'backlash_chunk_std', 'read_delay_ms', 'te_ptp_arcmin', 'resid_rms_arcmin', 'excluded']
    print(S[cols].round(2).to_string(index=False))
    g = S[S.excluded == '']
    print(f"\nratio used: {'forced ' + str(override) if override else 'measured per file'};  position slope mean {S.ratio_from_position.mean():.3f} "
          f"(min {S.ratio_from_position.min():.3f}, max {S.ratio_from_position.max():.3f}); speed-based median {S.ratio_from_speed.median():.2f}; nearest catalogue value {g.ratio_nominal.mode()[0]}")
    print(f"backlash over {len(g)} clean files: median {g.backlash_arcmin.median():.2f} arcmin, mean {g.backlash_arcmin.mean():.2f}, std {g.backlash_arcmin.std():.2f}, "
          f"range {g.backlash_arcmin.min():.2f}..{g.backlash_arcmin.max():.2f}")
    print(f"  = {g.backlash_arcmin.median()/60:.3f} deg output side = {g.backlash_arcmin.median()/60*g.ratio_used.mean():.2f} deg motor side")
