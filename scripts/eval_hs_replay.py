#!/usr/bin/env python3
"""Score a gait_fsm_replay run against contact markers taken from the logs.

    build/gait_fsm_replay config/motorized_shoe_params.yaml *_log.csv > events.csv
    scripts/eval_hs_replay.py [--ref-swing-min 2.0] events.csv *_log.csv

There is no footswitch, so a "reference landing" is defined from the gyro
alone, independently of either detector's trigger: a swing bout (gyro_z > 2
rad/s for >= 50 ms, Left sign-flipped like the FSM does), the first gyro_z < 0
after it (zero-cross, zc), and a flat foot (|gyro_z| < 0.4 for 100 ms) within
600 ms. Latencies are reported against zc and against two earlier contact
markers found between the swing peak and zc: the first accel sample-to-sample
change >= 8 m/s^2 ("jerk"; note the contact detector triggers on the same
quantity, so that column is only independent for the old detector) and the
first wheel kick (|motor velocity| > 3000 counts/s while the command is 0 and
after the wheel was quiet in swing), which no detector uses.
Pure standard library on purpose: the Pi has no numpy/pandas.
"""
import bisect
import csv
import os
import statistics
import sys
from collections import defaultdict

FEET = ("Left", "Right")
# Swing-bout threshold (rad/s) of the reference landings; --ref-swing-min X.
REF_SWING_MIN = 2.0


def load_log(path):
    """-> {foot: dict(imu=[(t, ax, ay, az, gz)], fsm=[(t, 'HS'|'TO')], motor=[(t, vel, cmd)], fires=[t])}"""
    out = {f: dict(imu=[], fsm=[], motor=[], fires=[]) for f in FEET}
    with open(path) as fh:
        r = csv.reader(fh)
        h = next(r)
        ix = {k: i for i, k in enumerate(h)}
        cols = {}
        for f in FEET:
            p = f.lower()
            cols[f] = dict(cnt=ix["imu_%s_msg_count" % p], ax=ix["imu_%s_ax" % p],
                           ay=ix["imu_%s_ay" % p], az=ix["imu_%s_az" % p],
                           gz=ix["imu_%s_gz" % p], ph=ix["gait_%s_phase" % p],
                           cv=ix["cmd_%s_velocity" % p], ct=ix["cmd_%s_type" % p],
                           mv=ix["motor_%s_velocity" % p])
        last_cnt = {f: "0" for f in FEET}
        last_ph = {f: "Stance" for f in FEET}
        last_cv = {f: 0 for f in FEET}
        last_mv = {f: None for f in FEET}
        for row in r:
            if len(row) < len(h):
                continue
            t = float(row[0])
            for f in FEET:
                c = cols[f]
                if row[c["cnt"]] != last_cnt[f]:
                    last_cnt[f] = row[c["cnt"]]
                    sign = -1.0 if f == "Left" else 1.0
                    out[f]["imu"].append((t, float(row[c["ax"]]), float(row[c["ay"]]),
                                          float(row[c["az"]]), sign * float(row[c["gz"]])))
                if row[c["ph"]] != last_ph[f]:
                    last_ph[f] = row[c["ph"]]
                    out[f]["fsm"].append((t, "HS" if last_ph[f] == "Stance" else "TO"))
                cv = int(row[c["cv"]])
                if cv != last_cv[f]:
                    if cv > 0 and last_cv[f] == 0 and row[c["ct"]] == "2":
                        out[f]["fires"].append(t)
                    last_cv[f] = cv
                mv = row[c["mv"]]
                if mv != last_mv[f]:
                    last_mv[f] = mv
                    out[f]["motor"].append((t, int(mv), cv))
    return out


def reference_landings(imu, motor):
    """-> [dict(zc, jerk, kick)] (times in s; jerk/kick may be None)"""
    refs = []
    mt = [m[0] for m in motor]
    n = len(imu)
    i = 1
    while i < n:
        # swing bout
        run = 0
        while i < n:
            run = run + 1 if imu[i][4] > REF_SWING_MIN else 0
            if run >= 5:
                break
            i += 1
        if i >= n:
            break
        bout = i
        peak = imu[i][4]
        k_peak = i
        # first gz < 0 within 1.5 s of the bout
        zc = None
        while i < n and imu[i][0] - imu[bout][0] < 1.5:
            if imu[i][4] > peak:
                peak, k_peak = imu[i][4], i
            if imu[i][4] < 0:
                zc = i
                break
            i += 1
        if zc is None:
            continue
        # flat foot within 600 ms
        flat = False
        run = 0
        j = zc
        while j < n and imu[j][0] - imu[zc][0] < 0.6:
            run = run + 1 if abs(imu[j][4]) < 0.4 else 0
            if run >= 10:
                flat = True
                break
            j += 1
        if flat:
            jerk_t = None
            for k in range(k_peak + 1, zc + 1):
                a, b = imu[k - 1], imu[k]
                d = ((b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2 + (b[3] - a[3]) ** 2) ** 0.5
                if d >= 8.0 and b[4] < peak - 0.3:
                    jerk_t = b[0]
                    break
            kick_t = None
            quiet = False
            m = bisect.bisect_left(mt, imu[bout][0])
            while m < len(motor) and motor[m][0] <= imu[zc][0] + 0.05:
                if abs(motor[m][1]) < 300:
                    quiet = True
                if quiet and abs(motor[m][1]) > 3000 and motor[m][2] == 0:
                    kick_t = motor[m][0]
                    break
                m += 1
            refs.append(dict(zc=imu[zc][0], jerk=jerk_t, kick=kick_t))
        i = zc + 1
    return refs


def pct(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p * len(v)))]


def summary(v):
    if not v:
        return "n=0"
    return "n=%3d  median %+5.0f  p10 %+5.0f  p90 %+5.0f  min %+5.0f  max %+5.0f ms" % (
        len(v), statistics.median(v), pct(v, 0.1), pct(v, 0.9), min(v), max(v))


def main():
    global REF_SWING_MIN
    args = sys.argv[1:]
    if "--ref-swing-min" in args:
        k = args.index("--ref-swing-min")
        REF_SWING_MIN = float(args[k + 1])
        del args[k:k + 2]
    events_path, log_paths = args[0], args[1:]
    events = defaultdict(list)  # (log, foot, mode) -> [(t_detect, t_event, label)]
    with open(events_path) as fh:
        for row in csv.DictReader(fh):
            events[(row["log"], row["foot"], row["mode"])].append(
                (float(row["t_detect_s"]), float(row["t_event_s"]), row["label"]))

    fidelity = dict(logged=0, matched=0, replayed=0)
    stats = {(f, m): defaultdict(list) for f in FEET for m in ("old", "contact")}
    counts = {(f, m): defaultdict(int) for f in FEET for m in ("old", "contact")}
    slip_rows = []

    for path in log_paths:
        name = os.path.basename(path)
        log = load_log(path)
        for foot in FEET:
            d = log[foot]
            imu = d["imu"]
            ts = [s[0] for s in imu]
            refs = reference_landings(imu, d["motor"])
            zcs = [r["zc"] for r in refs]

            # 1. harness fidelity: old-mode replay vs the transitions in the log
            old = events.get((name, foot, "old"), [])
            # (a RESET out of Swing also shows as a Swing->Stance change in the log)
            old_t = sorted(e[0] for e in old)
            fidelity["replayed"] += len(old_t)
            for t, _ in d["fsm"]:
                fidelity["logged"] += 1
                k = bisect.bisect_left(old_t, t - 0.015)
                if k < len(old_t) and abs(old_t[k] - t) <= 0.015:
                    fidelity["matched"] += 1

            for mode in ("old", "contact"):
                ev = events.get((name, foot, mode), [])
                hs = [e for e in ev if e[2] == "HS"]
                to = [e for e in ev if e[2] == "TO"]
                st, ct = stats[(foot, mode)], counts[(foot, mode)]
                ct["ref_landings"] += len(refs)
                ct["hs_events"] += len(hs)
                hs_t = [e[0] for e in hs]
                claimed = set()
                for r in refs:
                    # the HS that belongs to this landing: detect in [zc-0.40, zc+0.45]
                    k = bisect.bisect_left(hs_t, r["zc"] - 0.40)
                    if k < len(hs_t) and hs_t[k] <= r["zc"] + 0.45:
                        claimed.add(k)
                        st["vs_zc"].append((hs_t[k] - r["zc"]) * 1e3)
                        if r["jerk"] is not None:
                            st["vs_jerk"].append((hs_t[k] - r["jerk"]) * 1e3)
                        if r["kick"] is not None:
                            st["vs_kick"].append((hs_t[k] - r["kick"]) * 1e3)
                        first = r["jerk"] if r["jerk"] is not None else r["zc"]
                        st["vs_contact"].append((hs_t[k] - first) * 1e3)
                    else:
                        ct["missed_landings"] += 1
                for k, t in enumerate(hs_t):
                    if k > 0 and t - hs_t[k - 1] < 0.4:
                        ct["double_hs"] += 1
                    if k in claimed:
                        continue
                    i = bisect.bisect_left(ts, t)
                    post = [s[4] for s in imu[i:i + 25]]
                    pre = [s[4] for s in imu[max(0, i - 45):i]]
                    if post and pre and sum(post) / len(post) > 1.0 and max(pre) < 2.0:
                        ct["hs_at_toe_off"] += 1
                    else:
                        ct["hs_other_unmatched"] += 1
                # TO sanity: a real toe-off is followed by a swing bout within 400 ms
                for e in to:
                    i = bisect.bisect_left(ts, e[0])
                    seg = [s[4] for s in imu[i:i + 40]]
                    run = best = 0
                    for g in seg:
                        run = run + 1 if g > 2.0 else 0
                        best = max(best, run)
                    ct["to_events"] += 1
                    if best < 5:
                        ct["to_without_swing"] += 1
                # stance time the slip node's estimator would see (event stamps)
                merged = sorted([(e[1], e[2]) for e in ev if e[2] in ("HS", "TO")])
                for a, b in zip(merged, merged[1:]):
                    if a[1] == "HS" and b[1] == "TO" and 0.3 <= b[0] - a[0] <= 1.5:
                        st["stance_ms"].append((b[0] - a[0]) * 1e3)

            # 3. the slips that were actually triggered on this foot
            contact_hs = [e[0] for e in events.get((name, foot, "contact"), []) if e[2] == "HS"]
            for ft in d["fires"]:
                k = bisect.bisect_right(zcs, ft + 0.05) - 1
                near = k >= 0 and ft - zcs[k] < 0.45
                if near:
                    r = refs[k]
                    first = r["jerk"] if r["jerk"] is not None else r["zc"]
                    j = bisect.bisect_left(contact_hs, r["zc"] - 0.40)
                    e_t = contact_hs[j] if j < len(contact_hs) and contact_hs[j] <= r["zc"] + 0.45 else None
                    slip_rows.append((name[9:15], foot, ft, "landing", (ft - first) * 1e3,
                                      None if e_t is None else (e_t - first) * 1e3))
                else:
                    j = bisect.bisect_left(contact_hs, ft - 0.2)
                    fires_too = j < len(contact_hs) and contact_hs[j] < ft + 0.2
                    slip_rows.append((name[9:15], foot, ft, "TOE-OFF (inverted)", None,
                                      "contact detector also fires here!" if fires_too else "no contact HS here"))

    print("== harness fidelity (old-mode replay vs FSM transitions in the logs, +-15 ms)")
    print("   logged %d, reproduced %d, replay produced %d" % (
        fidelity["logged"], fidelity["matched"], fidelity["replayed"]))
    for foot in FEET:
        for mode in ("old", "contact"):
            ct, st = counts[(foot, mode)], stats[(foot, mode)]
            print("\n== %s foot, %s detector" % (foot, mode))
            print("   reference landings %d | HS events %d | landings missed %d | HS at toe-off %d | "
                  "other unmatched HS %d | HS pairs < 400 ms apart %d" % (
                      ct["ref_landings"], ct["hs_events"], ct["missed_landings"],
                      ct["hs_at_toe_off"], ct["hs_other_unmatched"], ct["double_hs"]))
            print("   TO events %d, of which not followed by a swing %d" % (
                ct["to_events"], ct["to_without_swing"]))
            print("   HS detect - first contact (jerk, else zc): " + summary(st["vs_contact"]))
            print("   HS detect - gyro zero-cross            : " + summary(st["vs_zc"]))
            print("   HS detect - wheel kick (independent)   : " + summary(st["vs_kick"]))
            print("   stance seen by the estimator (TO - HS stamps): " + summary(st["stance_ms"]))
    print("\n== triggered AfterHS slips: logged fire vs where the contact detector fires")
    print("   log    foot  t_fire   fired at            fire-contact  contact-det - contact (ms)")
    for r in slip_rows:
        fmt = lambda v: "%+.0f" % v if isinstance(v, float) else str(v)
        print("   %s %-5s %7.2f  %-19s %-13s %s" % (r[0], r[1], r[2], r[3], fmt(r[4]), fmt(r[5])))


if __name__ == "__main__":
    main()
