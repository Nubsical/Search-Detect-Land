"""
Fetch and summarise the FLIGHT CONTROLLER's own dataflash logs (.BIN).

This is the OTHER half of the logging story. `FlightLog.py` records what the Pi
did (commands we sent, what the camera saw). This script pulls what the FC
recorded internally -- the vehicle's own account of the same flight.

    python FetchFcLogs.py list                 # what logs are on the FC
    python FetchFcLogs.py get last             # download the most recent
    python FetchFcLogs.py get 42               # download log id 42
    python FetchFcLogs.py get last --out ./x   # somewhere other than logs/fc/
    python FetchFcLogs.py summarise <file.BIN> # -> readable .txt + .csv

Why `summarise` exists
----------------------
A .BIN is megabytes of packed binary; nobody (human or otherwise) can read it
directly, and it is far too big to paste into a chat. `summarise` reads it
locally and writes two small, shareable files:

  <name>_summary.txt  mode timeline, errors, statustexts, arm/disarm events
  <name>_trace.csv    the numeric columns that matter, downsampled

Those two ARE the thing to share when you want someone to look at a flight.

What to look for (given what we have been chasing)
--------------------------------------------------
The FC's ATT message logs DESIRED vs ACHIEVED attitude:

    DesRoll/Roll, DesPitch/Pitch, DesYaw/Yaw

If our SET_ATTITUDE_TARGET messages are being accepted, DesRoll/DesPitch/DesYaw
follow what the companion asked for. If they sit at zero (or just track the RC
sticks) while our Pi-side log shows commands going out, the FC is discarding
them -- which is precisely the type_mask failure. Cross-reference the Pi's
`cmd_*` columns against these `Des*` columns and the answer is unambiguous.

CTUN answers the altitude question: DCRt (desired climb rate) vs CRt (actual),
plus DAlt/Alt and ThO (throttle out). If DCRt is ~0 through the mode change and
the quad still rose, the step came from the altitude controller taking over,
not from a commanded climb.

Speed warning
-------------
Downloading over MAVLink at 115200 baud is SLOW -- roughly 5-8 KB/s in
practice, so a 10 MB log can take 20-30 minutes. For anything large, pulling
the FC's SD card and copying the file directly is far quicker. Raising the
link's SERIALn_BAUD helps if you download often. `list` shows sizes and an
estimate first so you can decide before committing.

  !!  Do this on the ground, disarmed. It saturates the telemetry link.  !!
"""

import argparse
import math
import os
import sys
import time
from pathlib import Path

from pymavlink import mavutil

# Reuse the connection settings so there is one place to change them.
try:
    import LandOnAprilTag as L
    CONNECTION_STRING = L.CONNECTION_STRING
    BAUD_RATE = L.BAUD_RATE
except Exception:      # importing pulls in cv2/picamera2; fall back if absent
    CONNECTION_STRING = '/dev/ttyAMA0'
    BAUD_RATE = 115200

CHUNK = 90                  # bytes per LOG_DATA message (protocol fixed)
STALL_TIMEOUT_S = 3.0       # no data for this long -> re-request the gaps
MAX_STALLS = 40             # give up after this many fruitless re-requests


def connect():
    print(f"Connecting to {CONNECTION_STRING} @ {BAUD_RATE} ...")
    mav = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    mav.wait_heartbeat()
    print(f"Heartbeat: system {mav.target_system}, "
          f"component {mav.target_component}  (mode {mav.flightmode})")

    armed = mav.motors_armed()
    if armed:
        print("!! VEHICLE IS ARMED. Downloading saturates the telemetry link.")
        print("!! Disarm before continuing.")
    return mav


def list_logs(mav, timeout=10.0):
    """Return [(id, size_bytes, time_utc), ...] sorted by id."""
    mav.mav.log_request_list_send(mav.target_system, mav.target_component,
                                  0, 0xFFFF)
    entries = {}
    expected = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = mav.recv_match(type='LOG_ENTRY', blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.num_logs == 0:
            return []
        expected = msg.num_logs
        entries[msg.id] = (msg.id, msg.size, msg.time_utc)
        if len(entries) >= expected:
            break
        deadline = time.monotonic() + timeout   # progress -> extend
    return [entries[k] for k in sorted(entries)]


def _fmt_size(n):
    for unit in ('B', 'KB', 'MB', 'GB'):
        if n < 1024 or unit == 'GB':
            return f"{n:.0f} {unit}" if unit == 'B' else f"{n:.1f} {unit}"
        n /= 1024.0


def _fmt_utc(t):
    if not t:
        return "(no GPS time)"
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t))


def _eta(size_bytes, baud):
    """Rough wall-clock estimate. ~10 bits/byte on the wire, and the log
    protocol's framing overhead means we net well under the raw rate."""
    bytes_per_s = (baud / 10.0) * 0.55
    return size_bytes / max(bytes_per_s, 1.0)


def cmd_list(mav):
    logs = list_logs(mav)
    if not logs:
        print("No logs on the FC (or it did not answer LOG_REQUEST_LIST).")
        return 1
    print(f"\n{len(logs)} log(s) on the flight controller:\n")
    print(f"  {'id':>4}  {'size':>10}  {'est. download':>14}   date")
    print(f"  {'-'*4}  {'-'*10}  {'-'*14}   {'-'*20}")
    for log_id, size, t_utc in logs:
        eta = _eta(size, BAUD_RATE)
        eta_txt = f"{eta/60:.0f} min" if eta >= 90 else f"{eta:.0f} s"
        print(f"  {log_id:>4}  {_fmt_size(size):>10}  {eta_txt:>14}   "
              f"{_fmt_utc(t_utc)}")
    print(f"\n  newest is id {logs[-1][0]}  ->  "
          f"python FetchFcLogs.py get last")
    print("  (large log? pulling the FC's SD card is much faster)")
    return 0


def download_log(mav, log_id, size, out_path):
    """Fetch one log to out_path, re-requesting any chunks that go missing."""
    n_chunks = (size + CHUNK - 1) // CHUNK
    data = bytearray(size)
    have = bytearray(n_chunks)          # 1 byte per chunk: received flag
    got = 0

    print(f"\nDownloading log {log_id}: {_fmt_size(size)} "
          f"({n_chunks} chunks). Ctrl-C to abort.")
    eta = _eta(size, BAUD_RATE)
    print(f"Estimated {eta/60:.1f} min at {BAUD_RATE} baud.\n")

    started = time.monotonic()
    mav.mav.log_request_data_send(mav.target_system, mav.target_component,
                                  log_id, 0, 0xFFFFFFFF)

    stalls = 0
    last_rx = time.monotonic()
    last_report = 0.0

    while got < n_chunks:
        msg = mav.recv_match(type='LOG_DATA', blocking=True, timeout=0.5)
        now = time.monotonic()

        if msg is not None and msg.id == log_id:
            idx = msg.ofs // CHUNK
            if 0 <= idx < n_chunks and not have[idx]:
                payload = bytes(bytearray(msg.data)[:msg.count])
                data[msg.ofs:msg.ofs + len(payload)] = payload
                have[idx] = 1
                got += 1
            last_rx = now
            stalls = 0

        if now - last_report >= 1.0:
            pct = 100.0 * got / n_chunks
            elapsed = now - started
            rate = (got * CHUNK) / elapsed if elapsed > 0 else 0
            remain = ((n_chunks - got) * CHUNK / rate) if rate > 0 else 0
            sys.stdout.write(
                f"\r  {pct:5.1f}%  {got}/{n_chunks} chunks  "
                f"{rate/1024:.1f} KB/s  ~{remain/60:.1f} min left   ")
            sys.stdout.flush()
            last_report = now

        # Nothing arriving -> ask again for what is still missing.
        if now - last_rx > STALL_TIMEOUT_S:
            stalls += 1
            if stalls > MAX_STALLS:
                print(f"\n!! Gave up: {n_chunks - got} chunks still missing "
                      f"after {MAX_STALLS} retries.")
                break
            start_idx = next((i for i in range(n_chunks) if not have[i]), None)
            if start_idx is None:
                break
            ofs = start_idx * CHUNK
            mav.mav.log_request_data_send(
                mav.target_system, mav.target_component,
                log_id, ofs, 0xFFFFFFFF)
            last_rx = now

    mav.mav.log_request_end_send(mav.target_system, mav.target_component)

    complete = (got == n_chunks)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_bytes(bytes(data))
    elapsed = time.monotonic() - started
    print(f"\n{'Saved' if complete else 'Saved (INCOMPLETE)'}: {out_path}  "
          f"({_fmt_size(len(data))} in {elapsed/60:.1f} min)")
    if not complete:
        print("!! The file has gaps and may not parse. Retry, or pull the SD "
              "card.")
    return complete


def cmd_get(mav, which, out_dir):
    logs = list_logs(mav)
    if not logs:
        print("No logs on the FC.")
        return 1

    if which == 'last':
        targets = [logs[-1]]
    elif which == 'all':
        targets = logs
    else:
        try:
            want = int(which)
        except ValueError:
            print(f"Not a log id: {which!r} (use an id, 'last' or 'all')")
            return 2
        targets = [e for e in logs if e[0] == want]
        if not targets:
            print(f"No log with id {want}. Available: "
                  f"{', '.join(str(e[0]) for e in logs)}")
            return 2

    ok = True
    for log_id, size, t_utc in targets:
        stamp = time.strftime("%Y%m%d_%H%M%S",
                              time.localtime(t_utc)) if t_utc else "nodate"
        out = Path(out_dir) / f"fc_{log_id:03d}_{stamp}.BIN"
        ok &= download_log(mav, log_id, size, out)
        if out.exists():
            print(f"Next: python FetchFcLogs.py summarise {out}")
    return 0 if ok else 1


# ----------------------------------------------------------------------
# Offline summarising -- no FC connection needed
# ----------------------------------------------------------------------
def summarise(bin_path, out_dir=None):
    """Turn a .BIN into a small readable .txt + .csv worth sharing."""
    bin_path = Path(bin_path)
    if not bin_path.exists():
        print(f"No such file: {bin_path}")
        return 2
    out_dir = Path(out_dir) if out_dir else bin_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    txt_path = out_dir / f"{bin_path.stem}_summary.txt"
    csv_path = out_dir / f"{bin_path.stem}_trace.csv"

    print(f"Reading {bin_path} ...")
    mlog = mavutil.mavlink_connection(str(bin_path))

    events = []          # (t, kind, text)
    modes = []           # (t, mode_name)
    trace = []           # downsampled numeric rows
    counts = {}
    t_first = t_last = None
    last_trace_t = -1e9
    TRACE_DT = 0.2       # ~5 Hz is plenty for eyeballing and keeps the CSV small

    # Latest value of each thing we care about, so a row is filled in from
    # whichever messages have arrived most recently.
    cur = {k: "" for k in (
        "mode", "DesRoll", "Roll", "DesPitch", "Pitch", "DesYaw", "Yaw",
        "DesRateYaw", "RateYaw", "DAlt", "Alt", "DCRt", "CRt", "ThO", "ch6")}

    while True:
        m = mlog.recv_match()
        if m is None:
            break
        mtype = m.get_type()
        counts[mtype] = counts.get(mtype, 0) + 1
        t = getattr(m, '_timestamp', None)
        if t is None:
            continue
        if t_first is None:
            t_first = t
        t_last = t

        if mtype == 'MODE':
            name = getattr(m, 'Mode', getattr(m, 'ModeNum', '?'))
            cur["mode"] = name
            modes.append((t - t_first, str(name)))
        elif mtype in ('MSG', 'STATUSTEXT'):
            txt = getattr(m, 'Message', getattr(m, 'text', '')).strip()
            events.append((t - t_first, 'MSG', txt))
        elif mtype == 'ERR':
            events.append((t - t_first, 'ERR',
                           f"subsys={getattr(m,'Subsys','?')} "
                           f"ecode={getattr(m,'ECode','?')}"))
        elif mtype == 'EV':
            events.append((t - t_first, 'EV', f"id={getattr(m,'Id','?')}"))
        elif mtype == 'ATT':
            for k in ('DesRoll', 'Roll', 'DesPitch', 'Pitch', 'DesYaw', 'Yaw'):
                if hasattr(m, k):
                    cur[k] = round(getattr(m, k), 2)
        elif mtype == 'RATE':
            if hasattr(m, 'YDes'):
                cur["DesRateYaw"] = round(m.YDes, 3)
            if hasattr(m, 'Y'):
                cur["RateYaw"] = round(m.Y, 3)
        elif mtype == 'CTUN':
            for src, dst in (('DAlt', 'DAlt'), ('Alt', 'Alt'),
                             ('DCRt', 'DCRt'), ('CRt', 'CRt'),
                             ('ThO', 'ThO')):
                if hasattr(m, src):
                    cur[dst] = round(getattr(m, src), 3)
        elif mtype == 'RCIN':
            if hasattr(m, 'C6'):
                cur["ch6"] = m.C6

        if t - last_trace_t >= TRACE_DT:
            last_trace_t = t
            trace.append(dict(t=round(t - t_first, 2), **cur))

    duration = (t_last - t_first) if (t_first and t_last) else 0

    # ---- summary text ----
    lines = []
    lines.append(f"FC dataflash summary: {bin_path.name}")
    lines.append(f"duration: {duration:.1f} s ({duration/60:.1f} min)")
    lines.append("")
    lines.append("flight mode timeline:")
    if modes:
        for t, name in modes:
            lines.append(f"  {t:8.1f}s  {name}")
    else:
        lines.append("  (no MODE messages found)")
    lines.append("")
    lines.append("errors / events / statustexts:")
    if events:
        for t, kind, txt in events[:400]:
            lines.append(f"  {t:8.1f}s  {kind:4s}  {txt}")
        if len(events) > 400:
            lines.append(f"  ... {len(events)-400} more")
    else:
        lines.append("  (none)")
    lines.append("")
    lines.append("message counts:")
    for k in sorted(counts, key=lambda k: -counts[k])[:25]:
        lines.append(f"  {k:12s} {counts[k]}")
    txt_path.write_text("\n".join(lines) + "\n")

    # ---- numeric trace ----
    if trace:
        import csv as _csv
        cols = ["t"] + [k for k in cur]
        with csv_path.open("w", newline="") as f:
            w = _csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            w.writerows(trace)

    print(f"\nWrote:\n  {txt_path}\n  {csv_path}  ({len(trace)} rows)")
    print("\nThose two files are small enough to share directly.")
    print("Key columns: DesRoll/Roll and DesPitch/Pitch (did the FC accept our")
    print("attitude commands?), DesYaw/Yaw, DCRt/CRt (commanded vs actual")
    print("climb rate), ch6 (the mode switch).")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="Fetch and summarise the FC's dataflash (.BIN) logs.")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="list logs stored on the FC")

    g = sub.add_parser("get", help="download a log from the FC")
    g.add_argument("which", help="log id, 'last', or 'all'")
    g.add_argument("--out", default=None,
                   help="output directory (default: <project>/logs/fc)")

    s = sub.add_parser("summarise", aliases=["summarize"],
                       help="turn a .BIN into a shareable .txt + .csv")
    s.add_argument("binfile")
    s.add_argument("--out", default=None, help="output directory")

    args = ap.parse_args()

    if args.cmd in ("summarise", "summarize"):
        return summarise(args.binfile, args.out)

    default_out = Path(__file__).resolve().parents[2] / "logs" / "fc"
    out_dir = Path(args.out) if args.out else default_out

    mav = connect()
    if args.cmd == "list":
        return cmd_list(mav)
    if args.cmd == "get":
        return cmd_get(mav, args.which, out_dir)
    return 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nAborted.")
        sys.exit(130)
