"""
bench_accel.py — Benchmark script for the 4×4 matrix multiplication accelerator.

Measures:
  • Round-trip latency  (load A+B → compute → read C[0][0])
  • Compute-only latency (just the CMD 0x03 → ACK window)
  • Throughput  (matrices per second, GOPs)
  • UART efficiency  (useful payload vs protocol overhead bytes)
  • Correctness under stress (random signed matrices, checked vs numpy)

Usage:
    pip install pyserial numpy
    python bench_accel.py --port COM5          # Windows
    python bench_accel.py --port /dev/ttyUSB1  # Linux/Mac
    python bench_accel.py --port COM5 --runs 200 --export results.json
"""

import argparse
import json
import math
import random
import statistics
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("Install pyserial first:  pip install pyserial")

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False
    print("[warn] numpy not found — correctness check skipped")


# ── Constants ──────────────────────────────────────────────────────────────────
CLOCK_HZ       = 100_000_000   # FPGA clock
BAUD           = 115_200
MATRIX_ELEM    = 16            # 4×4
BITS_PER_UART  = 10            # 8 data + 1 start + 1 stop
BYTE_TIME_US   = (1 / BAUD) * BITS_PER_UART * 1e6

# One full transaction byte counts:
#  CMD 0x01: 1 + 16 = 17 bytes TX, 0 RX
#  CMD 0x02: 1 + 16 = 17 bytes TX, 0 RX
#  CMD 0x03: 1 byte TX, 1 byte RX
#  CMD 0x04 × 16: (1+1) TX, 4 RX  → 32 TX, 64 RX
TX_BYTES_FULL = 17 + 17 + 1 + 32          # 67
RX_BYTES_FULL = 0  + 0  + 1 + 64          # 65
TOTAL_USEFUL_BYTES = MATRIX_ELEM + MATRIX_ELEM + MATRIX_ELEM * 4   # A+B input + C output
PROTOCOL_OVERHEAD  = (TX_BYTES_FULL + RX_BYTES_FULL) - TOTAL_USEFUL_BYTES

# Number of multiply-accumulate operations per 4×4 × 4×4
MACS_PER_MULTIPLY = 4 * 4 * 4   # 64


# ── Serial helpers ──────────────────────────────────────────────────────────────

def open_port(port: str, timeout: float = 2.0) -> serial.Serial:
    ser = serial.Serial(port, BAUD, timeout=timeout)
    time.sleep(0.1)
    ser.reset_input_buffer()
    return ser


def write_matrix(ser: serial.Serial, cmd: int, flat: list[int]) -> None:
    payload = bytes([cmd] + [v & 0xFF for v in flat])
    ser.write(payload)


def compute(ser: serial.Serial) -> float:
    """Send compute command; return round-trip time in µs."""
    ser.reset_input_buffer()
    t0 = time.perf_counter()
    ser.write(b'\x03')
    ack = ser.read(1)
    t1 = time.perf_counter()
    if ack != b'\xAA':
        raise RuntimeError(f"Bad ACK: {ack.hex() if ack else 'timeout'}")
    return (t1 - t0) * 1e6


def read_element(ser: serial.Serial, idx: int) -> int:
    ser.write(bytes([0x04, idx]))
    raw = ser.read(4)
    if len(raw) < 4:
        raise RuntimeError(f"Read timeout at index {idx}")
    return struct.unpack('<i', raw)[0]


def read_all(ser: serial.Serial) -> list[int]:
    return [read_element(ser, i) for i in range(MATRIX_ELEM)]


def rand_matrix() -> list[int]:
    return [random.randint(-128, 127) for _ in range(MATRIX_ELEM)]


def ref_multiply(a: list[int], b: list[int]) -> list[int]:
    """Pure-Python 4×4 signed multiply for correctness check."""
    c = [0] * 16
    for i in range(4):
        for j in range(4):
            acc = 0
            for k in range(4):
                acc += a[i*4+k] * b[k*4+j]
            c[i*4+j] = acc
    return c


# ── Benchmark routines ─────────────────────────────────────────────────────────

def bench_compute_only(ser: serial.Serial, runs: int, a: list[int], b: list[int]) -> list[float]:
    """
    Measure CMD 0x03 → ACK latency only (matrices pre-loaded once).
    Returns list of latencies in µs.
    """
    write_matrix(ser, 0x01, a)
    time.sleep(0.05)
    write_matrix(ser, 0x02, b)
    time.sleep(0.05)

    latencies = []
    for _ in range(runs):
        us = compute(ser)
        latencies.append(us)
    return latencies


def bench_round_trip(ser: serial.Serial, runs: int) -> tuple[list[float], list[float], int]:
    """
    Full round-trip: load A, load B, compute, read all 16 results.
    Returns (total_latencies_us, compute_latencies_us, errors).
    """
    total_lats   = []
    compute_lats = []
    errors = 0

    for _ in range(runs):
        a = rand_matrix()
        b = rand_matrix()
        expected = ref_multiply(a, b)

        t0 = time.perf_counter()

        write_matrix(ser, 0x01, a)
        write_matrix(ser, 0x02, b)
        compute_us = compute(ser)
        result = read_all(ser)

        t1 = time.perf_counter()

        total_lats.append((t1 - t0) * 1e6)
        compute_lats.append(compute_us)

        if result != expected:
            errors += 1

    return total_lats, compute_lats, errors


def bench_throughput(ser: serial.Serial, duration_s: float = 5.0) -> dict:
    """
    Run as many full round-trips as possible in duration_s seconds.
    """
    count  = 0
    errors = 0
    t_end  = time.perf_counter() + duration_s

    while time.perf_counter() < t_end:
        a = rand_matrix()
        b = rand_matrix()
        expected = ref_multiply(a, b)

        write_matrix(ser, 0x01, a)
        write_matrix(ser, 0x02, b)
        compute(ser)
        result = read_all(ser)

        if result != expected:
            errors += 1
        count += 1

    actual_s = time.perf_counter() - (t_end - duration_s)   # approx elapsed
    matrices_per_sec = count / duration_s
    gops = (matrices_per_sec * MACS_PER_MULTIPLY * 2) / 1e9  # MACs × 2 = ops

    return {
        "count"            : count,
        "errors"           : errors,
        "duration_s"       : duration_s,
        "matrices_per_sec" : matrices_per_sec,
        "gops"             : gops,
    }


# ── Reporting ──────────────────────────────────────────────────────────────────

def stats(values: list[float]) -> dict:
    return {
        "min"    : min(values),
        "max"    : max(values),
        "mean"   : statistics.mean(values),
        "median" : statistics.median(values),
        "stdev"  : statistics.stdev(values) if len(values) > 1 else 0.0,
        "p95"    : sorted(values)[int(len(values) * 0.95)],
        "p99"    : sorted(values)[int(len(values) * 0.99)],
    }


def print_stats(label: str, s: dict, unit: str = "µs") -> None:
    print(f"\n  {label}")
    print(f"    min    {s['min']:>10.1f} {unit}")
    print(f"    mean   {s['mean']:>10.1f} {unit}")
    print(f"    median {s['median']:>10.1f} {unit}")
    print(f"    p95    {s['p95']:>10.1f} {unit}")
    print(f"    p99    {s['p99']:>10.1f} {unit}")
    print(f"    max    {s['max']:>10.1f} {unit}")
    print(f"    stdev  {s['stdev']:>10.1f} {unit}")


def uart_budget() -> dict:
    """Theoretical UART timing breakdown in µs."""
    load_a_us    = 17 * BYTE_TIME_US
    load_b_us    = 17 * BYTE_TIME_US
    cmd_tx_us    =  1 * BYTE_TIME_US
    ack_rx_us    =  1 * BYTE_TIME_US
    read_cmd_us  = 32 * BYTE_TIME_US   # 16 × (0x04 + idx)
    read_data_us = 64 * BYTE_TIME_US   # 16 × 4 bytes
    compute_ns   = (10 / CLOCK_HZ) * 1e9  # 10 clock cycles
    overhead_us  = PROTOCOL_OVERHEAD * BYTE_TIME_US
    total_us     = load_a_us + load_b_us + cmd_tx_us + ack_rx_us + read_cmd_us + read_data_us
    return {
        "load_a_us"    : load_a_us,
        "load_b_us"    : load_b_us,
        "cmd_tx_us"    : cmd_tx_us,
        "ack_rx_us"    : ack_rx_us,
        "read_cmd_us"  : read_cmd_us,
        "read_data_us" : read_data_us,
        "compute_ns"   : compute_ns,
        "overhead_us"  : overhead_us,
        "total_us"     : total_us,
    }


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="FPGA matrix accelerator benchmark")
    parser.add_argument("--port",   required=True, help="Serial port (COM5 / /dev/ttyUSB1)")
    parser.add_argument("--runs",   type=int, default=100, help="Iterations for latency tests")
    parser.add_argument("--dur",    type=float, default=5.0, help="Seconds for throughput test")
    parser.add_argument("--export", help="Save results to JSON file")
    args = parser.parse_args()

    print(f"\n{'='*60}")
    print(f"  4×4 Matrix Accelerator Benchmark")
    print(f"  Port: {args.port}  |  Baud: {BAUD}  |  Runs: {args.runs}")
    print(f"{'='*60}")

    ser = open_port(args.port)

    # ── 1. Compute-only latency ────────────────────────────────────
    print(f"\n[1/3] Compute-only latency ({args.runs} runs, fixed matrices)...")
    a_fixed = [1]*16
    b_fixed = [1]*16
    co_lats = bench_compute_only(ser, args.runs, a_fixed, b_fixed)
    co_stats = stats(co_lats)
    print_stats("CMD 0x03 → ACK round-trip", co_stats)

    # ── 2. Full round-trip latency ─────────────────────────────────
    print(f"\n[2/3] Full round-trip latency ({args.runs} runs, random matrices)...")
    rt_lats, cmp_lats, rt_errors = bench_round_trip(ser, args.runs)
    rt_stats  = stats(rt_lats)
    cmp_stats = stats(cmp_lats)
    print_stats("Load A+B + compute + read all C", rt_stats)
    print_stats("Compute sub-step only (within round-trip)", cmp_stats)
    if rt_errors:
        print(f"\n  *** CORRECTNESS ERRORS: {rt_errors}/{args.runs} ***")
    else:
        print(f"\n  Correctness: PASS ({args.runs}/{args.runs})")

    # ── 3. Throughput ──────────────────────────────────────────────
    print(f"\n[3/3] Throughput test ({args.dur:.0f}s)...")
    tp = bench_throughput(ser, args.dur)
    print(f"\n  Matrices computed : {tp['count']}")
    print(f"  Errors            : {tp['errors']}")
    print(f"  Matrices / sec    : {tp['matrices_per_sec']:.2f}")
    print(f"  Effective GOPs    : {tp['gops']:.6f}  (UART-bottlenecked)")
    print(f"  Hardware GOPs     : {(MACS_PER_MULTIPLY * 2 * CLOCK_HZ / 10) / 1e9:.1f}  (compute only @ 100 MHz)")

    # ── 4. Theoretical UART budget ────────────────────────────────
    bud = uart_budget()
    print(f"\n  UART timing budget (theoretical @ {BAUD} baud):")
    print(f"    Load A           {bud['load_a_us']:>8.1f} µs")
    print(f"    Load B           {bud['load_b_us']:>8.1f} µs")
    print(f"    Compute trigger  {bud['cmd_tx_us']:>8.1f} µs")
    print(f"    ACK receive      {bud['ack_rx_us']:>8.1f} µs")
    print(f"    Read commands    {bud['read_cmd_us']:>8.1f} µs")
    print(f"    Read data        {bud['read_data_us']:>8.1f} µs")
    print(f"    ─────────────────────────────")
    print(f"    Total UART       {bud['total_us']:>8.1f} µs  ({bud['total_us']/1000:.2f} ms)")
    print(f"    Compute (hw)     {bud['compute_ns']:>8.1f} ns  (10 clock cycles)")
    print(f"    Protocol overhead{bud['overhead_us']:>8.1f} µs  ({100*bud['overhead_us']/bud['total_us']:.1f}% of UART time)")

    efficiency = (TOTAL_USEFUL_BYTES / (TX_BYTES_FULL + RX_BYTES_FULL)) * 100
    print(f"    Byte efficiency  {efficiency:>7.1f}%")

    ser.close()

    # ── 5. Export ──────────────────────────────────────────────────
    results = {
        "config": {
            "port": args.port, "baud": BAUD, "runs": args.runs,
            "clock_hz": CLOCK_HZ, "macs_per_multiply": MACS_PER_MULTIPLY,
        },
        "compute_only_us"  : co_stats,
        "round_trip_us"    : rt_stats,
        "compute_step_us"  : cmp_stats,
        "correctness_errors": rt_errors,
        "throughput"       : tp,
        "uart_budget_us"   : bud,
        "byte_efficiency_pct": efficiency,
    }

    if args.export:
        with open(args.export, "w") as f:
            json.dump(results, f, indent=2)
        print(f"\n  Results saved → {args.export}")

    print(f"\n{'='*60}\n")

    # ── 6. Print JSON for dashboard ────────────────────────────────
    print("Paste the block below into the dashboard:\n")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()