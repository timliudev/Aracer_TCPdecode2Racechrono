import argparse
import statistics
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

import decoode
from main import FRAME_HEADER, HeaderFramer


def iter_log_chunks(log_path):
    with log_path.open("r", encoding="utf-8") as file:
        for line_no, line in enumerate(file, 1):
            line = line.strip()
            if not line or "," not in line:
                continue

            _, hex_data = line.split(",", 1)
            hex_data = hex_data.strip()
            if not hex_data:
                continue

            try:
                yield line_no, bytes.fromhex(hex_data)
            except ValueError as exc:
                raise ValueError(f"{log_path}:{line_no}: invalid hex data") from exc


def split_chunk(chunk, chunk_size):
    if chunk_size <= 0:
        yield chunk
        return

    for start in range(0, len(chunk), chunk_size):
        yield chunk[start:start + chunk_size]


def percentile(values, pct):
    if not values:
        return 0.0

    values = sorted(values)
    index = (len(values) - 1) * pct / 100
    lower = int(index)
    upper = min(lower + 1, len(values) - 1)
    weight = index - lower
    return values[lower] * (1 - weight) + values[upper] * weight


def summarize_us(values):
    if not values:
        return {
            "count": 0,
            "avg": 0.0,
            "median": 0.0,
            "p95": 0.0,
            "p99": 0.0,
            "max": 0.0,
        }

    return {
        "count": len(values),
        "avg": statistics.fmean(values),
        "median": statistics.median(values),
        "p95": percentile(values, 95),
        "p99": percentile(values, 99),
        "max": max(values),
    }


def run_measurement(log_path, chunk_size):
    framer = HeaderFramer(FRAME_HEADER)
    decoder = decoode.EcuDecoder()

    line_count = 0
    byte_count = 0
    frame_count = 0
    output_count = 0
    chunk_times_us = []
    frame_times_us = []
    output_ready_times_us = []

    total_start = time.perf_counter_ns()

    for _, logged_chunk in iter_log_chunks(log_path):
        line_count += 1
        byte_count += len(logged_chunk)

        for tcp_chunk in split_chunk(logged_chunk, chunk_size):
            chunk_start = time.perf_counter_ns()

            for frame in framer.feed(tcp_chunk):
                frame_count += 1
                frame_start = time.perf_counter_ns()
                output = decoder.decode_frame(frame)
                frame_end = time.perf_counter_ns()
                frame_times_us.append((frame_end - frame_start) / 1000)

                if output:
                    output.encode()
                    output_count += 1
                    output_ready_times_us.append((time.perf_counter_ns() - chunk_start) / 1000)

            chunk_times_us.append((time.perf_counter_ns() - chunk_start) / 1000)

    total_elapsed_ms = (time.perf_counter_ns() - total_start) / 1_000_000

    return {
        "log": str(log_path),
        "chunk_size": chunk_size or "log-line",
        "lines": line_count,
        "bytes": byte_count,
        "frames": frame_count,
        "outputs": output_count,
        "remaining_buffer": len(framer.buffer),
        "total_elapsed_ms": total_elapsed_ms,
        "chunks": summarize_us(chunk_times_us),
        "frames_latency": summarize_us(frame_times_us),
        "outputs_latency": summarize_us(output_ready_times_us),
    }


def print_summary(result):
    chunks = result["chunks"]
    frames_latency = result["frames_latency"]
    outputs_latency = result["outputs_latency"]

    print(f"\n{result['log']} chunk={result['chunk_size']}")
    print(
        f"  input: lines={result['lines']} bytes={result['bytes']} "
        f"frames={result['frames']} outputs={result['outputs']} "
        f"remaining_buffer={result['remaining_buffer']}"
    )
    print(f"  total replay time: {result['total_elapsed_ms']:.3f} ms")
    print(
        "  dataReceived chunk processing us: "
        f"avg={chunks['avg']:.3f} median={chunks['median']:.3f} "
        f"p95={chunks['p95']:.3f} p99={chunks['p99']:.3f} max={chunks['max']:.3f}"
    )
    print(
        "  per-frame decode us: "
        f"avg={frames_latency['avg']:.3f} median={frames_latency['median']:.3f} "
        f"p95={frames_latency['p95']:.3f} p99={frames_latency['p99']:.3f} "
        f"max={frames_latency['max']:.3f}"
    )
    print(
        "  TCP chunk start -> output encoded us: "
        f"avg={outputs_latency['avg']:.3f} median={outputs_latency['median']:.3f} "
        f"p95={outputs_latency['p95']:.3f} p99={outputs_latency['p99']:.3f} "
        f"max={outputs_latency['max']:.3f}"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Measure framer + decoder + output encode latency from ECU logs."
    )
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument(
        "--chunk-size",
        action="append",
        type=int,
        default=[],
        help="Replay each logged TCP chunk as smaller chunks of this size. Use 0 for original log chunks.",
    )
    args = parser.parse_args()

    chunk_sizes = args.chunk_size or [0, 1, 7, 19, 64]

    for log_path in args.logs:
        for chunk_size in chunk_sizes:
            print_summary(run_measurement(log_path, chunk_size))


if __name__ == "__main__":
    main()
