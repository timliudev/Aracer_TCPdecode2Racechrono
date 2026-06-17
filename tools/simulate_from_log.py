import argparse
import sys
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


def run_simulation(log_path, chunk_size):
    framer = HeaderFramer(FRAME_HEADER)
    decoder = decoode.EcuDecoder()
    line_count = 0
    byte_count = 0
    frame_count = 0
    output_count = 0
    last_output = ""

    for _, logged_chunk in iter_log_chunks(log_path):
        line_count += 1
        byte_count += len(logged_chunk)

        for tcp_chunk in split_chunk(logged_chunk, chunk_size):
            for frame in framer.feed(tcp_chunk):
                frame_count += 1
                output = decoder.decode_frame(frame)
                if output:
                    output_count += 1
                    last_output = output

    return {
        "log": str(log_path),
        "chunk_size": chunk_size or "log-line",
        "lines": line_count,
        "bytes": byte_count,
        "frames": frame_count,
        "outputs": output_count,
        "remaining_buffer": len(framer.buffer),
        "last_output": last_output.splitlines()[-1] if last_output else "",
    }


def main():
    parser = argparse.ArgumentParser(description="Replay ECU logs through the TCP framer and decoder.")
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
        baseline = None
        for chunk_size in chunk_sizes:
            result = run_simulation(log_path, chunk_size)

            if baseline is None:
                baseline = result
            else:
                if result["frames"] != baseline["frames"]:
                    raise AssertionError(
                        f"{log_path}: frame count differs for chunk_size={chunk_size}: "
                        f"{result['frames']} != {baseline['frames']}"
                    )
                if result["outputs"] != baseline["outputs"]:
                    raise AssertionError(
                        f"{log_path}: output count differs for chunk_size={chunk_size}: "
                        f"{result['outputs']} != {baseline['outputs']}"
                    )

            print(
                f"{log_path} chunk={result['chunk_size']} "
                f"lines={result['lines']} bytes={result['bytes']} "
                f"frames={result['frames']} outputs={result['outputs']} "
                f"remaining_buffer={result['remaining_buffer']}"
            )

        if baseline and baseline["last_output"]:
            print(f"last_output: {baseline['last_output']}")


if __name__ == "__main__":
    main()
