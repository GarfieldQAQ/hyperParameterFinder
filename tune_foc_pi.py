#!/usr/bin/env python3
"""Automatic PI search tool for the Xtepper FOC firmware."""

from __future__ import annotations

import argparse
import csv
import math
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

JUSTFLOAT_TAIL = b"\x00\x00\x80\x7f"
CURRENT_CHANNELS = (
    "target_id",
    "target_iq",
    "id",
    "iq",
    "id_raw",
    "iq_raw",
    "vd",
    "vq",
    "va",
    "vb",
    "id_i",
    "iq_i",
)


@dataclass
class CandidateResult:
    kp: float
    ki: float
    score: float
    overshoot: float
    settle_error: float
    ripple: float
    rise_time_s: float
    saturation: float
    sample_count: int
    csv_path: Path


class JustFloatParser:
    """Incremental VOFA JustFloat parser for a fixed channel count."""

    def __init__(self, channel_count: int) -> None:
        self.channel_count = channel_count
        self.frame_size = channel_count * 4 + len(JUSTFLOAT_TAIL)
        self.buffer = bytearray()

    def feed(self, data: bytes) -> List[Tuple[float, ...]]:
        self.buffer.extend(data)
        frames: List[Tuple[float, ...]] = []

        while True:
            tail_index = self.buffer.find(JUSTFLOAT_TAIL)
            if tail_index < 0:
                max_keep = self.frame_size * 2
                if len(self.buffer) > max_keep:
                    del self.buffer[:-max_keep]
                break

            start = tail_index - self.channel_count * 4
            if start < 0:
                del self.buffer[: tail_index + len(JUSTFLOAT_TAIL)]
                continue

            payload = bytes(self.buffer[start:tail_index])
            del self.buffer[: tail_index + len(JUSTFLOAT_TAIL)]
            try:
                frames.append(struct.unpack("<" + "f" * self.channel_count, payload))
            except struct.error:
                continue

        return frames


class XtepperSerial:
    def __init__(self, port: str, baud: int, timeout: float = 0.02) -> None:
        try:
            import serial
        except ImportError as exc:
            raise SystemExit(
                "pyserial is required. Install with: python -m pip install -r requirements.txt"
            ) from exc

        self.serial = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        time.sleep(0.2)
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

    def close(self) -> None:
        self.serial.close()

    def command(self, line: str, quiet: bool = False) -> None:
        if not quiet:
            print(f"> {line}")
        self.serial.write((line.strip() + "\r\n").encode("ascii"))
        self.serial.flush()

    def read_available(self) -> bytes:
        waiting = self.serial.in_waiting
        if waiting <= 0:
            return b""
        return self.serial.read(waiting)

    def drain_text(self, duration_s: float = 0.2) -> str:
        end = time.monotonic() + duration_s
        data = bytearray()
        while time.monotonic() < end:
            chunk = self.read_available()
            if chunk:
                data.extend(chunk)
            else:
                time.sleep(0.01)
        return data.decode("utf-8", errors="replace")


def parse_float_list(spec: str) -> List[float]:
    """Parse either 'a,b,c' or 'start:stop:step'."""

    spec = spec.strip()
    if ":" in spec:
        parts = [float(p) for p in spec.split(":")]
        if len(parts) != 3:
            raise ValueError("range syntax must be start:stop:step")
        start, stop, step = parts
        if step == 0.0:
            raise ValueError("range step cannot be zero")
        values: List[float] = []
        x = start
        if step > 0:
            while x <= stop + abs(step) * 0.5e-6:
                values.append(round(x, 9))
                x += step
        else:
            while x >= stop - abs(step) * 0.5e-6:
                values.append(round(x, 9))
                x += step
        return values
    return [float(p.strip()) for p in spec.split(",") if p.strip()]


def stdev(values: Sequence[float]) -> float:
    if len(values) < 2:
        return 0.0
    mean = sum(values) / len(values)
    return math.sqrt(sum((x - mean) ** 2 for x in values) / (len(values) - 1))


def collect_frames(
    link: XtepperSerial,
    parser: JustFloatParser,
    duration_s: float,
) -> List[Tuple[float, ...]]:
    end = time.monotonic() + duration_s
    frames: List[Tuple[float, ...]] = []
    while time.monotonic() < end:
        chunk = link.read_available()
        if chunk:
            frames.extend(parser.feed(chunk))
        else:
            time.sleep(0.002)
    return frames


def score_current_response(
    frames: Sequence[Tuple[float, ...]],
    target: float,
    capture_s: float,
    sample_period_s: float,
    output_limit: float,
) -> Tuple[float, float, float, float, float, float]:
    if len(frames) < 10:
        return (1e9, 1e9, 1e9, 1e9, capture_s, 1e9)

    iq = [f[3] for f in frames]
    vq = [f[7] for f in frames]
    abs_target = max(abs(target), 1e-6)
    final_window = max(5, len(iq) // 5)
    final_samples = iq[-final_window:]
    final_mean = sum(final_samples) / len(final_samples)

    peak = max(iq) if target >= 0 else min(iq)
    if target >= 0:
        overshoot = max(0.0, (peak - target) / abs_target)
    else:
        overshoot = max(0.0, (target - peak) / abs_target)
    settle_error = abs(final_mean - target) / abs_target
    ripple = stdev(final_samples) / abs_target

    threshold = target * 0.9
    rise_time_s = capture_s
    for index, value in enumerate(iq):
        if (target >= 0 and value >= threshold) or (target < 0 and value <= threshold):
            rise_time_s = index * sample_period_s
            break

    saturation = sum(1 for value in vq if abs(value) >= output_limit * 0.98) / len(vq)
    score = (
        overshoot * 4.0
        + settle_error * 5.0
        + ripple * 3.0
        + saturation * 2.0
        + min(rise_time_s / max(capture_s, sample_period_s), 2.0)
    )
    return score, overshoot, settle_error, ripple, rise_time_s, saturation


def write_frames_csv(path: Path, frames: Sequence[Tuple[float, ...]], sample_period_s: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.writer(fp)
        writer.writerow(("t_s",) + CURRENT_CHANNELS)
        for index, frame in enumerate(frames):
            writer.writerow((index * sample_period_s, *frame))


def write_summary_csv(path: Path, results: Sequence[CandidateResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.writer(fp)
        writer.writerow(
            (
                "rank",
                "kp",
                "ki",
                "score",
                "overshoot",
                "settle_error",
                "ripple",
                "rise_time_s",
                "saturation",
                "sample_count",
                "csv_path",
            )
        )
        for rank, result in enumerate(sorted(results, key=lambda x: x.score), start=1):
            writer.writerow(
                (
                    rank,
                    result.kp,
                    result.ki,
                    result.score,
                    result.overshoot,
                    result.settle_error,
                    result.ripple,
                    result.rise_time_s,
                    result.saturation,
                    result.sample_count,
                    str(result.csv_path),
                )
            )


def candidate_name(kp: float, ki: float) -> str:
    return f"current_kp_{kp:.6g}_ki_{ki:.6g}".replace(".", "p").replace("-", "m")


def run_current_candidate(
    link: XtepperSerial,
    kp: float,
    ki: float,
    target_iq: float,
    output_limit: float,
    settle_s: float,
    capture_s: float,
    sample_period_s: float,
    output_dir: Path,
) -> CandidateResult:
    parser = JustFloatParser(channel_count=len(CURRENT_CHANNELS))

    link.command("vofa off")
    link.drain_text(0.15)
    link.command("foc off")
    link.drain_text(0.10)
    link.command("foc prep current")
    link.drain_text(0.20)
    link.command(f"foc limit {output_limit:.6g}")
    link.command(f"foc ckp {kp:.6g}")
    link.command(f"foc cki {ki:.6g}")
    link.command("foc id 0")
    link.command("foc iq 0")
    link.drain_text(0.15)
    link.command("vofa current on")
    time.sleep(0.05)
    link.command("foc on")
    time.sleep(settle_s)
    _ = collect_frames(link, parser, 0.2)

    link.command(f"foc iq {target_iq:.6g}", quiet=True)
    frames = collect_frames(link, parser, capture_s)
    link.command("foc iq 0", quiet=True)
    time.sleep(0.1)
    link.command("foc off", quiet=True)
    link.command("vofa off", quiet=True)
    link.drain_text(0.2)

    csv_path = output_dir / f"{candidate_name(kp, ki)}.csv"
    write_frames_csv(csv_path, frames, sample_period_s)

    score, overshoot, settle_error, ripple, rise_time_s, saturation = score_current_response(
        frames=frames,
        target=target_iq,
        capture_s=capture_s,
        sample_period_s=sample_period_s,
        output_limit=output_limit,
    )
    return CandidateResult(
        kp=kp,
        ki=ki,
        score=score,
        overshoot=overshoot,
        settle_error=settle_error,
        ripple=ripple,
        rise_time_s=rise_time_s,
        saturation=saturation,
        sample_count=len(frames),
        csv_path=csv_path,
    )


def run_current_search(args: argparse.Namespace) -> int:
    kp_values = parse_float_list(args.kp)
    ki_values = parse_float_list(args.ki)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    output_dir = Path(args.output) / f"current_{timestamp}"
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Opening {args.port} at {args.baud} baud")
    link = XtepperSerial(args.port, args.baud)
    results: List[CandidateResult] = []
    try:
        total = len(kp_values) * len(ki_values)
        index = 0
        for kp in kp_values:
            for ki in ki_values:
                index += 1
                print(f"\n[{index}/{total}] current loop kp={kp:.6g}, ki={ki:.6g}")
                result = run_current_candidate(
                    link=link,
                    kp=kp,
                    ki=ki,
                    target_iq=args.target_iq,
                    output_limit=args.limit,
                    settle_s=args.settle,
                    capture_s=args.capture,
                    sample_period_s=args.sample_period,
                    output_dir=output_dir,
                )
                results.append(result)
                print(
                    "score={:.4f} overshoot={:.3f} settle={:.3f} "
                    "ripple={:.3f} rise={:.3f}s sat={:.3f} samples={}".format(
                        result.score,
                        result.overshoot,
                        result.settle_error,
                        result.ripple,
                        result.rise_time_s,
                        result.saturation,
                        result.sample_count,
                    )
                )
    finally:
        try:
            link.command("foc iq 0", quiet=True)
            link.command("foc off", quiet=True)
            link.command("vofa off", quiet=True)
        except Exception:
            pass
        link.close()

    summary_path = output_dir / "summary.csv"
    write_summary_csv(summary_path, results)
    best = min(results, key=lambda x: x.score)
    print("\nBest current PI:")
    print(f"  ckp={best.kp:.6g}")
    print(f"  cki={best.ki:.6g}")
    print(f"  score={best.score:.4f}")
    print(f"  summary={summary_path}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Search Xtepper FOC PI parameters over UART")
    parser.add_argument("--port", required=True, help="Serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--loop", choices=("current",), default="current", help="Loop to tune")
    parser.add_argument("--kp", default="0.005:0.05:0.005", help="Kp list or start:stop:step")
    parser.add_argument("--ki", default="0.5:5.0:0.5", help="Ki list or start:stop:step")
    parser.add_argument("--target-iq", type=float, default=0.2, help="iq step target")
    parser.add_argument("--limit", type=float, default=0.3, help="FOC output limit")
    parser.add_argument("--settle", type=float, default=0.5, help="seconds to settle before step")
    parser.add_argument("--capture", type=float, default=2.0, help="seconds to capture after step")
    parser.add_argument("--sample-period", type=float, default=0.01, help="VOFA frame period seconds")
    parser.add_argument("--output", default="output", help="output directory")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.loop == "current":
        return run_current_search(args)
    parser.error(f"unsupported loop: {args.loop}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
