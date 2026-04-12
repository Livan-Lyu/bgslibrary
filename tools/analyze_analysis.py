#!/usr/bin/env python3
"""Parse and summarize BGSLibrary analysis logs.

Supported records:
- ViBe    time(sec):0.069625
- [ViBe C3R Seg] over 100 frames:
- [ViBe C3R] Seg vs Update (over 100 frames):
- [ViBe C3R Update] over 100 frames:
"""

from __future__ import annotations

import argparse
import math
import re
import statistics
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

TIME_RE = re.compile(r"^ViBe\s+time\(sec\):\s*([0-9]+(?:\.[0-9]+)?)\s*$")
SEG_HEADER_RE = re.compile(r"^\[ViBe C3R Seg\]\s+over\s+(\d+)\s+frames:\s*$")
SEG_VS_UPDATE_HEADER_RE = re.compile(r"^\[ViBe C3R\]\s+Seg vs Update\s+\(over\s+(\d+)\s+frames\):\s*$")
UPDATE_HEADER_RE = re.compile(r"^\[ViBe C3R Update\]\s+over\s+(\d+)\s+frames:\s*$")
INDENT_METRIC_RE = re.compile(r"^\s*([A-Za-z_ ]+):\s*([-+]?[0-9]*\.?[0-9]+)%\s*$")
SEG_UPDATE_LINE_RE = re.compile(
    r"^\s*(Seg|Update):\s*([-+]?[0-9]*\.?[0-9]+)%\s*\(\s*([-+]?[0-9]*\.?[0-9]+)\s*s\s*total,\s*([-+]?[0-9]*\.?[0-9]+)\s*ms/frame\s*\)\s*$"
)


@dataclass
class Checkpoint:
    frames: int
    seg_metrics: Dict[str, float] = field(default_factory=dict)
    seg_vs_update: Dict[str, Dict[str, float]] = field(default_factory=dict)
    update_metrics: Dict[str, float] = field(default_factory=dict)


@dataclass
class ParseResult:
    times_sec: List[float] = field(default_factory=list)
    checkpoints: List[Checkpoint] = field(default_factory=list)
    unknown_lines: int = 0


def percentile(sorted_values: List[float], q: float) -> float:
    if not sorted_values:
        return math.nan
    if q <= 0:
        return sorted_values[0]
    if q >= 100:
        return sorted_values[-1]

    k = (len(sorted_values) - 1) * (q / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return sorted_values[int(k)]
    return sorted_values[f] * (c - k) + sorted_values[c] * (k - f)


def _parse_indented_metrics(lines: List[str], start: int) -> tuple[Dict[str, float], int]:
    metrics: Dict[str, float] = {}
    i = start
    while i < len(lines):
        line = lines[i].rstrip("\n")
        m = INDENT_METRIC_RE.match(line)
        if not m:
            break
        key = m.group(1).strip().replace(" ", "_")
        metrics[key] = float(m.group(2))
        i += 1
    return metrics, i


def parse_file(path: Path) -> ParseResult:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    result = ParseResult()

    i = 0
    while i < len(lines):
        line = lines[i].strip("\n")

        m = TIME_RE.match(line)
        if m:
            result.times_sec.append(float(m.group(1)))
            i += 1
            continue

        m = SEG_HEADER_RE.match(line)
        if m:
            cp = Checkpoint(frames=int(m.group(1)))
            cp.seg_metrics, i = _parse_indented_metrics(lines, i + 1)
            result.checkpoints.append(cp)
            continue

        m = SEG_VS_UPDATE_HEADER_RE.match(line)
        if m:
            frames = int(m.group(1))
            cp = next((c for c in result.checkpoints if c.frames == frames), None)
            if cp is None:
                cp = Checkpoint(frames=frames)
                result.checkpoints.append(cp)

            i += 1
            while i < len(lines):
                sm = SEG_UPDATE_LINE_RE.match(lines[i])
                if not sm:
                    break
                name = sm.group(1).lower()
                cp.seg_vs_update[name] = {
                    "ratio_percent": float(sm.group(2)),
                    "total_seconds": float(sm.group(3)),
                    "ms_per_frame": float(sm.group(4)),
                }
                i += 1
            continue

        m = UPDATE_HEADER_RE.match(line)
        if m:
            frames = int(m.group(1))
            cp = next((c for c in result.checkpoints if c.frames == frames), None)
            if cp is None:
                cp = Checkpoint(frames=frames)
                result.checkpoints.append(cp)
            cp.update_metrics, i = _parse_indented_metrics(lines, i + 1)
            continue

        if line.strip():
            result.unknown_lines += 1
        i += 1

    result.checkpoints.sort(key=lambda c: c.frames)
    return result


def summarize_times(times: List[float]) -> str:
    if not times:
        return "No 'ViBe time(sec)' samples found."

    s = sorted(times)
    n = len(s)
    mean_s = statistics.fmean(s)
    std_s = statistics.stdev(s) if n > 1 else 0.0

    lines = [
        "=== ViBe time(sec) Summary ===",
        f"count:      {n}",
        f"mean(sec):  {mean_s:.6f}",
        f"std(sec):   {std_s:.6f}",
        f"min(sec):   {s[0]:.6f}",
        f"p50(sec):   {percentile(s, 50):.6f}",
        f"p90(sec):   {percentile(s, 90):.6f}",
        f"p95(sec):   {percentile(s, 95):.6f}",
        f"p99(sec):   {percentile(s, 99):.6f}",
        f"max(sec):   {s[-1]:.6f}",
    ]

    fps_values = [1.0 / v for v in s if v > 0]
    if fps_values:
        fps_sorted = sorted(fps_values)
        lines.extend(
            [
                "--- FPS (derived from 1/time) ---",
                f"mean(fps):  {statistics.fmean(fps_sorted):.2f}",
                f"p50(fps):   {percentile(fps_sorted, 50):.2f}",
                f"p90(fps):   {percentile(fps_sorted, 90):.2f}",
                f"min(fps):   {fps_sorted[0]:.2f}",
                f"max(fps):   {fps_sorted[-1]:.2f}",
            ]
        )

    return "\n".join(lines)


def summarize_checkpoints(checkpoints: List[Checkpoint]) -> str:
    if not checkpoints:
        return "No '[ViBe C3R ...]' checkpoint blocks found."

    lines = ["=== ViBe C3R Checkpoints ==="]
    for cp in checkpoints:
        lines.append(f"[frames={cp.frames}]")
        if cp.seg_metrics:
            seg_parts = ", ".join(f"{k}={v:.2f}%" for k, v in cp.seg_metrics.items())
            lines.append(f"  seg: {seg_parts}")
        if cp.seg_vs_update:
            seg = cp.seg_vs_update.get("seg")
            upd = cp.seg_vs_update.get("update")
            if seg:
                lines.append(
                    f"  seg_total: {seg['total_seconds']:.6f}s, {seg['ms_per_frame']:.4f}ms/frame, {seg['ratio_percent']:.2f}%"
                )
            if upd:
                lines.append(
                    f"  update_total: {upd['total_seconds']:.6f}s, {upd['ms_per_frame']:.4f}ms/frame, {upd['ratio_percent']:.2f}%"
                )
        if cp.update_metrics:
            upd_parts = ", ".join(f"{k}={v:.2f}%" for k, v in cp.update_metrics.items())
            lines.append(f"  update_distribution: {upd_parts}")

    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze BGSLibrary analysis log.")
    parser.add_argument("input", nargs="?", default="analysis.txt", help="Path to analysis log file")
    parser.add_argument("--json", action="store_true", help="Output compact JSON summary")
    args = parser.parse_args()

    path = Path(args.input)
    if not path.exists():
        raise SystemExit(f"Input file not found: {path}")

    result = parse_file(path)

    if args.json:
        import json

        s = sorted(result.times_sec)
        payload = {
            "file": str(path),
            "samples": len(s),
            "time_sec": {
                "mean": statistics.fmean(s) if s else None,
                "min": s[0] if s else None,
                "max": s[-1] if s else None,
                "p50": percentile(s, 50) if s else None,
                "p90": percentile(s, 90) if s else None,
                "p95": percentile(s, 95) if s else None,
                "p99": percentile(s, 99) if s else None,
            },
            "checkpoints": [
                {
                    "frames": c.frames,
                    "seg_metrics": c.seg_metrics,
                    "seg_vs_update": c.seg_vs_update,
                    "update_metrics": c.update_metrics,
                }
                for c in result.checkpoints
            ],
            "unknown_lines": result.unknown_lines,
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return

    print(f"File: {path}")
    print(summarize_times(result.times_sec))
    print()
    print(summarize_checkpoints(result.checkpoints))
    if result.unknown_lines:
        print()
        print(f"[Info] Unparsed non-empty lines: {result.unknown_lines}")


if __name__ == "__main__":
    main()
