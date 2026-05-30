#!/usr/bin/env python3
"""Offline daily evidence analyzer for RM65B D1-D5 outputs.

The script is intentionally dependency-light: it reads rosbag2 sqlite metadata,
MoveIt summary JSON, exported camera frames, videos, screenshots, and logs from
the latest output directory. Message payload decoding is limited to the few CDR
types needed for offline proxies.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import shutil
import sqlite3
import struct
import subprocess
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


DAY_RE = re.compile(r"^day0[1-5]$")
FORCE_TOPICS = {
    "target": "/force_control/target_wrench",
    "error": "/force_control/wrench_error",
    "offset": "/force_control/admittance_offset",
    "corrected": "/force_control/corrected_right_joint_trajectory",
    "state": "/force_control/state",
}
CAMERA_TOPICS = [
    "/left_camera/image_rect",
    "/rm65b/evidence_camera/image",
    "/left_camera/camera_info",
]
TRAJECTORY_TOPICS = [
    "/dual_arm_planning/left_joint_trajectory",
    "/dual_arm_planning/right_joint_trajectory",
    "/force_control/corrected_right_joint_trajectory",
]


def repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[2]


def latest_output_root(outputs_dir: Path) -> Path:
    candidates = []
    for child in outputs_dir.iterdir() if outputs_dir.exists() else []:
        if not child.is_dir():
            continue
        days = [child / f"day0{i}" for i in range(1, 6)]
        if all(day.is_dir() for day in days):
            candidates.append(child)
    if not candidates:
        raise FileNotFoundError(f"no output root with day01-day05 under {outputs_dir}")
    return max(candidates, key=lambda p: p.stat().st_mtime)


def read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {"_read_error": str(exc)}


def parse_key_value_file(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    if not path.exists():
        return result
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            result[key.strip()] = value.strip()
    return result


def parse_ppm_header(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"exists": False}
    try:
        with path.open("rb") as fh:
            magic = fh.readline().strip()
            dims = fh.readline().strip()
            while dims.startswith(b"#"):
                dims = fh.readline().strip()
            maxval = fh.readline().strip()
        parts = dims.split()
        return {
            "exists": True,
            "format": magic.decode("ascii", errors="replace"),
            "width": int(parts[0]) if len(parts) >= 2 else None,
            "height": int(parts[1]) if len(parts) >= 2 else None,
            "maxval": int(maxval) if maxval.isdigit() else None,
            "bytes": path.stat().st_size,
        }
    except Exception as exc:
        return {"exists": True, "read_error": str(exc), "bytes": path.stat().st_size}


def ffprobe_video(path: Path) -> dict[str, Any]:
    info = {"path": str(path), "exists": path.exists(), "bytes": path.stat().st_size if path.exists() else 0}
    if not path.exists() or not shutil.which("ffprobe"):
        return info
    cmd = [
        "ffprobe",
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=width,height,avg_frame_rate,nb_frames,duration",
        "-of",
        "json",
        str(path),
    ]
    try:
        proc = subprocess.run(cmd, check=False, capture_output=True, text=True, timeout=15)
        if proc.returncode == 0:
            stream = (json.loads(proc.stdout).get("streams") or [{}])[0]
            info.update(stream)
        else:
            info["ffprobe_error"] = proc.stderr.strip()[-300:]
    except Exception as exc:
        info["ffprobe_error"] = str(exc)
    return info


@dataclass
class TopicStats:
    topic_id: int
    name: str
    msg_type: str
    count: int
    first_ns: int | None
    last_ns: int | None

    @property
    def span_s(self) -> float | None:
        if self.first_ns is None or self.last_ns is None or self.last_ns <= self.first_ns:
            return None
        return (self.last_ns - self.first_ns) / 1e9

    @property
    def rate_hz(self) -> float | None:
        span = self.span_s
        if not span:
            return None
        return self.count / span


def rosbag_db(day_dir: Path) -> Path | None:
    bag_dir = day_dir / "rosbags" / "harmonic_planned_rosbag"
    dbs = sorted(bag_dir.glob("*.db3"))
    return dbs[0] if dbs else None


def read_topic_stats(db_path: Path | None) -> dict[str, TopicStats]:
    if db_path is None or not db_path.exists():
        return {}
    con = sqlite3.connect(str(db_path))
    try:
        topics = {
            row[0]: (row[1], row[2])
            for row in con.execute("select id, name, type from topics").fetchall()
        }
        rows = con.execute(
            "select topic_id, count(*), min(timestamp), max(timestamp) "
            "from messages group by topic_id"
        ).fetchall()
    finally:
        con.close()
    stats: dict[str, TopicStats] = {}
    for topic_id, count, first_ns, last_ns in rows:
        name, msg_type = topics.get(topic_id, ("<unknown>", "<unknown>"))
        stats[name] = TopicStats(topic_id, name, msg_type, count, first_ns, last_ns)
    return stats


def align(offset: int, size: int, base: int = 4) -> int:
    """Align a CDR field offset.

    ROS 2 sqlite bags store a 4-byte CDR encapsulation header before the message
    stream, so field alignment is relative to byte 4, not byte 0.
    """
    remainder = (offset - base) % size
    return offset if remainder == 0 else offset + size - remainder


def parse_wrench_stamped_cdr(data: bytes) -> dict[str, Any] | None:
    try:
        offset = 4
        offset = align(offset, 4)
        sec, nsec = struct.unpack_from("<iI", data, offset)
        offset += 8
        offset = align(offset, 4)
        (length,) = struct.unpack_from("<I", data, offset)
        offset += 4
        offset += length
        offset = align(offset, 4)
        offset = align(offset, 8)
        values = struct.unpack_from("<6d", data, offset)
        fx, fy, fz, tx, ty, tz = values
        return {
            "stamp_sec": sec + nsec / 1e9,
            "force": [fx, fy, fz],
            "torque": [tx, ty, tz],
            "force_norm": math.sqrt(fx * fx + fy * fy + fz * fz),
            "wrench_norm": math.sqrt(sum(v * v for v in values)),
        }
    except Exception:
        return None


def fetch_wrench_samples(db_path: Path | None, topic: str, limit: int = 5000) -> list[dict[str, Any]]:
    if db_path is None or not db_path.exists():
        return []
    con = sqlite3.connect(str(db_path))
    try:
        row = con.execute("select id from topics where name=?", (topic,)).fetchone()
        if not row:
            return []
        topic_id = row[0]
        samples = []
        for ts, data in con.execute(
            "select timestamp, data from messages where topic_id=? order by timestamp limit ?",
            (topic_id, limit),
        ):
            parsed = parse_wrench_stamped_cdr(data)
            if parsed:
                parsed["recorded_time_ns"] = ts
                samples.append(parsed)
        return samples
    finally:
        con.close()


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    idx = (len(ordered) - 1) * pct
    lo = math.floor(idx)
    hi = math.ceil(idx)
    if lo == hi:
        return ordered[int(idx)]
    return ordered[lo] * (hi - idx) + ordered[hi] * (idx - lo)


def latency_proxy_ms(stats: dict[str, TopicStats]) -> dict[str, float | None]:
    base = stats.get(FORCE_TOPICS["target"])
    result: dict[str, float | None] = {}
    for name, topic in FORCE_TOPICS.items():
        if name == "target":
            continue
        topic_stat = stats.get(topic)
        if base and topic_stat and base.first_ns and topic_stat.first_ns:
            result[f"target_to_{name}_first_ms"] = (topic_stat.first_ns - base.first_ns) / 1e6
        else:
            result[f"target_to_{name}_first_ms"] = None
    return result


def force_loop_proxy(db_path: Path | None, stats: dict[str, TopicStats]) -> dict[str, Any]:
    samples = fetch_wrench_samples(db_path, FORCE_TOPICS["error"])
    norms = [sample["wrench_norm"] for sample in samples]
    force_norms = [sample["force_norm"] for sample in samples]
    first = norms[0] if norms else None
    final = norms[-1] if norms else None
    threshold = None
    settling_s = None
    if norms and first is not None:
        threshold = max(0.05, first * 0.1)
        timestamps = [sample["recorded_time_ns"] for sample in samples]
        first_ts = timestamps[0]
        for idx, value in enumerate(norms):
            if value <= threshold and all(v <= threshold for v in norms[idx:]):
                settling_s = (timestamps[idx] - first_ts) / 1e9
                break
    proxy = {
        "sample_count": len(norms),
        "wrench_error_norm_first": first,
        "wrench_error_norm_final": final,
        "wrench_error_norm_max": max(norms) if norms else None,
        "wrench_error_norm_mean": sum(norms) / len(norms) if norms else None,
        "force_error_norm_p95": percentile(force_norms, 0.95),
        "settling_threshold_norm": threshold,
        "settling_time_s_proxy": settling_s,
        "latency_proxy_ms": latency_proxy_ms(stats),
    }
    if not norms:
        proxy["limitation"] = "no decodable /force_control/wrench_error samples; counts and timing only"
    return proxy


def media_check(day_dir: Path, summary: dict[str, Any], stats: dict[str, TopicStats]) -> dict[str, Any]:
    duration_s = summary.get("duration_s")
    frames: dict[str, Any] = {}
    for dirname in ["camera_frames", "left_camera_frames"]:
        folder = day_dir / dirname
        files = sorted(folder.glob("*.ppm")) if folder.exists() else []
        frame_info = {
            "path": str(folder),
            "count": len(files),
            "first": parse_ppm_header(files[0]) if files else {"exists": False},
            "last": parse_ppm_header(files[-1]) if files else {"exists": False},
        }
        if duration_s:
            frame_info["fps_vs_moveit_duration"] = len(files) / float(duration_s)
        frames[dirname] = frame_info
    videos = [ffprobe_video(path) for path in sorted((day_dir / "videos").glob("*.mp4"))]
    screenshots = [
        {"path": str(path), "bytes": path.stat().st_size, "exists": path.exists()}
        for path in sorted((day_dir / "screenshots").glob("*.png"))
    ]
    camera_rates = {}
    for topic in CAMERA_TOPICS:
        topic_stat = stats.get(topic)
        camera_rates[topic] = {
            "message_count": topic_stat.count if topic_stat else 0,
            "rate_hz": topic_stat.rate_hz if topic_stat else None,
            "span_s": topic_stat.span_s if topic_stat else None,
        }
    return {
        "camera_topic_rates": camera_rates,
        "frames": frames,
        "videos": videos,
        "screenshots": screenshots,
        "media_ok": bool(videos and screenshots and all(v["bytes"] > 0 for v in videos) and all(s["bytes"] > 0 for s in screenshots)),
    }


def summarize_day(day_dir: Path) -> dict[str, Any]:
    day_id = day_dir.name
    logs_dir = day_dir / "logs"
    summary = read_json(logs_dir / "dual_moveit_summary.json")
    db_path = rosbag_db(day_dir)
    stats = read_topic_stats(db_path)
    topic_counts = {name: stat.count for name, stat in sorted(stats.items())}
    trajectory_topic_counts = {
        topic: topic_counts.get(topic, 0)
        for topic in TRAJECTORY_TOPICS
    }
    moveit_segments = summary.get("segments") if isinstance(summary.get("segments"), list) else []
    planning_times = [float(seg.get("planning_time", 0.0)) for seg in moveit_segments if isinstance(seg, dict)]
    day = {
        "day_id": day_id,
        "day_dir": str(day_dir),
        "rosbag_db": str(db_path) if db_path else None,
        "topic_counts": topic_counts,
        "topic_count_total": sum(topic_counts.values()),
        "topic_count_unique": len(topic_counts),
        "camera": media_check(day_dir, summary, stats),
        "force_loop": force_loop_proxy(db_path, stats),
        "trajectory": {
            "combined_points": summary.get("combined_points"),
            "duration_s": summary.get("duration_s"),
            "segments": moveit_segments,
            "trajectory_topic_counts": trajectory_topic_counts,
        },
        "latency_proxy": {
            "moveit_planning_total_ms": sum(planning_times) * 1000.0 if planning_times else None,
            "moveit_planning_max_ms": max(planning_times) * 1000.0 if planning_times else None,
            **latency_proxy_ms(stats),
        },
        "log_status": parse_key_value_file(logs_dir / "status.txt"),
    }
    return day


def flatten_for_csv(day: dict[str, Any]) -> dict[str, Any]:
    camera = day["camera"]["camera_topic_rates"]
    force = day["force_loop"]
    latency = day["latency_proxy"]
    row = {
        "day_id": day["day_id"],
        "topic_count_total": day["topic_count_total"],
        "topic_count_unique": day["topic_count_unique"],
        "combined_points": day["trajectory"].get("combined_points"),
        "duration_s": day["trajectory"].get("duration_s"),
        "left_camera_image_count": day["topic_counts"].get("/left_camera/image_rect", 0),
        "left_camera_image_rate_hz": camera.get("/left_camera/image_rect", {}).get("rate_hz"),
        "evidence_camera_image_count": day["topic_counts"].get("/rm65b/evidence_camera/image", 0),
        "evidence_camera_image_rate_hz": camera.get("/rm65b/evidence_camera/image", {}).get("rate_hz"),
        "video_count": len(day["camera"]["videos"]),
        "screenshot_count": len(day["camera"]["screenshots"]),
        "media_ok": day["camera"]["media_ok"],
        "force_error_samples": force.get("sample_count"),
        "force_error_norm_mean": force.get("wrench_error_norm_mean"),
        "force_error_norm_final": force.get("wrench_error_norm_final"),
        "settling_time_s_proxy": force.get("settling_time_s_proxy"),
        "moveit_planning_total_ms": latency.get("moveit_planning_total_ms"),
        "target_to_error_first_ms": latency.get("target_to_error_first_ms"),
        "target_to_offset_first_ms": latency.get("target_to_offset_first_ms"),
        "target_to_corrected_first_ms": latency.get("target_to_corrected_first_ms"),
    }
    return row


def write_reports(output_root: Path, report: dict[str, Any]) -> tuple[Path, Path]:
    out_dir = output_root / "offline_analysis"
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / "daily_evidence_report.json"
    csv_path = out_dir / "daily_evidence_report.csv"
    json_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    rows = [flatten_for_csv(day) for day in report["days"]]
    fieldnames = list(rows[0].keys()) if rows else []
    with csv_path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return json_path, csv_path


def build_report(output_root: Path) -> dict[str, Any]:
    days = []
    for day_dir in sorted(output_root.iterdir()):
        if day_dir.is_dir() and DAY_RE.match(day_dir.name):
            days.append(summarize_day(day_dir))
    if not days:
        raise FileNotFoundError(f"no day01-day05 directories under {output_root}")
    return {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "output_root": str(output_root),
        "analysis_notes": [
            "camera fps is derived from rosbag record timestamps and extracted frame counts",
            "force-loop settling is a proxy from /force_control/wrench_error CDR samples when decodable",
            "latency is a first-message timing proxy between recorded topics, not a hardware timestamp measurement",
        ],
        "days": days,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, help="D1-D5 output root; defaults to latest under ./outputs")
    parser.add_argument("--outputs-dir", type=Path, default=None, help="outputs directory used for latest discovery")
    args = parser.parse_args()

    root = repo_root_from_script()
    outputs_dir = args.outputs_dir or (root / "outputs")
    output_root = args.output_root or latest_output_root(outputs_dir)
    output_root = output_root.resolve()
    report = build_report(output_root)
    json_path, csv_path = write_reports(output_root, report)
    print(f"output_root={output_root}")
    print(f"json={json_path}")
    print(f"csv={csv_path}")
    for day in report["days"]:
        row = flatten_for_csv(day)
        print(
            f"{row['day_id']}: topics={row['topic_count_unique']} "
            f"msgs={row['topic_count_total']} left_camera_hz={row['left_camera_image_rate_hz']} "
            f"points={row['combined_points']} media_ok={row['media_ok']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
