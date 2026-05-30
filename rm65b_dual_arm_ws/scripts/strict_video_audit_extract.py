#!/usr/bin/env python3
from __future__ import annotations

import json
import subprocess
from pathlib import Path


ROOT = Path("/mnt/e/rm65b_strict_audit_20260529")
DAYS = ("day01", "day02", "day03", "day04", "day05")
VIDEOS = (
    "moveit_harmonic_playback",
    "rviz",
    "left_gazebo_camera",
    "right_gazebo_camera",
)


def run(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)


def duration(path: Path) -> float:
    proc = run(
        [
            "ffprobe",
            "-v",
            "error",
            "-show_entries",
            "format=duration",
            "-of",
            "default=nw=1:nk=1",
            str(path),
        ]
    )
    try:
        return float(proc.stdout.strip())
    except ValueError:
        return 0.0


def extract(video: Path, out: Path, second: float) -> bool:
    out.parent.mkdir(parents=True, exist_ok=True)
    proc = run(
        [
            "ffmpeg",
            "-y",
            "-ss",
            f"{second:.3f}",
            "-i",
            str(video),
            "-frames:v",
            "1",
            str(out),
        ]
    )
    return proc.returncode == 0 and out.exists() and out.stat().st_size > 0


def main() -> int:
    audit_frames = ROOT / "audit_frames"
    rows: list[dict] = []
    for day in DAYS:
        for kind in VIDEOS:
            name = f"{day}_{kind}.mp4"
            video = ROOT / day / "videos" / name
            if not video.exists():
                rows.append({"day": day, "video": name, "exists": False})
                continue
            dur = duration(video)
            if kind in {"moveit_harmonic_playback", "rviz"}:
                points = (0.12, 0.35, 0.55, 0.78)
            else:
                points = (0.50,)
            outputs = []
            for frac in points:
                second = max(0.0, min(dur - 0.2, dur * frac)) if dur > 0.2 else 0.0
                out = audit_frames / day / f"{kind}_f{int(frac * 100):02d}.png"
                ok = extract(video, out, second)
                outputs.append({"fraction": frac, "second": round(second, 3), "frame": str(out), "ok": ok})
            rows.append(
                {
                    "day": day,
                    "video": name,
                    "exists": True,
                    "duration_s": round(dur, 3),
                    "size": video.stat().st_size,
                    "frames": outputs,
                }
            )
    (ROOT / "strict_audit_manifest.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    print(json.dumps(rows, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
