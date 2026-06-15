#!/usr/bin/env python3
"""Ждёт появления моделей waypoints в Gazebo, затем ros_gz_sim create.

Ожидаемое число моделей: ground (1) + N точек + max(0, N−1) сегментов-маршрутов = 2N при N≥1.
Если команда `gz model …` недоступна или таймаут — после предупреждения всё равно пробуем create.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import shlex
import subprocess
import sys
import time
from pathlib import Path


def _count_waypoints(path: Path) -> int:
    raw = json.loads(path.read_text(encoding="utf-8"))
    pts = raw.get("waypoints", [])
    if not isinstance(pts, list):
        return 0
    return len(pts)


def _expected_model_count(n_wp: int) -> int:
    if n_wp <= 0:
        return 1
    return 1 + n_wp + max(0, n_wp - 1)


def _parse_model_lines(text: str) -> list[str]:
    """Вытащить имена моделей из смешанного вывода gz / ign."""
    lines = [ln.rstrip() for ln in text.splitlines()]
    names: list[str] = []
    # Блок после строки «Model names:» / «Models:»
    start = 0
    for i, ln in enumerate(lines):
        low = ln.lower().strip()
        if "model" in low and ("name" in low or low.endswith(":")):
            start = i + 1
            break
    chunk = lines[start:] if start else lines
    header_words = frozenset(
        {
            "available",
            "models",
            "model",
            "names",
            "requesting",
            "world",
            "---",
            "",
        }
    )
    for line in chunk:
        s = line.strip()
        if not s or s.startswith("WARNING") or s.startswith("ERROR") or s.startswith("["):
            continue
        if s.lower() in header_words:
            continue
        if "requesting state" in s.lower():
            continue
        m = re.match(r"^[\-\*]\s*(.+)$", s)
        if m:
            names.append(m.group(1).strip())
            continue
        # Имя модели: буквы, цифры, _, -
        if re.match(r"^[\w\-]+$", s):
            names.append(s)
            continue
    # Уникальные, порядок сохраняем
    seen: set[str] = set()
    out: list[str] = []
    for n in names:
        if n not in seen:
            seen.add(n)
            out.append(n)
    return out


def _gz_model_list(world: str) -> list[str]:
    candidates = (
        ["gz", "model", "--list", "--world", world],
        ["gz", "model", "-l", "-w", world],
        ["gz", "model", "list", "--world", world],
    )
    for cmd in candidates:
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
            out = (r.stdout or "") + "\n" + (r.stderr or "")
            names = _parse_model_lines(out)
            if names:
                return names
            if r.returncode == 0 and out.strip():
                # Запасной подсчёт непустых строк без явных заголовков
                fallback = [
                    ln.strip()
                    for ln in out.splitlines()
                    if ln.strip()
                    and not ln.strip().lower().startswith("warning")
                    and "requesting" not in ln.lower()
                ]
                if fallback:
                    return fallback
        except (FileNotFoundError, subprocess.TimeoutExpired):
            continue
    return []


def _run_create(world: str, robot_name: str, sdf: Path) -> int:
    sdf_s = shlex.quote(str(sdf.resolve()))
    inner = (
        f"ros2 run ros_gz_sim create --ros-args -p use_sim_time:=true -- "
        f"-world {shlex.quote(world)} -name {shlex.quote(robot_name)} "
        f"-file {sdf_s} -z 0.0 -Y 0"
    )
    if sys.platform == "win32":
        cmd = ["cmd", "/c", inner]
    else:
        cmd = ["bash", "-lc", inner]
    print(f"[spawn_when_world_ready] Запуск: {inner}", flush=True)
    return subprocess.call(cmd)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--world", default="thrust_world")
    ap.add_argument("--robot-name", default="thrust_robot")
    ap.add_argument("--robot-sdf", type=Path, required=True)
    ap.add_argument("--waypoint-file", type=Path, required=True)
    ap.add_argument("--grace", type=float, default=2.0, help="Пауза перед первым опросом Gazebo")
    ap.add_argument("--poll", type=float, default=0.75, help="Период опроса списка моделей")
    ap.add_argument("--timeout", type=float, default=240.0)
    ap.add_argument(
        "--legacy-sleep",
        type=float,
        default=-1.0,
        help="Если >= 0 — не ждать модели, только sleep (сек) и create",
    )
    ap.add_argument(
        "--stable-reads",
        type=int,
        default=2,
        help="Сколько подряд успешных опросов с достаточным числом моделей",
    )
    args = ap.parse_args()

    wp_path = args.waypoint_file.expanduser()
    if not wp_path.is_file():
        print(f"[spawn_when_world_ready] Нет файла: {wp_path}", file=sys.stderr, flush=True)
        return 1

    sdf_path = args.robot_sdf.expanduser()
    if not sdf_path.is_file():
        print(f"[spawn_when_world_ready] Нет SDF робота: {sdf_path}", file=sys.stderr, flush=True)
        return 1

    if args.legacy_sleep >= 0.0:
        print(
            f"[spawn_when_world_ready] Режим фиксированной задержки: {args.legacy_sleep} с",
            flush=True,
        )
        time.sleep(args.legacy_sleep)
        return _run_create(args.world, args.robot_name, sdf_path)

    n_wp = _count_waypoints(wp_path)
    need = _expected_model_count(n_wp)
    print(
        f"[spawn_when_world_ready] waypoints в файле: {n_wp}, ожидаем моделей в мире (мин.): {need}",
        flush=True,
    )

    time.sleep(max(0.0, args.grace))
    deadline = time.monotonic() + max(10.0, args.timeout)
    stable = 0
    last_count = -1

    while time.monotonic() < deadline:
        models = _gz_model_list(args.world)
        cnt = len(models)
        if cnt != last_count:
            print(f"[spawn_when_world_ready] Моделей в '{args.world}': {cnt} (нужно ≥ {need})", flush=True)
            last_count = cnt
        if cnt >= need:
            stable += 1
            if stable >= max(1, args.stable_reads):
                print("[spawn_when_world_ready] Условие выполнено — спавн робота.", flush=True)
                return _run_create(args.world, args.robot_name, sdf_path)
        else:
            stable = 0
        time.sleep(max(0.2, args.poll))

    print(
        "[spawn_when_world_ready] Таймаут ожидания моделей — пробуем спавн всё равно "
        "(проверьте `gz model --list`).",
        file=sys.stderr,
        flush=True,
    )
    extra = min(30.0, 5.0 + math.sqrt(max(0, n_wp)) * 0.5)
    print(f"[spawn_when_world_ready] Доп. пауза {extra:.1f} с перед create.", flush=True)
    time.sleep(extra)
    return _run_create(args.world, args.robot_name, sdf_path)


if __name__ == "__main__":
    sys.exit(main())
