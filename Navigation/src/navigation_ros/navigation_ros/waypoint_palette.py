"""Палитра из 10 цветов для waypoints: цикл по индексу (порядок в файле)."""

from __future__ import annotations

# Яркие отличаемые оттенки (RGB 0…1), один цвет на индекс i % 10.
WAYPOINT_PALETTE_RGB: tuple[tuple[float, float, float], ...] = (
    (0.90, 0.22, 0.18),  # красный
    (0.98, 0.52, 0.12),  # оранжевый
    (0.95, 0.82, 0.15),  # жёлтый
    (0.55, 0.88, 0.22),  # лайм
    (0.18, 0.78, 0.42),  # зелёный
    (0.16, 0.72, 0.82),  # бирюзовый
    (0.22, 0.42, 0.95),  # синий
    (0.52, 0.28, 0.92),  # фиолетовый
    (0.88, 0.22, 0.72),  # пурпурный
    (0.82, 0.38, 0.55),  # розовый
)


def rgb_for_index(i: int) -> tuple[float, float, float]:
    return WAYPOINT_PALETTE_RGB[i % len(WAYPOINT_PALETTE_RGB)]


def gazebo_point_radius_rgba(i: int) -> tuple[str, str]:
    """Пара строк r g b a для Gazebo: сфера точки и полупрозрачный диск радиуса."""
    r, g, b = rgb_for_index(i)
    return (
        f"{r:.4f} {g:.4f} {b:.4f} 1.0",
        f"{r:.4f} {g:.4f} {b:.4f} 0.33",
    )
