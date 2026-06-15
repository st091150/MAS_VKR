# Система навигации гусеничного робота по GPS и IMU в среде Gazebo

## Описание системы

Программный проект разработан в рамках выпускной квалификационной работы. Реализовано **автономное следование гусеничного мобильного робота по заданному маршруту** в симуляторе **Gazebo Harmonic** на платформе **ROS 2 Jazzy**.

Исходные данные: **GPS** (`/gps/fix`, `sensor_msgs/NavSatFix`) и **IMU** (`/imu/data`, `sensor_msgs/Imu`).  
Выходные команды: **/cmd_vel** (`geometry_msgs/Twist`) — линейная и угловая скорость.

## Состав программного обеспечения

```text
gazebo/
└── src/
    ├── navigation_module/   # алгоритмы навигации
    ├── navigation_ros/      # ROS-узел waypoint_nav
    └── thrust_nav_sim/      # среда Gazebo
```


| Пакет | Роль в системе |
|-------|----------------|
| navigation_module | Преобразование координат, загрузка маршрута, расчёт команд движения |
| navigation_ros | Узел waypoint_nav: оценка позы, следование по waypoints, маркеры RViz |
| thrust_nav_sim | Модель робота и мира Gazebo, мост топиков, launch |

Подробное описание пакетов:

- `src/navigation_module/README.md`
- `src/navigation_ros/README.md`
- `src/thrust_nav_sim/README.md`

## Архитектура

```text
Gazebo (/gps, /imu)  →  ros_gz_bridge  →  /gps/fix, /imu/data
                                                    ↓
                                              waypoint_nav
                                                    ↓
                                         /cmd_vel  →  Gazebo (гусеницы)
                                                    ↓
                                         /waypoint_markers  →  RViz
```

Мост топиков: `src/thrust_nav_sim/config/bridge.yaml`.


| Топик ROS | Топик Gazebo | Тип сообщения |
|-----------|--------------|---------------|
| /gps/fix | /gps | sensor_msgs/NavSatFix |
| /imu/data | /imu | sensor_msgs/Imu |
| /cmd_vel | /model/thrust_robot/cmd_vel | geometry_msgs/Twist |
| /waypoint_markers | - | visualization_msgs/MarkerArray |

Конфигурация навигации и шум курса: `src/navigation_ros/config/nav_params.yaml`.  
Модели GPS и IMU в SDF: `src/thrust_nav_sim/sdf/thrust_robot.sdf`.

## Программное обеспечение

- ROS 2 Jazzy;
- Gazebo Harmonic (`ros_gz_sim`, `ros_gz_bridge`);
- Python 3.12.

## Логирование

При работе симуляции формируются записи:

1. Узел **waypoint_nav** — журнал ROS 2 (уровни `info`, `warning`); частота периодических сообщений — параметр `debug_log_hz` в `nav_params.yaml`.
2. **sim.launch.py**, **spawn_when_world_ready**, **generate_gazebo_world** — служебные сообщения в stdout при старте.

Категории сообщений навигации описаны в `src/navigation_ros/README.md`.

## Маршрут

Файл по умолчанию: `src/navigation_ros/config/waypoints.json`.

```json
{
  "origin": { "latitude": 55.0, "longitude": 37.0, "yaw": 0.0 },
  "default_reach_radius_m": 0.9,
  "waypoints": [
    { "id": "wp_1", "latitude": 55.0, "longitude": 37.0002, "reach_radius_m": 0.9 }
  ]
}
```

Поле `origin` задаёт привязку кадра `map` и GPS-origin мира Gazebo (подстановка в SDF при запуске `sim.launch.py`).

## Сборка и запуск

```bash
source /opt/ros/jazzy/setup.bash
cd <workspace>/gazebo
colcon build --symlink-install
source install/setup.bash
ros2 launch thrust_nav_sim sim.launch.py
```

Аргументы: `waypoint_file:=...`, `enable_rviz:=false`.

## Верификация

```bash
source install/setup.bash
pytest src/navigation_module/tests src/navigation_ros/tests src/thrust_nav_sim/tests -q
```
