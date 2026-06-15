# navigation_ros

## Описание

Пакет реализует ROS 2-узел **`waypoint_nav`**: приём GPS и IMU, оценка позы, следование по waypoints, публикация `/cmd_vel` и маркеров RViz.

## Функции узла waypoint_nav


| Этап | Реализация |
|------|------------|
| Приём данных | Подписка на `/gps/fix`, `/imu/data` |
| Оценка позы | `GpsImuPoseEstimator`: XY из GPS (фильтр), курс из IMU (+ `yaw_noise_stddev_rad`), ω_z из IMU |
| Управление | Конечный автомат прохода waypoints |
| Выход | `/cmd_vel`, `/waypoint_markers` |

## Конфигурационные файлы


| Файл | Содержание |
|------|------------|
| config/nav_params.yaml | Топики, регуляторы, пороги, фильтры, логирование |
| config/waypoints.json | Маршрут по умолчанию |

Параметр `waypoint_file` задаётся в `thrust_nav_sim/launch/sim.launch.py`.

## Конечный автомат

```text
TURN_INIT → TURNING → SETTLE_BEFORE_DRIVE → DRIVING → SETTLE_AFTER_DRIVE
```


| Состояние | Действие |
|-----------|----------|
| TURNING | Разворот на пеленг (angular.z) |
| SETTLE | Пауза после манёвра: |ω_z| < ε (и минимальное время) |
| DRIVING | Прямой ход с удержанием курса yaw_hold (linear.x, при необходимости angular.z) |
| Коррекция | Остановка и повторный разворот при ошибке пеленга > heading_realign_rad |

## Визуализация

Топик `/waypoint_markers`, частота 5 Гц, кадр `map`.

Реализованные маркеры: точки маршрута, радиусы достижения, стрелки направления по сегментам (`waypoints_route_arrow`), поза робота, пройденная траектория (`robot_trajectory_line`). Конфигурация RViz: `thrust_nav_sim/rviz/waypoint_nav.rviz`.

## Параметры (config/nav_params.yaml)


| Группа | Параметры |
|--------|-----------|
| Топики и кадр | gps_topic, imu_topic, cmd_vel_topic, frame_id |
| Датчики | gps_filter_alpha, yaw_noise_stddev_rad, gps_timeout_sec, imu_timeout_sec |
| Управление | turn_kp, forward_gain, drive_heading_kp, control_hz |
| Пороги | yaw_tolerance_rad, heading_realign_rad, default_reach_radius_m |
| Стабилизация | settle_time_sec, settle_w_epsilon_rps, settle_v_epsilon_mps |
| Логирование | debug_log_hz, course_check_period_sec |

## Логирование

Узел ведёт журнал ROS 2 (`get_logger()`). Параметр **`debug_log_hz`** задаёт частоту периодических записей (по умолчанию 1 Гц; `0` — только событийные сообщения).


| Категория | Префикс | Содержание |
|-----------|---------|------------|
| Инициализация | при старте | Топики, параметры FSM |
| Маршрут | при загрузке | Waypoints, координаты, радиусы |
| Состояние | НАВ | Поза, FSM, дистанция, ошибка курса |
| Поворот | ПОВОРОТ_* | Разворот, команды angular.z |
| Прямой ход | ПРЯМОЙ_* | Движение, команды linear.x / angular.z |
| Курс | КУРС_ПРОВЕРКА | Периодическая проверка курса |
| Коррекция | КУРС_КОРРЕКЦИЯ | Повтор подхода к точке |
| Достижение | Достигнута, ПРОВЕРКА_ДОСТИГНУТО, ПРОВЕРКА_МИМО | Переход к следующему waypoint или повтор подхода |
| Предупреждение | warning | Отсутствие GPS или IMU |

Записи `КУРС_ПРОВЕРКА` формируются таймером `course_check_period_sec` (5 с).

## Запуск узла

```bash
ros2 run navigation_ros waypoint_nav --ros-args \
  -p waypoint_file:=/path/to/waypoints.json \
  --params-file $(ros2 pkg prefix navigation_ros)/share/navigation_ros/config/nav_params.yaml
```
