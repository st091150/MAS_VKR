# thrust_nav_sim

## Описание

Пакет реализует среду **Gazebo** для навигации гусеничного робота: SDF-модель робота и мира, мост топиков Gazebo–ROS 2, launch-файл запуска, конфигурация RViz.

## Состав пакета


| Компонент | Путь | Реализуемая функция |
|-----------|------|-------------------|
| Мир | worlds/thrust_world.sdf | Плоская сцена, физика, имя мира thrust_world |
| Модель робота | sdf/thrust_robot.sdf | Гусеничная платформа, GPS (NavSat), IMU |
| Мост топиков | config/bridge.yaml | Сопоставление топиков Gazebo и ROS 2 |
| Launch | launch/sim.launch.py | Запуск Gazebo, робота, моста, waypoint_nav, RViz |
| RViz | rviz/waypoint_nav.rviz | Отображение маршрута и позы робота |

## Мост топиков


| Топик ROS | Топик Gazebo | Направление |
|-----------|--------------|-------------|
| /gps/fix | /gps | Gazebo → ROS |
| /imu/data | /imu | Gazebo → ROS |
| /cmd_vel | /model/thrust_robot/cmd_vel | ROS → Gazebo |
| /clock | /clock | Gazebo → ROS |

## Модели GPS и IMU

Параметры сенсоров и гауссовых шумов заданы в **`sdf/thrust_robot.sdf`**. Шум курса и фильтрация GPS — в **`navigation_ros/config/nav_params.yaml`**.

СКО — стандартное отклонение гауссова шума: типичный разброс ошибки измерения.

### GPS (NavSat)

Топик Gazebo: `/gps`, 10 Гц. Публикация в ROS: `/gps/fix`.


| Измерение | СКО (σ) | Использование в waypoint_nav |
|-----------|---------|------------------------------|
| Положение, горизонталь (широта и долгота) | 0.0000054° (~0.6 м) | широта/долгота → XY в кадре map |

### IMU

Топик Gazebo: `/imu`, 50 Гц. Публикация в ROS: `/imu/data`.


| Измерение | СКО (σ) | Использование в waypoint_nav |
|-----------|---------|------------------------------|
| Угловая скорость, ось X | 0.0004 рад/с | — |
| Угловая скорость, ось Y | 0.0004 рад/с | — |
| Угловая скорость, ось Z | 0.0008 рад/с | ω_z, стабилизация после поворота |
| Линейное ускорение, ось X | 0.03 м/с² | — |
| Линейное ускорение, ось Y | 0.03 м/с² | — |
| Курс (`yaw_noise_stddev_rad`, nav_params) | 0.02 рад (~1.1°) | курс из IMU, разворот и удержание направления |

Фильтрация GPS: параметр `gps_filter_alpha` в `navigation_ros/config/nav_params.yaml`.

## Согласование GPS-origin с маршрутом

Функция `generate_gazebo_world()` при запуске формирует SDF мира с блоком `spherical_coordinates` по полю `origin` из JSON-маршрута. Обеспечивается совпадение координат GPS и waypoints в кадре `map`.

## Последовательность запуска

```text
Gazebo (мир) → spawn робота → ros_gz_bridge → waypoint_nav (+ RViz)
```

Задержки `bridge_delay_s` и `waypoint_nav_delay_s` задаются в `launch/sim.launch.py`.

## Запуск

```bash
ros2 launch thrust_nav_sim sim.launch.py
```


| Аргумент | По умолчанию | Описание |
|----------|--------------|----------|
| waypoint_file | navigation_ros/config/waypoints.json | Файл маршрута |
| enable_rviz | true | Запуск RViz |
| waypoint_nav_delay_s | 5.0 | Задержка запуска waypoint_nav, с |
| bridge_delay_s | 1.0 | Задержка запуска моста после spawn, с |

## Служебный вывод при запуске

| Источник | Содержание |
|----------|------------|
| sim.launch.py | Путь к сгенерированному SDF мира |
| generate_gazebo_world | Координаты waypoints, длины сегментов |
| spawn_when_world_ready | Готовность мира, команда spawn робота |

Журнал движения робота — узел `waypoint_nav` (`navigation_ros/README.md`).
