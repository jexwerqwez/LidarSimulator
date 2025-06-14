# LidarSimulator


# Установка

## Источники

**ROS2**
```
source /opt/ros/galactic/setup.bash
```

**install**
```
source install/setup.bash
```

## Сборка и запуск

Сборка:
```
colcon build
```

Публикация:

```
ros2 launch lidar_simulation lidar_with_trunk_detector.launch.py
```

## Управление

В отдельном терминале запустить:

```
ros2 topic pub /lidar_control geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}}'
```

## Запуск тестов

```
colcon test --packages-select lidar_simulation
```

```
colcon test-result --verbose
./build/lidar_simulation/tests --gtest_filter=* --gtest_color=yes --gtest_repeat=1 --gtest_break_on_failure
```