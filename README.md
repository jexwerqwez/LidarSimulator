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

**Терминал №1**

Сборка:
```
colcon build --packages-select lidar_simulation
```

Публикация:

```
ros2 run lidar_simulation lidar_simulation_node --ros-args -p num_circles:=10 -p lidar_height:=1.0
```


**Терминал №2**

Запуск rviz2:

```
rviz2
```
