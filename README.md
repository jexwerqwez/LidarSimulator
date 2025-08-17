# LidarSimulator

Проект посвящен разработке легковесного симулятора лазерного дальномера (лидара) и набора алгоритмов обработки облаков точек, предназначенных для ипользования в ситемах компьютерного зрения мобильных роботов. Реализован следующий функционал:
- математическая модель сканирования сцены с параметризованными лазерными лучами;
- программная архитектура симулятора с поддержкой многосенсорной конфигурации;
- модуль объединения облаков точек с воксельной фильтрацией и удалением плоскости пола;
- алгоритм классификации препятствий на основе регулярной сетки и анализа распределения высот;
- интеграция всех компонентов в ROS2, визуализация в RViz, тестирование системы.

## Используемые технологии:
- C++;
- ROS2;
- Point Cloud Library (PCL);
- RViz.

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
