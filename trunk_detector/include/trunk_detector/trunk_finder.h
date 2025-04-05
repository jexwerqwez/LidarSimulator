/*  The trunk_finder package
**  TrunkFinder class header file
**  Created on February 8 2018
*/

#ifndef TRUNK_FINDER_H
#define TRUNK_FINDER_H

#include <map>
#include <math.h>
#include <sstream>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "trunk_detector_msgs/msg/trunk_pose.hpp"
#include "trunk_detector_msgs/msg/trunk_pose_array.hpp"

typedef pcl::PointXYZI PointT;

// Класс детектора вертикальных препятствий
class TrunkFinder {
public:
  // конструктор по умолчанию
  TrunkFinder();
  // деструктор
  ~TrunkFinder();
  // выполнение анализа облака
  std::string analyze(pcl::PointCloud<PointT>::Ptr source_cloud,
                        pcl::PointCloud<PointT>::Ptr output_cloud,
                        std::map<int, std::vector<int> >* found_points_number);
  // установка параметров
  std::string setParameters(std::map<std::string, double>* param);
  
protected:
  double empty_radius;     // радиус зоны фильтрации в центра облака
  double v_delta;          // высота слоя
  double v_shift;          // величина сдвига начальной точки (высота робота)
  double cell_size;        // размер ячейки при понижении плотности
  double cluster_tolerance;// допуск при сегментации
  double cluster_max_size; // максимальный размер кластера при сегментации
  double cluster_min_size; // минимальный размер кластера при сегментации
  double max_angle;        // максимальный допустимый угол отклонения от вертикали
  int slices_number;       // число горизонтальных слоев
  PointT zero;             // центральная точка облака
  pcl::PointCloud<PointT>::Ptr filtered_cloud;                // указатель на облако с удаленной центральной частью
  pcl::PointCloud<PointT>::Ptr downsampled_cloud;             // указатель на облако с уменьшенной плотностью точек
  std::vector<pcl::PointCloud<PointT>::Ptr> slices;           // вектор с указателями на облака с горизонтальными слоями
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters; // вектор с указателями на облака с центрами кластеров
  pcl::PassThrough<PointT> filter;                            // фильтр для "нарезания" облака
  pcl::VoxelGrid<PointT> downsampler;                         // фильтр для уменьшения плотности точек облака
  pcl::ExtractIndices<PointT> indices_extractor;              // фильтр облака по индексам точек
  pcl::EuclideanClusterExtraction<PointT> cluster_extractor;  // кластеризация по евклидовому расстоянию между точками
  pcl::search::KdTree<PointT>::Ptr source_kd_tree;            // kd-дерево исходного облака
  pcl::search::KdTree<PointT>::Ptr kd_tree;                   // kd-дерево слоев
};

#endif // TRUNK_FINDER_H
