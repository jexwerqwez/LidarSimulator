/*  The trunk_finder package
**  TrunkFinder class cpp file
**  Created on February 8 2018
*/

#include "trunk_detector/trunk_finder.h"

using namespace pcl;
using namespace std;

// конструктор по умолчанию;
TrunkFinder::TrunkFinder() {
  empty_radius = 5;        // для обнаружения снежных отвалов
  v_delta = 0.75;
  v_shift = 1.9;
  cell_size = 0.1;
  cluster_tolerance = 0.9;
  cluster_max_size = 200;
  cluster_min_size = 1;
  max_angle = 0.7;
  slices_number = 4;
  zero.x = 0; zero.y = 0; zero.z = 0;
  
  filtered_cloud = PointCloud<PointT>::Ptr(new PointCloud<PointT>);
  downsampled_cloud = PointCloud<PointT>::Ptr(new PointCloud<PointT>);
  slices = vector<PointCloud<PointT>::Ptr>(slices_number);
  clusters = vector<PointCloud<PointT>::Ptr>(slices_number);
  source_kd_tree = pcl::search::KdTree<PointT>::Ptr(new search::KdTree<PointT>);
  kd_tree = pcl::search::KdTree<PointT>::Ptr(new search::KdTree<PointT>);
  indices_extractor.setNegative(true);
  
  for (int i = 0; i < slices.size(); i++) {
    slices[i] = PointCloud<PointT>::Ptr(new PointCloud<PointT>);
    clusters[i] = PointCloud<PointT>::Ptr(new PointCloud<PointT>);
  }
  
  cluster_extractor.setClusterTolerance(cluster_tolerance);
  cluster_extractor.setMinClusterSize(cluster_min_size);
  cluster_extractor.setMaxClusterSize(cluster_max_size);
  downsampler.setLeafSize(cell_size, cell_size, v_delta / 2.0);
}

// деструктор;
TrunkFinder::~TrunkFinder() {
}

// выполнение анализа облака;
std::string TrunkFinder::analyze(PointCloud<PointT>::Ptr source_cloud,
                                 PointCloud<PointT>::Ptr output_cloud,
                                 std::map<int, std::vector<int> >* found_points_number) {
  stringstream err;
  if (source_cloud->points.size() == 0) {
    err << "zero size of source cloud;";
    return err.str();
  }
  int K = 3; // число ближайших соседей для центров кластера
  IndicesPtr point_idx_source(new vector<int>);
  vector<int> point_idx(K);
  vector<float> point_nkn_sd;
  source_kd_tree->setInputCloud(source_cloud);

  // удаление центральной области внутри радиуса empty_radius
  if (source_kd_tree->radiusSearch(zero, empty_radius, *point_idx_source, point_nkn_sd) > 0) {
    indices_extractor.setInputCloud(source_cloud);
    indices_extractor.setIndices(point_idx_source);
    indices_extractor.filter(*filtered_cloud);
    *source_cloud = *filtered_cloud;
  } else {
    filtered_cloud = source_cloud;
  }

  // "нарезание" облака
  filter.setInputCloud(filtered_cloud);
  filter.setFilterFieldName("z");
  for (int i = 0; i < slices.size(); i++) {
    filter.setFilterLimits(-v_shift + ((double)i - 0.5)*v_delta, -v_shift + ((double)i + 0.5)*v_delta);
    filter.filter(*downsampled_cloud);
    // понижение плотности точек
    downsampler.setInputCloud(downsampled_cloud);
    downsampler.filter(*(slices[i]));
  }

  // кластеризация точек по их близости
  for (int i = 1; i < slices.size(); i++) {
    clusters[i]->clear();
    if (slices[i]->points.size() == 0) {
      err << "zero slice " << i << " size; ";
      continue;
    }
    vector<PointIndices> indices;
    cluster_extractor.setInputCloud(slices[i]);
    kd_tree->setInputCloud(slices[i]);
    cluster_extractor.setSearchMethod(kd_tree);
    cluster_extractor.extract(indices);

    // получение центров кластеров и группирование их в облака
    for (auto it = indices.begin(); it != indices.end(); ++it) {
      PointT center;
      double min_x = 1E100, min_y = 1E100, max_x = -1E100, max_y = -1E100;
      center.x = 0; center.y = 0; center.z = 0; center.intensity = 0;
      for (auto p_it = it->indices.begin(); p_it != it->indices.end(); ++p_it) {
        center.x += slices[i]->points[*p_it].x;
        center.y += slices[i]->points[*p_it].y;
        // определение размера кластера
        if (it->indices.size() > 1) {
          if (slices[i]->points[*p_it].x < min_x)
            min_x = slices[i]->points[*p_it].x;
          if (slices[i]->points[*p_it].x > max_x)
            max_x = slices[i]->points[*p_it].x;
          if (slices[i]->points[*p_it].y < min_y)
            min_y = slices[i]->points[*p_it].y;
          if (slices[i]->points[*p_it].y > max_y)
            max_y = slices[i]->points[*p_it].y;
          if (max_x - min_x > max_y - min_y)
            center.intensity = max_x - min_x;
          else
            center.intensity = max_y - min_y;
        }
      }
      center.x /= it->indices.size();
      center.y /= it->indices.size();
      center.z = -v_shift + ((double)i + 1.5)*v_delta;

      clusters[i]->push_back(center);
    }
  }

  // поиск ближайших точек в соседнем слое
  int count = 0;
  output_cloud->clear();
  for (int i = 1; i < clusters.size() - 1; i++) {
    if (clusters[i + 1]->points.size() > 0) {
      kd_tree->setInputCloud(clusters[i + 1]);
      for (int j = 0; j < clusters[i]->points.size(); j++) {
        PointT p = clusters[i]->points[j];
        kd_tree->nearestKSearch(p, K, point_idx, point_nkn_sd);
        int key;
        bool root = false;
        for (int k = 0; k < point_idx.size(); k++) {
          double x = clusters[i + 1]->points[point_idx[k]].x - clusters[i]->points[j].x;
          double y = clusters[i + 1]->points[point_idx[k]].y - clusters[i]->points[j].y;
          double z = clusters[i + 1]->points[point_idx[k]].z - clusters[i]->points[j].z;
          if (acos(z / sqrt( x*x + y*y + z*z)) < max_angle) {
            if (!root) {
              key = count++;
              output_cloud->push_back(clusters[i]->points[j]);
              root = true;
            }
            output_cloud->push_back(clusters[i + 1]->points[point_idx[k]]);
            (*found_points_number)[key].push_back(count++);
          }
        }
      }
    } else {
      if (i < clusters.size() - 2 && clusters[i + 2]->points.size() > 0) {
        kd_tree->setInputCloud(clusters[i + 2]);
        for (int j = 0; j < clusters[i]->points.size(); j++) {
          PointT p = clusters[i]->points[j];
          kd_tree->nearestKSearch(p, K, point_idx, point_nkn_sd);
          int key;
          bool root = false;
          for (int k = 0; k < point_idx.size(); k++) {
            double x = clusters[i + 2]->points[point_idx[k]].x - clusters[i]->points[j].x;
            double y = clusters[i + 2]->points[point_idx[k]].y - clusters[i]->points[j].y;
            double z = clusters[i + 2]->points[point_idx[k]].z - clusters[i]->points[j].z;
            if (acos(z / sqrt( x*x + y*y + z*z)) < max_angle) {
              if (!root) {
                key = count++;
                output_cloud->push_back(clusters[i]->points[j]);
                root = true;
              }
              output_cloud->push_back(clusters[i + 2]->points[point_idx[k]]);
              (*found_points_number)[key].push_back(count++);
            }
          }
        }
      }
    }
  }
  // получение данных из нулевого слоя
  for (int j = 0; j < slices[0]->points.size(); j++) {
    int key = count++;
    output_cloud->push_back(slices[0]->points[j]);
    output_cloud->points[key].intensity = -1;
    (*found_points_number)[key].push_back(key);
  }

  return err.str();
}

// установка параметров;
std::string TrunkFinder::setParameters(map<string, double>* param) {
  stringstream err;
  for (auto it = param->begin(); it != param->end(); ++it) {
    if (it->first.compare("v_delta") == 0) {
      if (it->second > 0)
        v_delta = it->second;
      else
        err << "incorrect v_delta value " << it->second << "; ";
    } else if (it->first.compare("v_shift") == 0) {
      v_shift = it->second;
    } else if (it->first.compare("cell_size") == 0) {
      if (it->second > 0)
        cell_size = it->second;
      else
        err << "incorrect cell_size value " << it->second << "; ";
    } else if (it->first.compare("cluster_tolerance") == 0) {
      if (it->second > 0)
        cluster_tolerance = it->second;
      else
        err << "incorrect cluster_tolerance value " << it->second << "; ";
    } else if (it->first.compare("cluster_max_size") == 0) {
      if (it->second > 1)
        cluster_max_size = it->second;
      else
        err << "incorrect cluster_max_size value " << it->second << "; ";
    } else if (it->first.compare("cluster_min_size") == 0) {
      if (it->second > 0)
        cluster_min_size = it->second;
      else
        err << "incorrect cluster_min_size value " << it->second << "; ";
    } else if (it->first.compare("max_angle_deg") == 0) {
      max_angle = M_PI * it->second / 180.0;
    } else if (it->first.compare("max_angle_rad") == 0) {
      max_angle = it->second;
    } else if (it->first.compare("slices_number") == 0) {
      if ((int)it->second > 1)
        slices_number = it->second;
      else
        err << "incorrect slices_number value " << it->second << "; ";
    } else if (it->first.compare("empty_radius") == 0) {
      if (it->second >= 0)
        empty_radius = it->second;
      else
        err << "incorrect empty_radius value " << it->second << "; ";
    } else {
      err << "undefined parameter \"" << it->first << "\"" << "; ";
    }
  }
  
  slices = vector<PointCloud<PointT>::Ptr>(slices_number);
  clusters = vector<PointCloud<PointT>::Ptr>(slices_number);
  for (int i = 0; i < slices.size(); i++) {
    slices[i] = PointCloud<PointT>::Ptr(new PointCloud<PointT>);
    clusters[i] = PointCloud<PointT>::Ptr(new PointCloud<PointT>);
  }
  downsampler.setLeafSize(cell_size, cell_size, v_delta/2.0);
  cluster_extractor.setClusterTolerance(cluster_tolerance);
  cluster_extractor.setMinClusterSize(cluster_min_size);
  cluster_extractor.setMaxClusterSize(cluster_max_size);
  return err.str();
}
