#ifndef PARSE_HPP
#define PARSE_HPP

#include <string>
#include <iostream>
#include <fstream>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tools/common.h"
#include "mypcl.hpp"

using namespace std;

void parse_cloud(pcl::PointCloud<PointType> inc, int dis, bool is_edge = false)
{
    size_t size = inc.points.size();
    string address = "/home/sam/catkin_ws/src/lidar_calib/raw_bag/nine3/2/";
    string path = address + (is_edge?"edge":"plane") + std::to_string(dis) + ".json";
    ofstream file_w;
    file_w.open(path, std::ofstream::app);
    for (size_t i = 0; i < size; i++)
        file_w << inc.points[i].x << "\t" << inc.points[i].y << "\t" << inc.points[i].z << "\n";
    file_w.close();
}

void parse_pose()
{
    std::vector<mypcl::pose> pose_vec;
    std::vector<mypcl::pose> after_pose;
    fstream file;
    file.open("/home/sam/catkin_ws/src/lidar_calib/nine/fine/pose.json");
    double tx, ty, tz, w, x, y, z;
    while (!file.eof())
    {
        file >> tx >> ty >> tz >> w >> x >> y >> z;
        Eigen::Quaterniond q(w, x, y, z);
        Eigen::Vector3d t(tx, ty, tz);
        pose_vec.push_back(mypcl::pose(q, t));
    }
    file.close();
    Eigen::Quaterniond q(pose_vec[0].q.w(),
                         pose_vec[0].q.x(), 
                         pose_vec[0].q.y(),
                         pose_vec[0].q.z());
    Eigen::Vector3d t(pose_vec[0].t(0),
                      pose_vec[0].t(1),
                      pose_vec[0].t(2));
                      
    for (size_t i = 0; i < pose_vec.size() - 1; i++)
    {
        Eigen::Vector3d t_tmp(0, 0, 0);
        Eigen::Quaterniond q_temp(1, 0, 0, 0);
        t_tmp << q.inverse()*(pose_vec[i].t-t);
        q_temp = q.inverse()*pose_vec[i].q;
        after_pose.push_back(mypcl::pose(q_temp, t_tmp));
    }

    ofstream file_w;
    file_w.open("/home/sam/catkin_ws/src/lidar_calib/nine/fine/parse_pose.json", std::ofstream::trunc);
    for (size_t i = 0; i < after_pose.size(); i++)
    {
        file_w << after_pose[i].t(0) << "\t"
               << after_pose[i].t(1) << "\t"
               << after_pose[i].t(2) << "\t"
               << after_pose[i].q.w() << "\t"
               << after_pose[i].q.x() << "\t"
               << after_pose[i].q.y() << "\t"
               << after_pose[i].q.z() << "\n";
    }
    file_w.close();
}

#endif