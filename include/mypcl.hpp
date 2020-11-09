#ifndef MYPCL_HPP
#define MYPCL_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>

namespace mypcl
{
    struct pose
    {
        pose(Eigen::Quaterniond _q, Eigen::Vector3d _t) : q(_q), t(_t){}
        Eigen::Quaterniond q;
        Eigen::Vector3d t;
    };

    std::vector<Eigen::Vector3d> pointcloud2eigen(pcl::PointCloud<PointType> pc)
    {
        std::vector<Eigen::Vector3d> pc_vec;
        for (size_t i = 0; i < pc.points.size(); i++)
            pc_vec.push_back(Eigen::Vector3d(pc.points[i].x,
                                             pc.points[i].y,
                                             pc.points[i].z));
        return pc_vec;
    }

    pcl::PointCloud<PointType> read_pointcloud(std::string path)
    {
        pcl::PointCloud<PointType> pc;
        pc.points.resize(1e8);
        std::fstream file;
        file.open(path);
        size_t cnt = 0;
        float x, y, z;
        while (!file.eof())
        {
            file >> x >> y >> z;
            pc.points[cnt].x = x;
            pc.points[cnt].y = y;
            pc.points[cnt].z = z;
            cnt++;
        }
        file.close();
        pc.points.resize(cnt);
        return pc;
    }

    pcl::PointCloud<PointType> read_pointcloud(
        std::vector<pcl::PointCloud<PointType>::Ptr> vec,
        int pn)
    {
        return *(vec[pn]);
    }

    std::vector<pose> read_pose(std::string path)
    {
        std::vector<pose> pose_vec;
        std::fstream file;
        file.open(path);
        double tx, ty, tz, w, x, y, z;
        while (!file.eof())
        {
            file >> tx >> ty >> tz >> w >> x >> y >> z;
            pose_vec.push_back(pose(Eigen::Quaterniond(w, x, y, z),
                                    Eigen::Vector3d(tx, ty, tz)));
        }
        file.close();
        return pose_vec;
    }

    void transform_pointcloud(pcl::PointCloud<PointType> const& pc_in,
                              pcl::PointCloud<PointType>& pt_out,
                              Eigen::Vector3d t,
                              Eigen::Quaterniond q)
    {
        size_t size = pc_in.points.size();
        pt_out.points.resize(size);
        for (size_t i = 0; i < size; i++)
        {
            Eigen::Vector3d pt_cur(pc_in.points[i].x, pc_in.points[i].y, pc_in.points[i].z);
            Eigen::Vector3d pt_to;
            pt_to = q * pt_cur + t;
            pt_out.points[i].x = pt_to.x();
            pt_out.points[i].y = pt_to.y();
            pt_out.points[i].z = pt_to.z();
        }
    }

    pcl::PointCloud<PointType>::Ptr append_cloud(pcl::PointCloud<PointType>::Ptr pc1,
                                                 pcl::PointCloud<PointType> pc2)
    {
        size_t size1 = pc1->points.size();
        size_t size2 = pc2.points.size();
        pc1->points.resize(size1 + size2);
        for (size_t i = size1; i < size1 + size2; i++)
        {
            pc1->points[i].x = pc2.points[i - size1].x;
            pc1->points[i].y = pc2.points[i - size1].y;
            pc1->points[i].z = pc2.points[i - size1].z;
        }
        return pc1;
    }

    pcl::PointCloud<PointType>::Ptr append_cloud(pcl::PointCloud<PointType>::Ptr pc1,
                                                 std::string path)
    {
        pcl::PointCloud<PointType> pc2 = read_pointcloud(path);
        pc1 = append_cloud(pc1, pc2);
        return pc1;
    }

    pcl::PointCloud<PointType> read_rest_pointcloud(std::vector<pose> pose_vec,
                                                    int num, bool is_plane = 1)
    {
        std::string path = "/home/sam/catkin_ws/src/lidar_calib/still_clouds/downsample/";
        pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>);
        pc->points.resize(1e6);
        bool is_init = false;
        
        for (size_t i = 0; i < pose_vec.size() - 1; i++)
        {
            Eigen::Vector3d t = pose_vec[i].t;
            Eigen::Quaterniond q = pose_vec[i].q;
            if (i != (size_t)num && !is_init)
            {
                if (is_plane)
                    *pc = read_pointcloud(path + "plane" + std::to_string(i) + ".json");
                else
                    *pc = read_pointcloud(path + "edge" + std::to_string(i) + ".json");
                transform_pointcloud(*pc, *pc, t, q);
                is_init = true;
            }
            else if (i != (size_t)num && is_init)
            {
                pcl::PointCloud<PointType>::Ptr pc2(new pcl::PointCloud<PointType>);
                if (is_plane)
                    *pc2 = read_pointcloud(path + "plane" + std::to_string(i) + ".json");
                else
                    *pc2 = read_pointcloud(path + "edge" + std::to_string(i) + ".json");
                transform_pointcloud(*pc2, *pc2, t, q);
                pc = append_cloud(pc, *pc2);
            }
        }
        return *pc;
    }

    pcl::PointCloud<PointType> read_rest_pointcloud(
        std::vector<pcl::PointCloud<PointType>::Ptr> vec,
        std::vector<pose> pose_vec,
        int pn)
    {
        pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>);
        pc->points.resize(1e6);
        bool is_init = false;
        
        for (size_t i = 0; i < pose_vec.size() - 1; i++)
        {
            Eigen::Vector3d t = pose_vec[i].t;
            Eigen::Quaterniond q = pose_vec[i].q;
            if (i != (size_t)pn && !is_init)
            {
                *pc = read_pointcloud(vec, i);
                transform_pointcloud(*pc, *pc, t, q);
                is_init = true;
            }
            else if (i != (size_t)pn && is_init)
            {
                pcl::PointCloud<PointType>::Ptr pc2(new pcl::PointCloud<PointType>);
                *pc2 = read_pointcloud(vec, i);
                transform_pointcloud(*pc2, *pc2, t, q);
                pc = append_cloud(pc, *pc2);
            }
        }
        return *pc;
    }

    void transform(PointType const* const pi,
                   PointType *const po,
                   Eigen::Quaterniond q,
                   Eigen::Vector3d t)
    {
        Eigen::Vector3d pt_cur(pi->x, pi->y, pi->z);
        Eigen::Vector3d pt_to;
        pt_to = q * pt_cur + t;
        po->x = pt_to.x();
        po->y = pt_to.y();
        po->z = pt_to.z();
    }

    double compute_inlier_ratio(std::vector<double> residuals, double ratio)
    {
        std::set<double> dis_vec;
        for (size_t i = 0; i < (size_t)(residuals.size() / 3); i++)
            dis_vec.insert(fabs(residuals[3 * i + 0]) + fabs(residuals[3 * i + 1]) + fabs(residuals[3 * i + 2]));

        return *(std::next(dis_vec.begin(), (int)((ratio) * dis_vec.size())));
    }

    std::vector<pose> record_pose(std::vector<pose> pose_vec)
    {
        std::vector<pose> best_pose;
        for (size_t i = 0; i < pose_vec.size(); i++)
        {
            Eigen::Quaterniond q(pose_vec[i].q.w(),
                                 pose_vec[i].q.x(),
                                 pose_vec[i].q.y(),
                                 pose_vec[i].q.z());
            Eigen::Vector3d t(pose_vec[i].t(0),
                              pose_vec[i].t(1),
                              pose_vec[i].t(2));
            best_pose.push_back(pose(q, t));
        }
        return best_pose;
    }

    void write_pose(std::vector<pose> pose_vec)
    {
        std::ofstream file;
        std::string path = "/home/sam/catkin_ws/src/lidar_calib/nine3/fine/";
        file.open(path + "pose.json", std::ofstream::trunc);
        Eigen::Quaterniond q0(pose_vec[0].q.w(),
                                pose_vec[0].q.x(), 
                                pose_vec[0].q.y(),
                                pose_vec[0].q.z());
        Eigen::Vector3d t0(pose_vec[0].t(0),
                            pose_vec[0].t(1),
                            pose_vec[0].t(2));
        for (size_t i = 0; i < pose_vec.size() - 1; i++)
        {
            Eigen::Vector3d t(0, 0, 0);
            Eigen::Quaterniond q(1, 0, 0, 0);
            t << q0.inverse()*(pose_vec[i].t-t0);
            q = q0.inverse()*pose_vec[i].q;
            file << t(0) << " " << t(1) << " " << t(2) << " "
                 << q.w() << " "<< q.x() << " "<< q.y() << " "<< q.z() << "\n";
        }
        file.close();
    }

    void write_pose(std::vector<pose> pose_vec, std::vector<pose> ref_vec)
    {
        std::ofstream file;
        std::string path = "/home/sam/catkin_ws/src/lidar_calib/nine3/fine/";
        file.open(path + "pose.json", std::ofstream::trunc);
        Eigen::Quaterniond q0(pose_vec[0].q.w(),
                                pose_vec[0].q.x(), 
                                pose_vec[0].q.y(),
                                pose_vec[0].q.z());
        Eigen::Vector3d t0(pose_vec[0].t(0),
                            pose_vec[0].t(1),
                            pose_vec[0].t(2));
        for (size_t i = 0; i < pose_vec.size() - 1; i++)
        {
            Eigen::Vector3d t(0, 0, 0);
            Eigen::Quaterniond q(1, 0, 0, 0);
            t << q0.inverse()*(pose_vec[i].t-t0);
            q = q0.inverse()*pose_vec[i].q;
            file << t(0) << " " << t(1) << " " << t(2) << " "
                 << q.w() << " "<< q.x() << " "<< q.y() << " "<< q.z() << "\n";
        }
        file.close();

        file.open(path + "ref.json", std::ofstream::trunc);
        for (size_t i = 0; i < ref_vec.size() - 1; i++)
        {
            Eigen::Quaterniond q = ref_vec[i].q;
            Eigen::Vector3d t = ref_vec[i].t;
            file << t(0) << " " << t(1) << " " << t(2) << " "
                 << q.w() << " "<< q.x() << " "<< q.y() << " "<< q.z() << "\n";
        }
        file.close();
    }

    void log_pose(std::vector<pose> pose_vec, int plane = 0, double threshold = 0.0,
                  double calib = 0.0)
    {
        std::ofstream file;
        std::string path = "/home/sam/catkin_ws/src/lidar_calib/nine3/fine/";
        file.open(path + "pose_log.json", std::ofstream::app);
        Eigen::Quaterniond q0(pose_vec[0].q.w(),
                                pose_vec[0].q.x(), 
                                pose_vec[0].q.y(),
                                pose_vec[0].q.z());
        Eigen::Vector3d t0(pose_vec[0].t(0),
                            pose_vec[0].t(1),
                            pose_vec[0].t(2));
        file << plane << " " << threshold << " " << calib << "\n";
        for (size_t i = 0; i < pose_vec.size() - 1; i++)
        {
            Eigen::Vector3d t(0, 0, 0);
            Eigen::Quaterniond q(1, 0, 0, 0);
            t << q0.inverse()*(pose_vec[i].t-t0);
            q = q0.inverse()*pose_vec[i].q;
            file << t(0) << " " << t(1) << " " << t(2) << " "
                 << q.w() << " "<< q.x() << " "<< q.y() << " "<< q.z() << "\n";
        }
        file << "\n";
        file.close();
    }

    void log_ref(std::vector<pose> ref_vec, int plane = 0, double threshold = 0.0,
                 double calib = 0.0)
    {
        std::ofstream file;
        std::string path = "/home/sam/catkin_ws/src/lidar_calib/nine3/fine/";
        file.open(path + "ref_log.json", std::ofstream::app);
        file << plane << " " << threshold << " " << calib << "\n";
        for (size_t i = 0; i < ref_vec.size() - 1; i++)
        {
            Eigen::Quaterniond q = ref_vec[i].q;
            Eigen::Vector3d t = ref_vec[i].t;
            file << t(0) << " " << t(1) << " " << t(2) << " "
                 << q.w() << " "<< q.x() << " "<< q.y() << " "<< q.z() << "\n";
        }
        file << "\n";
        file.close();
    }
}

#endif