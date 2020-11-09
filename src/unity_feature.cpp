#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

typedef pcl::PointXYZI PointType;
using namespace std;

ros::Publisher pub_full, pub_surf, pub_corn;

enum LID_TYPE{MID, HORIZON, VELO16, OUST64};

enum Feature{None, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround{Prev, Next};
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

struct orgtype
{
	double x_range; // sqrt(x*x+y*y)
	double dist; // 该点与后一个点的距离平方
	double angle[2]; // 前(后)一个点、该点、原点所成角度
	double inter_angle; // 前后点与该点的夹角
	E_jump adj[2]; // 每个点前后点的关系
	// Surround nor_dir;
	Feature feature_type;
	orgtype()
	{
		x_range = 0;
		adj[Prev] = Nr_nor;
		adj[Next] = Nr_nor;
		feature_type = None;
		inter_angle = 2;
	}
};

// const int hor_line = 6;
const double rad2deg = 180*M_1_PI;

int lidar_type;
double blind, inf_bound;
int N_SCANS;
int group_size;
double disA, disB;
double limit_maxmid, limit_midmin, limit_maxmin;
double p2l_ratio;
double jump_up_limit, jump_down_limit;
double cos160;
double edgea, edgeb;
double smallp_intersect, smallp_ratio;
int point_filter_num;

void mid_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
void horizon_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
void velo16_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
void give_feature(pcl::PointCloud<PointType> &pc, vector<orgtype> &types, pcl::PointCloud<PointType> &pl_corn, pcl::PointCloud<PointType> &pl_surf);
void pub_func(pcl::PointCloud<PointType> &pc, ros::Publisher pub, const ros::Time &ct);
int plane_judge(const pcl::PointCloud<PointType> &pc, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
bool small_plane(const pcl::PointCloud<PointType> &pc, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
bool edge_jump_judge(const pcl::PointCloud<PointType> &pc, vector<orgtype> &types, uint i, Surround nor_dir);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "unity_feature");
	ros::NodeHandle n;

	n.param<int>("lidar_type", lidar_type, 0);
	n.param<double>("blind", blind, 0.5);
	n.param<double>("inf_bound", inf_bound, 10);
	n.param<int>("N_SCANS", N_SCANS, 1);
	n.param<int>("group_size", group_size, 8);
	n.param<double>("disA", disA, 0.01);
	n.param<double>("disB", disB, 0.1);
	n.param<double>("p2l_ratio", p2l_ratio, 400);
	n.param<double>("limit_maxmid", limit_maxmid, 9);
	n.param<double>("limit_midmin", limit_midmin, 16);
	n.param<double>("limit_maxmin", limit_maxmin, 3.24);
	n.param<double>("jump_up_limit", jump_up_limit, 175.0);
	n.param<double>("jump_down_limit", jump_down_limit, 5.0);
	n.param<double>("cos160", cos160, 160.0);
	n.param<double>("edgea", edgea, 3);
	n.param<double>("edgeb", edgeb, 0.05);
	n.param<double>("smallp_intersect", smallp_intersect, 170.0);
	n.param<double>("smallp_ratio", smallp_ratio, 1.2);
	n.param<int>("point_filter_num", point_filter_num, 4);

	jump_up_limit = cos(jump_up_limit/180*M_PI);
	jump_down_limit = cos(jump_down_limit/180*M_PI);
	cos160 = cos(cos160/180*M_PI);
	smallp_intersect = cos(smallp_intersect/180*M_PI);

	ros::Subscriber sub_points;

	switch(lidar_type)
	{
		case MID:
		printf("MID40-70\n");
		sub_points = n.subscribe("/livox/lidar", 1000, mid_handler);
		// sub_points = n.subscribe("/livox/lidar_1LVDG1S006J5GZ3", 1000, mid_handler);
		break;

		case HORIZON:
		printf("HORIZON_MID70pro\n");
		sub_points = n.subscribe("/livox/lidar", 1000, horizon_handler);
		break;

		case VELO16:
		printf("VELO16\n");
		sub_points = n.subscribe("/velodyne_points", 1000, velo16_handler);
		break;

		case OUST64:
		printf("OUST64\n");
		sub_points = n.subscribe("/os1_cloud_node/points", 1000, oust64_handler);
		break;

		default:
		printf("Lidar type is wrong.\n");
		exit(0);
		break;
	}

	pub_full = n.advertise<sensor_msgs::PointCloud2>("/pc2_fullN", 20);
	pub_surf = n.advertise<sensor_msgs::PointCloud2>("/pc2_surfaceN", 20);
	pub_corn = n.advertise<sensor_msgs::PointCloud2>("/pc2_cornersN", 20);

	ros::spin();
	return 0;
}

double vx, vy, vz;
void mid_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::PointCloud<PointType> pc;
	pcl::fromROSMsg(*msg, pc);

	pcl::PointCloud<PointType> pl_corn, pl_surf;
	vector<orgtype> types;
	uint pt_size = pc.size() - 1;
	pl_corn.reserve(pt_size);
	pl_surf.reserve(pt_size);
	types.resize(pt_size + 1);

	for (uint i = 0; i < pt_size; i++)
	{
		types[i].x_range = pc[i].x;
		vx = pc[i].x - pc[i+1].x;
		vy = pc[i].y - pc[i+1].y;
		vz = pc[i].z - pc[i+1].z;
		types[i].dist = vx*vx + vy*vy + vz*vz;
	}

	types[pt_size].x_range = sqrt(pc[pt_size].x*pc[pt_size].x + pc[pt_size].y*pc[pt_size].y);

	give_feature(pc, types, pl_corn, pl_surf);

	ros::Time ct(ros::Time::now());
	pub_func(pc, pub_full, ct);
	pub_func(pl_surf, pub_surf, ct);
	pub_func(pl_corn, pub_corn, ct);
}

void horizon_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
  vector<pcl::PointCloud<PointType>> pl_buff(N_SCANS);
  vector<vector<orgtype>> typess(N_SCANS);
  pcl::PointCloud<PointType> pl_full, pl_corn, pl_surf;

  uint pt_size = msg->point_num;
  pl_corn.reserve(pt_size); pl_surf.reserve(pt_size);
  pl_full.resize(pt_size);

  for (int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].reserve(pt_size);
  }
  
  for (uint i=0; i<pt_size; i++)
  {
    if (msg->points[i].line < N_SCANS)
    {
      pl_full[i].x = msg->points[i].x;
      pl_full[i].y = msg->points[i].y;
      pl_full[i].z = msg->points[i].z;
      pl_full[i].intensity =  msg->points[i].reflectivity;
      pl_buff[msg->points[i].line].push_back(pl_full[i]);
    }
  }

  for (int j=0; j<N_SCANS; j++)
  {
    pcl::PointCloud<PointType> &pc = pl_buff[j];
    vector<orgtype> &types = typess[j];
    pt_size = pc.size();
    types.resize(pt_size);
    pt_size--;
    for (uint i=0; i<pt_size; i++)
    {
      types[i].x_range = sqrt(pc[i].x*pc[i].x + pc[i].y*pc[i].y);
      vx = pc[i].x - pc[i+1].x;
      vy = pc[i].y - pc[i+1].y;
      vz = pc[i].z - pc[i+1].z;
      types[i].dist = vx*vx + vy*vy + vz*vz;
    }
    // pt_size++;
    types[pt_size].x_range = sqrt(pc[pt_size].x*pc[pt_size].x + pc[pt_size].y*pc[pt_size].y);

    give_feature(pc, types, pl_corn, pl_surf);
  }

  ros::Time ct(ros::Time::now());
  pub_func(pl_full, pub_full, ct);
  pub_func(pl_surf, pub_surf, ct);
  pub_func(pl_corn, pub_corn, ct);
}

void velo16_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PointCloud<PointType> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);

  pcl::PointCloud<PointType> pl_corn, pl_surf, pl_full;
  vector<pcl::PointCloud<PointType>> pl_buff(N_SCANS);
  vector<vector<orgtype>> typess(N_SCANS);

  uint pt_size = pl_orig.size();
  pl_corn.reserve(pt_size);
  pl_surf.reserve(pt_size);
  pl_full.reserve(pt_size);

  for (int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].resize(pt_size);
    typess[i].resize(pt_size);
  }

  int idx = -1;
  int stat = 0; // 0代表上一次为0
  int scanID = 0;

  for (uint i=0; i<pt_size; i++)
  {
    PointType &ap = pl_orig[i];
    double leng = sqrt(ap.x*ap.x + ap.y*ap.y);
    
    if (leng > blind)
    {
      if (stat == 0)
      {
        stat = 1;
        idx++;
      }

      double ang = atan(ap.z / leng)*rad2deg;
      scanID = int((ang + 15) / 2 + 0.5);
      if (scanID>=N_SCANS || scanID <0)
      {
        continue;
      }
      pl_buff[scanID][idx] = ap;
      typess[scanID][idx].x_range = leng;
      pl_full.push_back(ap);
    }
    else
    {
      stat = 0;
    }
  }
  idx++;


  for (int j=0; j<N_SCANS; j++)
  {
    pcl::PointCloud<PointType> &pc = pl_buff[j];
    vector<orgtype> &types = typess[j];
    pc.erase(pc.begin()+idx, pc.end());
    types.erase(types.begin()+idx, types.end());
    pt_size = idx - 1;
    for (uint i=0; i<pt_size; i++)
    {
      // types[i].x_range = sqrt(pc[i].x*pc[i].x + pc[i].y*pc[i].y);
      vx = pc[i].x - pc[i+1].x;
      vy = pc[i].y - pc[i+1].y;
      vz = pc[i].z - pc[i+1].z;
      types[i].dist = vx*vx + vy*vy + vz*vz;
    }
    types[pt_size].x_range = sqrt(pc[pt_size].x*pc[pt_size].x + pc[pt_size].y*pc[pt_size].y);
    
    give_feature(pc, types, pl_corn, pl_surf);
  }

  ros::Time ct(ros::Time::now());
  pub_func(pl_full, pub_full, ct);
  pub_func(pl_surf, pub_surf, ct);
  pub_func(pl_corn, pub_corn, ct);
}

void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PointCloud<PointType> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  
  vector<pcl::PointCloud<PointType>> pl_buff(N_SCANS);
  vector<vector<orgtype>> typess(N_SCANS);
  pcl::PointCloud<PointType> pl_corn, pl_surf;

  uint pt_size = pl_orig.size();

  pl_corn.reserve(pt_size); pl_surf.reserve(pt_size);
  for (int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].reserve(pt_size);
    // typess[i].reserve(pt_size);
  }

  for (uint i=0; i<pt_size; i+=N_SCANS)
  {
    for (int j=0; j<N_SCANS; j++)
    {
      pl_buff[j].push_back(pl_orig[i+j]);
    }
  }

  for (int j=0; j<N_SCANS; j++)
  {
    pcl::PointCloud<PointType> &pc = pl_buff[j];
    vector<orgtype> &types = typess[j];
    pt_size = pc.size() - 1;
    types.resize(pt_size+1);
    for (uint i=0; i<pt_size; i++)
    {
      types[i].x_range = sqrt(pc[i].x*pc[i].x + pc[i].y*pc[i].y);
      vx = pc[i].x - pc[i+1].x;
      vy = pc[i].y - pc[i+1].y;
      vz = pc[i].z - pc[i+1].z;
      types[i].dist = vx*vx + vy*vy + vz*vz;
    }
    types[pt_size].x_range = sqrt(pc[pt_size].x*pc[pt_size].x + pc[pt_size].y*pc[pt_size].y);
    give_feature(pc, types, pl_corn, pl_surf);
  }

  ros::Time ct(ros::Time::now());
  pub_func(pl_orig, pub_full, ct);
  pub_func(pl_surf, pub_surf, ct);
  pub_func(pl_corn, pub_corn, ct);
}

void give_feature(pcl::PointCloud<PointType> &pc, vector<orgtype> &types,
				  pcl::PointCloud<PointType> &pl_corn, pcl::PointCloud<PointType> &pl_surf)
{
	uint pt_size = pc.size();
	uint plsize2;
	if (pt_size == 0)
	{
		printf("something wrong\n");
		return;
	}
	uint head = 0;
	while(types[head].x_range < blind)
	{
		head++;
	}

	// 平面点检测
	plsize2 = pt_size - group_size;

	Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
	Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

	uint i_nex, i2;
	uint last_i = 0;
	uint last_i_nex = 0;
	// 0:上次状态无用 1:上次为平面组
	int last_state = 0;
	int plane_type;

	for (uint i = head; i < plsize2; i++)
	{
		if (types[i].x_range < blind)
			continue;

		i2 = i;
		plane_type = plane_judge(pc, types, i, i_nex, curr_direct);

		if (plane_type == 1)
		{
			for (uint j = i; j <= i_nex; j++)
			{ 
				if (j != i && j != i_nex)
					types[j].feature_type = Real_Plane;
				else
					types[j].feature_type = Poss_Plane;
			}
			
			if (last_state == 1 && last_direct.norm() > 0.1)
			{
				double mod = last_direct.transpose() * curr_direct;
				if (mod > -0.707 && mod < 0.707)
					types[i].feature_type = Edge_Plane;
				else
					types[i].feature_type = Real_Plane;
			}
			
			i = i_nex - 1;
			last_state = 1;
		}
		else if (plane_type == 2)
		{
			i = i_nex;
			last_state = 0;
		}
		else if (plane_type == 0)
		{
			if (last_state == 1)
			{
				uint i_nex_tem; // 临时变量
				uint j;
				for (j=last_i+1; j<=last_i_nex; j++)
				{
					uint i_nex_tem2 = i_nex_tem;
					Eigen::Vector3d curr_direct2; // curr_direct临时变量

					uint ttem = plane_judge(pc, types, j, i_nex_tem, curr_direct2);

					if (ttem != 1)
					{
						i_nex_tem = i_nex_tem2;
						break;
					}
					curr_direct = curr_direct2;
				}

				if (j == last_i+1)
					last_state = 0;
				else
				{
					for (uint k=last_i_nex; k<=i_nex_tem; k++)
					{
						if (k != i_nex_tem)
							types[k].feature_type = Real_Plane;
						else
							types[k].feature_type = Poss_Plane;
					}
					i = i_nex_tem-1;
					i_nex = i_nex_tem;
					i2 = j-1;
					last_state = 1;
				}
			}
		}

		last_i = i2;
		last_i_nex = i_nex;
		last_direct = curr_direct;
	}

	plsize2 = pt_size - 3;
	for (uint i = head + 3; i < plsize2; i++)
	{
		if (types[i].x_range < blind || types[i].feature_type >= Real_Plane)
			continue;

		if (types[i-1].dist < 1e-16 || types[i].dist < 1e-16)
			continue;

		Eigen::Vector3d vec_a(pc[i].x, pc[i].y, pc[i].z);
		Eigen::Vector3d vecs[2];

		for (int j = 0; j < 2; j++)
		{
			int m = -1;
			if (j == 1)
				m = 1;

			if (types[i+m].x_range < blind)
			{
				if (types[i].x_range > inf_bound)
					types[i].adj[j] = Nr_inf;
				else
					types[i].adj[j] = Nr_blind;
				continue;
			}

			vecs[j] = Eigen::Vector3d(pc[i+m].x, pc[i+m].y, pc[i+m].z);
			vecs[j] = vecs[j] - vec_a;
			
			types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
			if (types[i].angle[j] < jump_up_limit)
				types[i].adj[j] = Nr_180;
			else if (types[i].angle[j] > jump_down_limit)
				types[i].adj[j] = Nr_zero;
		}

		types[i].inter_angle = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();

		if (types[i].adj[Prev] == Nr_nor && types[i].adj[Next] == Nr_zero &&
			types[i].dist > 0.0225 && types[i].dist > 4 * types[i-1].dist)
		{
			if (types[i].inter_angle > cos160)
				if (edge_jump_judge(pc, types, i, Prev))
					types[i].feature_type = Edge_Jump;
		}
		else if (types[i].adj[Prev] == Nr_zero && types[i].adj[Next] == Nr_nor &&
				 types[i-1].dist>0.0225 && types[i-1].dist>4*types[i].dist)
		{
			if (types[i].inter_angle > cos160)
				if (edge_jump_judge(pc, types, i, Next))
					types[i].feature_type = Edge_Jump;
		}
		else if (types[i].adj[Prev] == Nr_nor && types[i].adj[Next] == Nr_180)
		{
			if (edge_jump_judge(pc, types, i, Next))
				types[i].feature_type = Edge_Jump;
		}
		else if (types[i].adj[Prev] == Nr_180 && types[i].adj[Next] == Nr_nor)
		{
			if (edge_jump_judge(pc, types, i, Next))
				types[i].feature_type = Edge_Jump;
		}
		else if (types[i].adj[Prev]==Nr_nor && types[i].adj[Next]==Nr_inf)
		{
			if (edge_jump_judge(pc, types, i, Prev))
				types[i].feature_type = Edge_Jump;
		}
		else if (types[i].adj[Prev]==Nr_inf && types[i].adj[Next]==Nr_nor)
		{
			if (edge_jump_judge(pc, types, i, Next))
				types[i].feature_type = Edge_Jump;	
		}
		else if (types[i].adj[Prev] > Nr_nor && types[i].adj[Next] > Nr_nor)
		{
			if (types[i].feature_type == None)
				types[i].feature_type = Wire;
		}
	}

	plsize2 = pt_size - 1;
	// comment = without la si
	double ratio;
	for (uint i = head + 1; i < plsize2; i++)
	{
		if (types[i].x_range < blind || types[i-1].x_range < blind || types[i+1].x_range < blind)
			continue;

		if (types[i-1].dist < 1e-8 || types[i].dist < 1e-8)
			continue;

		if (types[i].feature_type == None)
		{
			if (types[i-1].dist > types[i].dist)
				ratio = types[i-1].dist / types[i].dist;
			else
				ratio = types[i].dist / types[i-1].dist;

			if (types[i].inter_angle < smallp_intersect && ratio < smallp_ratio)
			{
				if (types[i-1].feature_type == None)
					types[i-1].feature_type = Real_Plane;

				if (types[i+1].feature_type == None)
					types[i+1].feature_type = Real_Plane;

				types[i].feature_type = Real_Plane;
			}
		}
	}

	int last_surface = -1;

	for (uint j = head; j < pt_size; j++)
	{
		if (types[j].feature_type == Poss_Plane || types[j].feature_type == Real_Plane)
		{
			if (last_surface == -1)
				last_surface = j;
			else if (j == (last_surface + point_filter_num - 1))
			{
				PointType ap;
				for (uint k = last_surface; k <= j; k++)
				{
					ap.x += pc[k].x;
					ap.y += pc[k].y;
					ap.z += pc[k].z;
				}
				ap.x /= point_filter_num;
				ap.y /= point_filter_num;
				ap.z /= point_filter_num;
				pl_surf.push_back(ap);
				last_surface = -1;
			}
		}
		else
		{
			// if (types[j].feature_type == Edge_Jump)
			if (types[j].feature_type == Edge_Jump || types[j].feature_type == Edge_Plane)
				pl_corn.push_back(pc[j]);

			if (last_surface != -1)
			{
				PointType ap;
				for (uint k=last_surface; k<j; k++)
				{
					ap.x += pc[k].x;
					ap.y += pc[k].y;
					ap.z += pc[k].z;
				}
				ap.x /= (j-last_surface);
				ap.y /= (j-last_surface);
				ap.z /= (j-last_surface);
				pl_surf.push_back(ap);
			}
			last_surface = -1;
		}
	}
}

void pub_func(pcl::PointCloud<PointType> &pc, ros::Publisher pub, const ros::Time &ct)
{
	pc.height = 1;
	pc.width = pc.size();
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(pc, output);
	output.header.frame_id = "camera_init";
	output.header.stamp = ct;
	pub.publish(output);
}

int plane_judge(const pcl::PointCloud<PointType>& pc, vector<orgtype>& types,
				uint i_cur, uint& i_nex, Eigen::Vector3d& curr_direct)
{
	double group_dis = disA * types[i_cur].x_range + disB;
	group_dis = group_dis * group_dis;

	double two_dis;
	vector<double> disarr;
	disarr.reserve(20);

	for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++)
	{
		if (types[i_nex].x_range < blind)
		{
			curr_direct.setZero();
			return 2;
		}
		disarr.push_back(types[i_nex].dist);
	}

	for (;;)
	{
		if (types[i_nex].x_range < blind)
		{
			curr_direct.setZero();
			return 2;
		}
		vx = pc[i_nex].x - pc[i_cur].x;
		vy = pc[i_nex].y - pc[i_cur].y;
		vz = pc[i_nex].z - pc[i_cur].z;
		two_dis = vx*vx + vy*vy + vz*vz;
		if (two_dis >= group_dis)
			break;

		disarr.push_back(types[i_nex].dist);
		i_nex++;
	}

	double leng_wid = 0;
	double v1[3], v2[3];
	for (uint j = i_cur + 1; j < i_nex; j++)
	{
		v1[0] = pc[j].x - pc[i_cur].x;
		v1[1] = pc[j].y - pc[i_cur].y;
		v1[2] = pc[j].z - pc[i_cur].z;

		v2[0] = v1[1]*vz - vy*v1[2];
		v2[1] = v1[2]*vx - v1[0]*vz;
		v2[2] = v1[0]*vy - vx*v1[1];

		double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
		if (lw > leng_wid)
			leng_wid = lw;
	}

	if ((two_dis * two_dis / leng_wid) < p2l_ratio)
	{
		curr_direct.setZero();
		return 0;
	}

	uint disarrsize = disarr.size();
	for (uint j = 0; j < disarrsize - 1; j++)
	{
		for (uint k = j + 1; k < disarrsize; k++)
		{
			if (disarr[j] < disarr[k])
			{
				leng_wid = disarr[j];
				disarr[j] = disarr[k];
				disarr[k] = leng_wid;
			}
		}
	}

	if (disarr[disarr.size()-2] < 1e-16)
	{
		curr_direct.setZero();
		return 0;
	}

	if (lidar_type == MID || lidar_type == HORIZON)
	{
		double dismax_mid = disarr[0] / disarr[disarrsize/2];
		double dismid_min = disarr[disarrsize/2] / disarr[disarrsize-2];

		if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin)
		{
			curr_direct.setZero();
			return 0;
		}
	}
	else
	{
		double dismax_min = disarr[0] / disarr[disarrsize-2];
		if (dismax_min >= limit_maxmin)
		{
			curr_direct.setZero();
			return 0;
		}
	}

	curr_direct << vx, vy, vz;
	curr_direct.normalize();
	return 1;
}

bool edge_jump_judge(const pcl::PointCloud<PointType> &pc, vector<orgtype> &types, uint i, Surround nor_dir)
{
	if (nor_dir == 0)
		if (types[i-1].x_range < blind || types[i-2].x_range < blind)
			return false;
	else if (nor_dir == 1)
		if (types[i+1].x_range < blind || types[i+2].x_range < blind)
			return false;

	double d1 = types[i+nor_dir-1].dist;
	double d2 = types[i+3*nor_dir-2].dist;
	double d;

	if (d1 < d2)
	{
		d = d1;
		d1 = d2;
		d2 = d;
	}

	d1 = sqrt(d1);
	d2 = sqrt(d2);

	if (d1 > edgea * d2 || (d1 - d2) > edgeb)
		return false;

	return true;
}