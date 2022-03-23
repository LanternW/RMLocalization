#include <ros/ros.h> 
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

Vector2d p_center;

class PointPair
{
public:
    PointPair(Vector2d p_, Vector2d q_){p = p_; q = q_;}
    Vector2d p;
    Vector2d q;
    Matrix2d getW(){ return (p - p_center) * q.transpose(); }
public:
    typedef shared_ptr<PointPair> Ptr;
};

class RL2D
{

public:
    string map_filepath;

public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map;
    pcl::KdTreeFLANN<pcl::PointXYZ> global_kdtree;
    pcl::PointCloud<pcl::PointXYZ> lidar_scan;
    pcl::PointCloud<pcl::PointXYZ> lidar_scan_global;
    Vector2d lsg_center;

    ros::Subscriber gm_sub;
    ros::Subscriber ls_sub;
    ros::Subscriber imu_sub;
    ros::Publisher pose_pub;
    ros::Publisher gm_pub;
    ros::Publisher sc_pub;
    ros::Timer timer;


    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;

    Vector2d position;
    double yaw;
    Matrix3d Tlw;

    int last_imu_sec;
    int last_imu_nsec;

    bool has_map;
    bool rec_first_imu;

public:
    void init(ros::NodeHandle& nh);
    void loadGM(const string& filename);
    void globalMapCallBack(const sensor_msgs::PointCloud2& global_map_msg);
    void lidarCallback(const sensor_msgs::LaserScan& lidar_msg);
    void imuCallback( const sensor_msgs::Imu& imu_msg );
    void initPoseCallBack(const nav_msgs::Odometry& msg);
    void updatePose(Vector2d pos, double yaw);
    void updateLSG();
    void extractPose();
    void poseOutputStream(const ros::TimerEvent& e);
    void ICP2D();

public:
    typedef shared_ptr<RL2D> Ptr;

};