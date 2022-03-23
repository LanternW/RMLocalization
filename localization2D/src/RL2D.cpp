#include "localization2D/RL2D.hpp"


void RL2D::globalMapCallBack(const sensor_msgs::PointCloud2& global_map_msg)
{
    if(has_map){return;}
    global_map.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(global_map_msg, *global_map);

    global_kdtree.setInputCloud(global_map);
    has_map = true;
    cout << "Get Map!" << endl;
}

void RL2D::loadGM(const string& filename)
{
    if(has_map){return;}
    global_map.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloudBlob;
    pcl::io::loadPCDFile(filename, cloudBlob);
    pcl::fromPCLPointCloud2(cloudBlob, *global_map);

    global_kdtree.setInputCloud(global_map);
    has_map = true;

    // pcl::PointCloud<pcl::PointXYZ> global_map_3D;
    // pcl::PointXYZ tpt;

    // global_map_3D.points.clear();
    // global_map_3D.width  = global_map->points.size();
    // global_map_3D.height = 41;
    // global_map_3D.points.resize( global_map->points.size() * 41);
    // int index = 0;
    // for(int i = 0 ; i < global_map->points.size() ; i++)
    // {
    //     for(double z = 0.0; z < 0.8 ; z += 0.02)
    //     {
    //         pcl::PointXYZ tpt = global_map->points[i];
    //         tpt.z = z;
    //         global_map_3D.points[index++] = tpt;
    //     } 
    // }
    // pcl::io::savePCDFileASCII (map_filepath, global_map_3D); //将点云保存到PCD文件中


    cout << "Get Map!" << endl;
}


void RL2D::updateLSG()
{
    int n = 0;
    Vector3d pt;
    pcl::PointXYZ p;
    lidar_scan_global.points.clear();
    lsg_center = Vector2d::Zero();
    for(int i = 0 ; i < lidar_scan.points.size(); i++)
    {
        pt(0) = lidar_scan.points[i].x;
        pt(1) = -lidar_scan.points[i].y;
        pt(2) = 1;
        pt    = Tlw * pt; 

        p.x = pt(0);
        p.y = pt(1);
        p.z = 0;
        lidar_scan_global.points.push_back(p);

        lsg_center = ( n*lsg_center + pt.head(2) )/(n + 1.0);
        n++;
    }
}

void RL2D::lidarCallback(const sensor_msgs::LaserScan& lidar_msg)
{
    if(!has_map){return;}

    sensor_msgs::PointCloud2 lidar_msg_c;
    projector_.transformLaserScanToPointCloud("laser", lidar_msg, lidar_msg_c, tfListener_);
    pcl::fromROSMsg(lidar_msg_c, lidar_scan);

    // pcl::fromROSMsg(lidar_msg, lidar_scan);

    updateLSG();
    // //cout<<"lc = "<<lsg_center<<endl;
    ICP2D();
    sensor_msgs::PointCloud2 scan_map;
    pcl::toROSMsg(lidar_scan_global, scan_map);
    scan_map.header.frame_id = "world";
    sc_pub.publish(scan_map);
}

void RL2D::imuCallback( const sensor_msgs::Imu& imu_msg )
{ 
    if(rec_first_imu == false) { 
        last_imu_sec  = imu_msg.header.stamp.sec;
        last_imu_nsec = imu_msg.header.stamp.nsec;
        rec_first_imu = true; 
        return;
    }
    else {
        int dsec  = imu_msg.header.stamp.sec  - last_imu_sec;
        int dnsec = imu_msg.header.stamp.nsec - last_imu_nsec;
        double dt = dsec + dnsec/(1000000000.0);
        yaw += imu_msg.angular_velocity.z * dt;
        //cout<<"dt = "<<dt<<" vz = "<<imu_msg.angular_velocity.z<<" yaw="<<yaw<<endl;
        updatePose(position ,yaw);
        last_imu_sec  = imu_msg.header.stamp.sec;
        last_imu_nsec = imu_msg.header.stamp.nsec;
    }
}

void RL2D::extractPose()
{
    position = Tlw.block<2,1>(0,2);
    yaw      = atan2(Tlw(1,0), Tlw(0,0));
}

void RL2D::updatePose(Vector2d pos, double yaw)
{
    position = pos;
    this->yaw = yaw;
    Tlw(0,2) = pos(0);
    Tlw(1,2) = pos(1);
    double cy = cos(yaw);
    double sy = sin(yaw);

    Tlw(0,0) = Tlw(1,1) = cy;
    Tlw(0,1) = -sy;
    Tlw(1,0) = sy;
}


void RL2D::initPoseCallBack(const nav_msgs::Odometry& msg)
{
    position(0) = msg.pose.pose.position.x; 
    position(1) = msg.pose.pose.position.y;
    yaw = 2 * atan2( msg.pose.pose.orientation.z, msg.pose.pose.orientation.w );
}

void RL2D::poseOutputStream(const ros::TimerEvent& e)
{
    // extractPose();
    cout<<"yaw = "<< yaw <<"  t = "<<position<<endl;
    nav_msgs::Odometry odom;
    odom.header.frame_id = "world";
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = position(0);
    odom.pose.pose.position.y = position(1);
    odom.pose.pose.position.z = 0;

    odom.pose.pose.orientation.z = sin(yaw / 2.0);
    odom.pose.pose.orientation.w = cos(yaw / 2.0);
    pose_pub.publish(odom);

    sensor_msgs::PointCloud2 gl_map;
    pcl::toROSMsg(*global_map, gl_map);
    gl_map.header.frame_id = "world";
    gm_pub.publish(gl_map);
}

void RL2D::ICP2D()
{
    Matrix2d R = Matrix2d::Identity();
    Vector2d t = Vector2d::Zero();
    Matrix3d T = Matrix3d::Identity();
    vector<PointPair::Ptr> pps;
    PointPair::Ptr pp;

    vector<int> pointIdxNKNSearch(1);
    vector<float> pointNKNSquaredDistance(1);

    pcl::PointXYZ searchPoint;
    Vector2d p_point , s_point;
    Matrix2d W;


    W = Matrix2d::Zero();
    Matrix2d V,U;
    Matrix2d S;

    int n = 0;
    int iter_time = 10;
    while(iter_time--)
    {
        n = 0;
        W = Matrix2d::Zero();
        pps.clear();
        p_center = Vector2d::Zero();
        for(int i = 0 ; i < lidar_scan_global.points.size() ; i++)
        {
            searchPoint = lidar_scan_global.points[i];
            s_point(0) = searchPoint.x;
            s_point(1) = searchPoint.y;
            if(s_point.norm() < 0.5){continue;}
            // if(s_point.norm() > 9.0){continue;}
            if( global_kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            {
                if( pointNKNSquaredDistance[0] > 0.8){continue;}
                p_point(0) = global_map -> points[ pointIdxNKNSearch[0] ].x;
                p_point(1) = global_map -> points[ pointIdxNKNSearch[0] ].y;
                
                p_center = (n*p_center + p_point)/(n+1);
                n++;
                pp.reset( new PointPair(p_point, s_point - lsg_center) );
                pps.push_back( pp );
            }
        }

        for(int i = 0 ; i < pps.size(); i++)
        {
            W += pps[i] -> getW();
        }

        JacobiSVD<Eigen::MatrixXd> svd(W, ComputeFullU | ComputeFullV );
        V = svd.matrixV();
        U = svd.matrixU();
        //S = U.inverse() * W * V.transpose().inverse();
        R = U*V.transpose();
        if(R.determinant() < 0){R = -R;}
        t = p_center - R*lsg_center;

        T.block<2,2>(0,0) = R;
        T.block<2,1>(0,2) = t;
        Tlw = T * Tlw;
        updateLSG();

        // sensor_msgs::PointCloud2 scan_map;
        // pcl::toROSMsg(lidar_scan_global, scan_map);
        // scan_map.header.frame_id = "world";
        // sc_pub.publish(scan_map);
    }
    

    extractPose();

}



void RL2D::init(ros::NodeHandle& nh)
{
    //this->nh = nh;
    has_map         = false;
    rec_first_imu   = false;
    Tlw      = Matrix3d::Identity();

    tfListener_.setExtrapolationLimit(ros::Duration(0.1));

    nh.param("map_filepath",map_filepath );
    nh.param("init_x", position(0) ,2.02);
    nh.param("init_y", position(1) ,-3.84);
    nh.param("init_yaw", yaw ,0.0);

    gm_sub   = nh.subscribe("/global_map", 5, &RL2D::globalMapCallBack, this); 
    ls_sub   = nh.subscribe("/scan", 5, &RL2D::lidarCallback, this); 
    imu_sub  = nh.subscribe("/mavros/imu/data", 5 ,&RL2D::imuCallback, this);

    pose_pub = nh.advertise<nav_msgs::Odometry>("/odom", 5);
    gm_pub   = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 5);
    sc_pub   = nh.advertise<sensor_msgs::PointCloud2>("/scan_map", 5);

    timer    = nh.createTimer(ros::Duration(0.01), &RL2D::poseOutputStream, this);

}

int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "rl2d"); 
    ros::NodeHandle nh; 

    RL2D::Ptr steve;
    steve.reset(new RL2D);
    steve -> init(nh);
    steve -> loadGM( "/home/lantern/ROS_workspace/RMLocalization/src/RMLocalization/localization2D/data/RM_transform.pcd" );
    steve -> updatePose( steve->position , steve->yaw );

    ros::spin();
    return 0;
}
