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

        if(abs(pt(0)) < 0.18) {continue;}
        if(pt(1) < 0.1 && pt(1) > -0.3) {continue;}

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_scan_raw(new pcl::PointCloud<pcl::PointXYZ>);
    projector_.transformLaserScanToPointCloud("laser", lidar_msg, lidar_msg_c, tfListener_);
    pcl::fromROSMsg(lidar_msg_c, *lidar_scan_raw);


    Static.setInputCloud (lidar_scan_raw);                           //设置待滤波的点云
    Static.setMeanK (10);                               //设置在进行统计时考虑查询点临近点数
    Static.setStddevMulThresh (0.2);                      //设置判断是否为离群点的阀值
    Static.filter (lidar_scan);                    //存储

    // pcl::fromROSMsg(lidar_msg, lidar_scan);

    updateLSG();
    // //cout<<"lc = "<<lsg_center<<endl;
    ros::Time t1 = ros::Time::now();
    ICP2D();
    ros::Time t2 = ros::Time::now();
    std::cout<<" icp cost = "<< 1000 * (t2.toSec() - t1.toSec()) <<" ms"<<std::endl;
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

Vector2d RL2D::findNearestLineSeg(Vector2d p, double& min_dis)
{
    double min_dist = 10e9 , dist;
    int index = 0;
    for(int i = 0 ; i < map_segs.size(); i++){
        dist = map_segs[i].getDis(p);
        if(dist < min_dist){min_dist = dist; index = i;}
    }
    min_dis = min_dist;
    return map_segs[index].getProjection(p);
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
    int iter_time = 5;
    while(iter_time--)
    {
        n = 0;
        W = Matrix2d::Zero();
        pps.clear();
        p_center = Vector2d::Zero();
        q_center = Vector2d::Zero();
        for(int i = 0 ; i < lidar_scan_global.points.size() ; i++)
        {
            searchPoint = lidar_scan_global.points[i];
            s_point(0) = searchPoint.x;
            s_point(1) = searchPoint.y;
            // if((s_point - position).norm() < 0.3){continue;}
            // if((s_point - position).norm() > 7.0){continue;}
            double min_dist;
            // ros::Time t1 = ros::Time::now();
            p_point = findNearestLineSeg(s_point, min_dist);
            // ros::Time t2 = ros::Time::now();
            // cout<<"COST t = " << t2.toSec() - t1.toSec()<<endl;
            if( min_dist > 0.5){continue;}

            p_center = (n*p_center + p_point)/(n+1);
            q_center = (n*q_center + s_point)/(n+1);
            n++;
            pp.reset( new PointPair(p_point, s_point) );
            pps.push_back( pp );

            // if( global_kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            // {
            //     if( pointNKNSquaredDistance[0] > 0.8){continue;}
            //     p_point(0) = global_map -> points[ pointIdxNKNSearch[0] ].x;
            //     p_point(1) = global_map -> points[ pointIdxNKNSearch[0] ].y;
                
            //     p_center = (n*p_center + p_point)/(n+1);
            //     n++;
            //     pp.reset( new PointPair(p_point, s_point - lsg_center) );
            //     pps.push_back( pp );
            // }
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
        t = p_center - R*q_center;

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


void RL2D::ICPStream(const ros::TimerEvent& e)
{
    updateLSG();
    ICP2D();
}



void RL2D::init(ros::NodeHandle& nh)
{
    //this->nh = nh;
    has_map         = false;
    rec_first_imu   = false;
    Tlw      = Matrix3d::Identity();

    tfListener_.setExtrapolationLimit(ros::Duration(0.1));

    nh.param("map_filepath",map_filepath );
    nh.param("init_x", position(0) ,-2.02);
    nh.param("init_y", position(1) ,3.84);
    nh.param("init_yaw", yaw ,0.0);

    //地图边界
    map_segs.push_back(MapLineSeg( Vector2d(2.24, -4.04) , Vector2d(2.24, 4.04) ));  
    map_segs.push_back(MapLineSeg( Vector2d(2.24, 4.04) , Vector2d(-2.24, 4.04) ));
    map_segs.push_back(MapLineSeg( Vector2d(-2.24, 4.04) , Vector2d(-2.24, -4.04) ));
    map_segs.push_back(MapLineSeg( Vector2d(-2.24, -4.04) , Vector2d(2.24, -4.04) ));

    //四个大障碍块
    map_segs.push_back(MapLineSeg( Vector2d(1.35, -0.5) , Vector2d(1.15, -0.5) ));  
    map_segs.push_back(MapLineSeg( Vector2d(1.15, -0.5) , Vector2d(1.15, 0.5) ));
    map_segs.push_back(MapLineSeg( Vector2d(1.15, 0.5) , Vector2d(1.35, 0.5) ));
    map_segs.push_back(MapLineSeg( Vector2d(1.35, 0.5) , Vector2d(1.35, -0.5) ));

    map_segs.push_back(MapLineSeg( Vector2d(-1.35, -0.5) , Vector2d(-1.15, -0.5) ));  
    map_segs.push_back(MapLineSeg( Vector2d(-1.15, -0.5) , Vector2d(-1.15, 0.5) ));
    map_segs.push_back(MapLineSeg( Vector2d(-1.15, 0.5) ,  Vector2d(-1.35, 0.5) ));
    map_segs.push_back(MapLineSeg( Vector2d(-1.35, 0.5) ,  Vector2d(-1.35, -0.5) ));

    map_segs.push_back(MapLineSeg( Vector2d(0.1, -2.54) , Vector2d(-0.1, -2.54) ));  
    map_segs.push_back(MapLineSeg( Vector2d(-0.1, -2.54) , Vector2d(-0.1, -1.54) ));
    map_segs.push_back(MapLineSeg( Vector2d(-0.1, -1.54) ,  Vector2d(0.1, -1.54) ));
    map_segs.push_back(MapLineSeg( Vector2d(0.1, -1.54) ,  Vector2d(0.1, -2.54) ));


    map_segs.push_back(MapLineSeg( Vector2d(0.1,  2.54) , Vector2d(-0.1, 2.54) ));  
    map_segs.push_back(MapLineSeg( Vector2d(-0.1, 2.54) , Vector2d(-0.1, 1.54) ));
    map_segs.push_back(MapLineSeg( Vector2d(-0.1, 1.54) ,  Vector2d(0.1, 1.54) ));
    map_segs.push_back(MapLineSeg( Vector2d(0.1,  1.54) ,  Vector2d(0.1, 2.54) ));

    //一个小障碍块
    map_segs.push_back(MapLineSeg( Vector2d(0,  0.15) , Vector2d(0.15, 0) ));  
    map_segs.push_back(MapLineSeg( Vector2d(0.15, 0) , Vector2d(0, -0.15) ));
    map_segs.push_back(MapLineSeg( Vector2d(0, -0.15) ,  Vector2d(-0.15, 0) ));
    map_segs.push_back(MapLineSeg( Vector2d(-0.15, 0) ,  Vector2d(0,  0.15) ));



    gm_sub   = nh.subscribe("/global_map", 5, &RL2D::globalMapCallBack, this); 
    ls_sub   = nh.subscribe("/scan", 5, &RL2D::lidarCallback, this); 
    imu_sub  = nh.subscribe("/mavros/imu/data", 5 ,&RL2D::imuCallback, this);

    pose_pub = nh.advertise<nav_msgs::Odometry>("/odom", 5);
    gm_pub   = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 5);
    sc_pub   = nh.advertise<sensor_msgs::PointCloud2>("/scan_map", 5);

    timer     = nh.createTimer(ros::Duration(0.01), &RL2D::poseOutputStream, this);
    // icp_timer = nh.createTimer(ros::Duration(0.01), &RL2D::ICPStream, this);

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
