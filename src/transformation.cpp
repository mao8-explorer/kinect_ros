
#include "ros/ros.h"

#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <iostream>
#include <json/json.h>
#include <fstream>
#include <vector>

using namespace std;
vector<double> readFileJson()
{
    vector<double> tfmat;
	Json::Reader reader;
	Json::Value root;
	//从文件中读取，保证当前文件有demo.json文件  
	ifstream in(
    "/home/zm/RobotVector/tr/multicamera/k4a-calibration/build/cn01.json", 
    ios::binary);
	if (!in.is_open())
	{
		cout << "Error opening file\n";
		return tfmat;
	}
	if (reader.parse(in, root))
	{
		tfmat.push_back(root["value0"]["translation"]["m00"].asDouble());
		tfmat.push_back(root["value0"]["translation"]["m10"].asDouble());
		tfmat.push_back(root["value0"]["translation"]["m20"].asDouble());
		tfmat.push_back(root["value0"]["rotation"]["x"].asDouble());
		tfmat.push_back(root["value0"]["rotation"]["y"].asDouble());
		tfmat.push_back(root["value0"]["rotation"]["z"].asDouble());
		tfmat.push_back(root["value0"]["rotation"]["w"].asDouble());
	}
	else
	{
		cout << "parse error\n" << endl;
	}

	in.close();
    return tfmat;
}


int main(int argc, char *argv[])
{   setlocale(LC_ALL,"");
    // 2.初始化 ros 节点
    ros::init(argc,argv,"sub_frames");
    // 3.创建 ros 句柄
    ros::NodeHandle nh;
    // 4.创建 TF 订阅对象
    tf2_ros::Buffer buffer; 
    
    tf2_ros::TransformListener listener(buffer);
    
    tf2_ros::StaticTransformBroadcaster broadcaster;
    // 5.解析订阅信息中获取 son1 坐标系原点在 son2 中的坐标
    vector<double> tfmat;
    tfmat = readFileJson();

    ros::Rate r(1);
    while (ros::ok())
    {
        try
        {
            geometry_msgs::TransformStamped Tsd_sr = buffer.lookupTransform("subordinate_rgb_camera_link","subordinate_depth_camera_link",ros::Time(0), ros::Duration(3.0));
            geometry_msgs::TransformStamped Tmr_md = buffer.lookupTransform("master_depth_camera_link","master_rgb_camera_link",ros::Time(0),ros::Duration(3.0));

            geometry_msgs::Pose Tsr_mr;
            Tsr_mr.position.x=     tfmat.at(0);
            Tsr_mr.position.y=     tfmat.at(1);
            Tsr_mr.position.z=     tfmat.at(2);
            Tsr_mr.orientation.x=  tfmat.at(3);
            Tsr_mr.orientation.y=  tfmat.at(4); 
            Tsr_mr.orientation.z=  tfmat.at(5);
            Tsr_mr.orientation.w=  tfmat.at(6);

            geometry_msgs::Pose  Tmr_md_;
            Tmr_md_.position.x = Tmr_md.transform.translation.x;
            Tmr_md_.position.y = Tmr_md.transform.translation.y;
            Tmr_md_.position.z = Tmr_md.transform.translation.z;
            Tmr_md_.orientation.x = Tmr_md.transform.rotation.x;
            Tmr_md_.orientation.y = Tmr_md.transform.rotation.y;
            Tmr_md_.orientation.z = Tmr_md.transform.rotation.z;
            Tmr_md_.orientation.w = Tmr_md.transform.rotation.w;

            geometry_msgs::Pose  Tsd_sr_;
            Tsd_sr_.position.x = Tsd_sr.transform.translation.x;
            Tsd_sr_.position.y = Tsd_sr.transform.translation.y;
            Tsd_sr_.position.z = Tsd_sr.transform.translation.z;
            Tsd_sr_.orientation.x = Tsd_sr.transform.rotation.x;
            Tsd_sr_.orientation.y = Tsd_sr.transform.rotation.y;
            Tsd_sr_.orientation.z = Tsd_sr.transform.rotation.z;
            Tsd_sr_.orientation.w = Tsd_sr.transform.rotation.w;
            ROS_INFO("%.2f,%.2f,%.2f",Tsd_sr.transform.translation.x,Tsd_sr.transform.translation.y,Tsd_sr.transform.translation.z);

            tf2::Transform tf_Tmr_md;
            tf2::fromMsg(Tmr_md_,tf_Tmr_md);
            tf2::Transform tf_Tsr_mr;
            tf2::fromMsg(Tsr_mr,tf_Tsr_mr);
            tf2::Transform tf_Tsd_sr;
            tf2::fromMsg(Tsd_sr_,tf_Tsd_sr);
            
            tf2::Transform tf_Tsd_md;
            tf_Tsd_md.mult(tf_Tsr_mr,tf_Tsd_sr);
            tf_Tsd_md.mult(tf_Tmr_md,tf_Tsd_md);

            geometry_msgs::Pose out;

            tf2::toMsg(tf_Tsd_md,out);
            geometry_msgs::TransformStamped ts;
            //----设置头信息
            ts.header.seq = 100;
            ts.header.stamp = ros::Time::now();
            ts.header.frame_id = "master_depth_camera_link";
            //----设置子级坐标系
            ts.child_frame_id = "subordinate_depth_camera_link";
            //----设置子级相对于父级的偏移量
            ts.transform.translation.x = out.position.x;
            ts.transform.translation.y = out.position.y;
            ts.transform.translation.z = out.position.z;
            ts.transform.rotation.x = out.orientation.x;
            ts.transform.rotation.y = out.orientation.y;
            ts.transform.rotation.z = out.orientation.z;
            ts.transform.rotation.w = out.orientation.w;
            // 5.广播器发布坐标系信息
            broadcaster.sendTransform(ts);

        }

        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("异常信息:%s",e.what());
        }


        r.sleep();
        // 6.spin
        ros::spinOnce();
    }
    return 0;
}
