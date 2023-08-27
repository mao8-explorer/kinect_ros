#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>


using namespace sensor_msgs;
using namespace message_filters;

class SubscribeAndPublish {
public:
    SubscribeAndPublish() 
    {
        // This class provides an easy way to request and receive coordinate frame transform information
        tf2_ros::TransformListener listener(buffer);
         // 初始化指针
        master_pcl = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(new pcl::PointCloud<pcl::PointXYZRGB>());
        sub_pcl    = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(new pcl::PointCloud<pcl::PointXYZRGB>());
        sub_pcl_tf = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(new pcl::PointCloud<pcl::PointXYZRGB>());
        final_pcl  = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(new pcl::PointCloud<pcl::PointXYZRGB>());

        getTF_sub2master();
        fuse_pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/final_point_cloud", 10);
        message_filters::Subscriber<sensor_msgs::PointCloud2> master_pcl_sub(nh_, "/master/points2", 100, ros::TransportHints().tcpNoDelay());
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pcl_sub(nh_, "/sub/points2", 100, ros::TransportHints().tcpNoDelay());
        typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2>syncPolocy;
        Synchronizer<syncPolocy> sync(syncPolocy(10), master_pcl_sub,sub_pcl_sub);
        sync.registerCallback(boost::bind(&SubscribeAndPublish::callback,this,_1, _2));
        ros::spin();
    }
    void callback(const PointCloud2::ConstPtr& PointCloud21, const PointCloud2::ConstPtr& PointCloud22)
    {
        /*
        1.ros点云 转为 pcl库中点云 （PointCloud21 & PointCloud22）
        2.使用PCL库对副机做坐标变换
        3.对父子相机点云相加
        4.pcl点云转ros点云
        */
        // 1.ros点云 转为 pcl库中点云 （PointCloud21 & PointCloud22）
        //转换
        pcl::fromROSMsg(*PointCloud21, *master_pcl);
        pcl::fromROSMsg(*PointCloud22, *sub_pcl);
        // 2.使用PCL库对副机做坐标变换
        pcl::transformPointCloud(*sub_pcl, *sub_pcl_tf, transform_martix_);
        // 3.对父子相机点云相加
        *final_pcl = *master_pcl + *sub_pcl_tf;
        // 4.pcl点云转ros点云
        sensor_msgs::PointCloud2 fuse_pcl_ros;
        pcl::toROSMsg(*final_pcl, fuse_pcl_ros);
        fuse_pcl_ros.header.frame_id = "master_depth_camera_link";
        fuse_pcl_pub_.publish(fuse_pcl_ros);
        // ROS_INFO_STREAM("subordinate_depth_camera_link in master_depth_camera_link matrix=\n"
        //                 << transform_martix_);
    }

    void getTF_sub2master()
    {

        geometry_msgs::TransformStamped tfGeom;
        while (!is_tf_get_ && ros::ok())
        {
            try
            {
                is_tf_get_ = true;
                //target_frame – The frame to which data should be transformed
                //source_frame – The frame where the data originated
                tfGeom = buffer.lookupTransform("master_depth_camera_link","subordinate_depth_camera_link",ros::Time(0));
                ROS_INFO("%.3f,%.3f,%.3f",tfGeom.transform.translation.x,tfGeom.transform.translation.y,tfGeom.transform.translation.z);
                // 这里是否有隐患： 你查tf是需要时间的 有这个时间 点云发布怎么办？ 还能保证实时性吗？ 点云的发布频率是否会取决与 tf的获取时间？
                
            }
            catch (tf2::TransformException &e)
            {
                ROS_ERROR_STREAM("master_subordinate Transform Error ... ");
                ROS_INFO("异常信息:%s",e.what());
                is_tf_get_ = false;
            }
            // tf2矩阵转换成Eigen::Matrix4f
            Eigen::Quaternionf qw(tfGeom.transform.rotation.w, tfGeom.transform.rotation.x, tfGeom.transform.rotation.y, tfGeom.transform.rotation.z); //tf 获得的四元数
            Eigen::Vector3f qt(tfGeom.transform.translation.x, tfGeom.transform.translation.y, tfGeom.transform.translation.z);                        //tf获得的平移向量
            transform_martix_.block<3, 3>(0, 0) = qw.toRotationMatrix();
            transform_martix_.block<3, 1>(0, 3) = qt;
            ROS_INFO_STREAM("subordinate_depth_camera_link in master_depth_camera_link matrix=\n"
                            << transform_martix_);
        }

    }

private:
    ros::NodeHandle nh_;
    ros::Publisher fuse_pcl_pub_;
    Eigen::Matrix4f transform_martix_ = Eigen::Matrix4f::Identity();
    bool is_tf_get_ = false;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr master_pcl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_pcl;
    // sub_pcl 转 sub_pcl_tf
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_pcl_tf;
    // 合成点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_pcl;

    tf2_ros::Buffer buffer;



};

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "subscribe_two_and_translate_pub_one");
    SubscribeAndPublish sub_and_pub;
    ros::spin();
    return 0;
}
