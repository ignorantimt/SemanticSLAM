#include <opencv2/core/core.hpp>

#include <System.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/msg/image.hpp>
#include <pcl/io/pcd_io.h>

using namespace std;

geometry_msgs::msg::PoseStamped Cam_Pose;
geometry_msgs::msg::PoseWithCovarianceStamped Cam_odom;
nav_msgs::msg::Odometry odom;

cv::Mat Camera_Pose;
tf2::Transform sg_slam_tf;


class RosRGBD : public rclcpp::Node
{
public:
    RosRGBD(const std::string& vocabulary_path, const std::string& settings_path)
        : Node("ros_rgbd")
    {
        //mpSLAM = new ORB_SLAM2::System(vocabulary_path, settings_path, ORB_SLAM2::System::RGBD, true);
        mpSLAM = std::make_unique<ORB_SLAM2::System>(vocabulary_path, settings_path, ORB_SLAM2::System::RGBD, true);
        
        CamPose_Pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/Camera_Pose", 1);
        Camodom_Pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/Camera_Odom", 1);
        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);

        rgb_sub.subscribe(this, "/camera/color/image_raw");
        depth_sub.subscribe(this, "/camera/aligned_depth_to_color/image_raw");

        sync.reset(new Sync(SyncPolicy(10), rgb_sub, depth_sub));
        sync->registerCallback(&RosRGBD::GrabRGBD, this);
    }

    ~RosRGBD() = default;


    void shutdown()
    {
        // Save global point cloud to .pcd file
        if (mpSLAM->is_global_pc_reconstruction)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            *global_point_cloud = mpSLAM->GetPointCloudMapper()->get_globalMap();
            if (!global_point_cloud->empty())
                pcl::io::savePCDFileBinary("global_pcd.pcd", *global_point_cloud);
        }

        // Stop all threads
        mpSLAM->Shutdown();

        // Save camera trajectory
        mpSLAM->SaveTrajectoryTUM("CameraTrajectory.txt");
        mpSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

    void GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr& msgRGB,const sensor_msgs::msg::Image::ConstSharedPtr& msgD)
    {
        // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptrRGB, cv_ptrD;
        try
        {
            cv_ptrRGB = cv_bridge::toCvShare(msgRGB,"bgr8");
            cv_ptrD = cv_bridge::toCvShare(msgD);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("semantic_slam_ros_rgbd"), "cv_bridge exception: %s", e.what());
            return;
        }

        Camera_Pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.sec +
                                                                        cv_ptrRGB->header.stamp.nanosec*1e-9);

        Pub_CamPose(Camera_Pose); 
    }

    void Pub_CamPose(cv::Mat &pose)
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        Eigen::Matrix<double,3,3> rotationMat;
        if(pose.dims < 2 || pose.rows < 3)
        {
            Rwc = Rwc;
            twc = twc;
        }
        else
        {
            Rwc = pose.rowRange(0,3).colRange(0,3).t();//pose is Tcw, so Rwc need .t()
            twc = -Rwc*pose.rowRange(0,3).col(3);
            
            rotationMat << Rwc.at<float>(0,0), Rwc.at<float>(0,1), Rwc.at<float>(0,2),
                        Rwc.at<float>(1,0), Rwc.at<float>(1,1), Rwc.at<float>(1,2),
                        Rwc.at<float>(2,0), Rwc.at<float>(2,1), Rwc.at<float>(2,2);
            Eigen::Quaterniond Q(rotationMat);

            geometry_msgs::msg::PoseStamped cam_pose;
            cam_pose.header.stamp = this->now();
            cam_pose.header.frame_id = "map";

            // Convert from Eigen Quaternion to geometry_msgs Quaternion
            tf2::Quaternion tf_quat(Q.z(), -Q.x(), -Q.y(), Q.w());
            cam_pose.pose.position.x = twc.at<float>(2);
            cam_pose.pose.position.y = -twc.at<float>(0);
            cam_pose.pose.position.z = -twc.at<float>(1);
            cam_pose.pose.orientation = tf2::toMsg(tf_quat);

            CamPose_Pub->publish(cam_pose);
        }
    }
public:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr CamPose_Pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr Camodom_Pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub;
    
private:
    std::unique_ptr<ORB_SLAM2::System> mpSLAM;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    std::shared_ptr<Sync> sync;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    

    if (argc < 3)
    {
        std::cerr << std::endl << "Usage: ros2 run semantic_slam_ros_rgbd path_to_vocabulary path_to_settings" << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    auto node = std::make_shared<RosRGBD>(argv[1], argv[2]);
    rclcpp::spin(node);
    node->shutdown();
    rclcpp::shutdown();

    return 0;
}
