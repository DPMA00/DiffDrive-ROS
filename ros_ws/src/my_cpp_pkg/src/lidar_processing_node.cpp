#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2/exceptions.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>


struct TrackedCluster{
    int id;
    rclcpp::Time timestamp;
    Eigen::Vector2f center;
    double angle_rad;
    double major_axis, minor_axis;
};

class LidarProcessNode : public rclcpp::Node
{
public:
    LidarProcessNode() : Node("lidar_processing_node")
    {
        point_cloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud>("point_cloud", 10,
            std::bind(&LidarProcessNode::PointCloudCallback, this, std::placeholders::_1));

        filtered_pt_cloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud_filtered",10);

        clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    }




private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr point_cloud_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pt_cloud_publisher;
    rclcpp::Clock::SharedPtr clock_;
    std::unique_ptr<tf2_ros::Buffer>tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener>tf_listener;
    std::vector<EllipticalCluster> previous_clusters;
    std::vector<EllipticalCluster> current_clusters;



    double std_dev = 0.02; //m

    void PointCloudCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg)
    {
        sensor_msgs::msg::PointCloud2 pc2_msg;
        sensor_msgs::convertPointCloudToPointCloud2(*msg, pc2_msg);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(pc2_msg, *cloud);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

        ec.setClusterTolerance(0.05);
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(250);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        for (const auto& indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

            for (int idx : indices.indices)
            {
                cloud_cluster->points.push_back(cloud->points[idx]);
            }
            
            pcl::PCA<pcl::PointXYZ> pca;

            pca.setInputCloud(cloud_cluster);

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud_cluster, centroid);

            Eigen::Matrix3f eig_vec = pca.getEigenVectors();
            Eigen::Vector3f eig_val = pca.getEigenValues();

            double angle = std::atan2(eig_vec(1,0), eig_vec(0,0));
            double major = std::sqrt(eig_val(0));
            double minor = std::sqrt(eig_val(1));

            EllipticalCluster ellipse_cluster;

            ellipse_cluster.timestamp = this->get_clock()->now();
            ellipse_cluster.center = centroid.head<2>();
            ellipse_cluster.angle_rad = angle;
            ellipse_cluster.major_axis = major;
            ellipse_cluster.minor_axis = minor;

            current_clusters.push_back(ellipse_cluster);

            
        }
    }

    void get_laser_frame_world_pose()
    {
        try {
            geometry_msgs::msg::TransformStamped tf = tf_buffer->lookupTransform("map", "laser_frame", tf2::TimePointZero);

            double x = tf.transform.translation.x;
            double x = tf.transform.translation.x;
            tf2::Quaternion q (tf.transform.rotation.x,
                               tf.transform.rotation.y,
                               tf.transform.rotation.z,
                               tf.transform.rotation.w
                              );

            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        }
    }


    void 
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<LidarProcessNode>());
    rclcpp::shutdown();
    return 0;
}