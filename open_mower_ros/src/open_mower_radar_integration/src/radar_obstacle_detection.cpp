#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <std_msgs/Bool.h>

class RadarObstacleNode {
public:
    RadarObstacleNode() {
        ros::NodeHandle nh;

        sub_ = nh.subscribe("/zero/ti_mmwave/radar_scan_pcl", 1, &RadarObstacleNode::pointCloudCallback, this);
        marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("radar_obstacles_markers", 1);
        radar_emergency_pub_ = nh.advertise<std_msgs::Bool>("/radar/emergency_trigger", 1);

        emergency_active_ = false;
        consecutive_detections_ = 0;
        consecutive_clear_frames_ = 0;
        last_trigger_time_ = ros::Time(0);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : cloud->points) {
            if (std::hypot(point.x, point.y) < 2.0) {
                filtered->points.push_back(point);
            }
        }

        if (filtered->empty()) {
            consecutive_clear_frames_++;
            if (emergency_active_ && consecutive_clear_frames_ >= 5 &&
                (ros::Time::now() - last_trigger_time_).toSec() > 5.0) {
                publishEmergency(false);
            }
            if (consecutive_clear_frames_ >= 5) {
                resetCounters();
            }
            return;
        }

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.5);
        ec.setMinClusterSize(3);
        ec.setMaxClusterSize(100);
        ec.setSearchMethod(tree);
        ec.setInputCloud(filtered);
        ec.extract(cluster_indices);

        consecutive_detections_++;
        consecutive_clear_frames_ = 0;
        ROS_INFO_THROTTLE(1.0, "Clusters: %lu | Detections: %d", cluster_indices.size(), consecutive_detections_);

        publishMarkers(cluster_indices, filtered, msg->header);

        if (consecutive_detections_ >= 3 && !emergency_active_ &&
            (ros::Time::now() - last_trigger_time_).toSec() > 1.0) {
            publishEmergency(true);
        }
    }

private:
    ros::Subscriber sub_;
    ros::Publisher marker_pub_;
    ros::Publisher radar_emergency_pub_;

    bool emergency_active_;
    int consecutive_detections_;
    int consecutive_clear_frames_;
    ros::Time last_trigger_time_;

    void publishEmergency(bool trigger) {
        std_msgs::Bool msg;
        msg.data = trigger;
        radar_emergency_pub_.publish(msg);
        emergency_active_ = trigger;
        last_trigger_time_ = ros::Time::now();

        if (trigger) {
            ROS_WARN("⚠️ Emergency triggered (published to /radar/emergency_trigger)");
        } else {
            ROS_INFO("✅ Emergency cleared (published to /radar/emergency_trigger)");
        }
    }

    void resetCounters() {
        consecutive_detections_ = 0;
        consecutive_clear_frames_ = 0;

        visualization_msgs::Marker clear_marker;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;

        visualization_msgs::MarkerArray clear_array;
        clear_array.markers.push_back(clear_marker);
        marker_pub_.publish(clear_array);
    }

    void publishMarkers(const std::vector<pcl::PointIndices>& clusters,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        const std_msgs::Header& header) {
        visualization_msgs::MarkerArray markers;
        int id = 0;

        for (const auto& indices : clusters) {
            float cx = 0, cy = 0, cz = 0;
            for (int idx : indices.indices) {
                cx += cloud->points[idx].x;
                cy += cloud->points[idx].y;
                cz += cloud->points[idx].z;
            }
            size_t count = indices.indices.size();
            if (count == 0) continue;

            cx /= count;
            cy /= count;
            cz /= count;

            visualization_msgs::Marker marker;
            marker.header = header;
            marker.ns = "radar_obstacles";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = cx;
            marker.pose.position.y = cy;
            marker.pose.position.z = cz;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration(0.2);
            markers.markers.push_back(marker);
        }

        marker_pub_.publish(markers);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "radar_obstacle_detection");
    RadarObstacleNode node;
    ros::spin();
    return 0;
}

