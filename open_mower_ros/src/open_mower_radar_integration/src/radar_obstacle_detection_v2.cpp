// IMPORTS / IMPORDID
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

// USED MATERIALS / KASUTATUD MATERJALID:
// General / Üldine:
// 1) ApexAI radar hands-on: https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/09_Perception_Radar/Radar-Hands-On.md
// 2) Radar clustering & PCL: https://www.youtube.com/watch?v=6RSNjo3kMMM&ab_channel=ROBOMECHTRIX
// 3) ROS PCL Handbook: https://wiki.ros.org/pcl/Handbook

// // ── ROS Publisher/Subscriber Tutorials / ROS Publisheri ja Subscriberi õppevideod ──
// 4) ROS Publisher: https://www.youtube.com/watch?v=6RSNjo3kMMM&ab_channel=ROBOMECHTRIX
// 5) ROS Subscriber: https://www.youtube.com/watch?v=vtoJg7LbxJI&ab_channel=ROBOMECHTRIX
// 6) Pub/Sub combined: https://www.youtube.com/watch?v=Y-bFqQQ3lDs&ab_channel=ROBOMECHTRIX

// // ── PCL + ROS Integration Examples / PCL ja ROS integreerimise näited ──
// 7) perception_pcl: https://github.com/ros-perception/perception_pcl/tree/ros2
// 8) Cluster Segmentation: https://github.com/jupidity/PCL-ROS-cluster-Segmentation

// ── Euclidean Cluster Extraction / Eukleidiline klastrite eraldamine ──
// 9) Official PCL tutorial: https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html
// 10) ROS Node for Cluster Based Segmentation with PCL: https://github.com/jupidity/PCL-ROS-cluster-Segmentation

// Create RadarObstacle Node class / RadarObstacleNode klassi loomine
class RadarObstacleNode {
public:
    RadarObstacleNode() {
        ros::NodeHandle nh;
        
        // Subscribe to point cloud data from mmWave radar
        // Subscriber mmWave radari punktipilve andmete saamiseks
        sub_ = nh.subscribe("/zero/ti_mmwave/radar_scan_pcl", 1, &RadarObstacleNode::pointCloudCallback, this);
        
        // Publisher for visualization markers (used in RViz)
        // Publisher visualiseerimismarkerite jaoks (RViz-is kasutamiseks)
        marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("radar_obstacles_markers", 1);
        
        // Publisher to indicate whether an obstacle was detected
        // Publisher, mis teatab, kas takistus tuvastati
        detection_state_pub_ = nh.advertise<std_msgs::Bool>("/radar/obstacle_detected", 1);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // Convert ROS PointCloud2 message to PCL format
        // Teisenda ROS PointCloud2 sõnum PCL-i formaati
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        // Remove any NaN points from the cloud
        // Eemalda punktipilvest kõik NaN väärtuse
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        // Filter out points beyond 2.0 meters
        // Filtreeri välja kõik punktid, mis asuvad kaugemal kui 2 meetrit
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : cloud->points) {
            if (std::hypot(point.x, point.y) < 2.0) {
                filtered->points.push_back(point);
            }
        }
        // If no points are left, no obstacle detected
        // Kui filtreerimise järel punkte ei jäänud, siis takistusi ei tuvastatud
        if (filtered->empty()) {
            publishDetectionState(false);
            return;
        }
        
        // Set up a k-d tree for clustering
        // Seab üles k-d puu, mida kasutatakse klastrite moodustamiseks
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(filtered);

        // Performs Euclidean Cluster Extraction to find distinct obstacles
        // Teostab eukleidilise klastrite eraldamise erinevate takistuste tuvastamiseks
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.5);
        ec.setMinClusterSize(3);
        ec.setMaxClusterSize(100);
        ec.setSearchMethod(tree);
        ec.setInputCloud(filtered);
        ec.extract(cluster_indices);

        // Publish markers and detection state
        // Avalda markerid (visualiseerimiseks) ja tuvastuse olek
        publishMarkers(cluster_indices, filtered, msg->header);
        publishDetectionState(!cluster_indices.empty());
    }

private:
    ros::Subscriber sub_; // Subscriber for radar point cloud / mmWave radari punktipilve subscriber
    ros::Publisher marker_pub_; // Publisher for RViz markers / RViz-i markerite publisher
    ros::Publisher detection_state_pub_; // Publisher for detection state / Tuvastuse oleku publisher

    // Publish a boolean message to indicate detection state
    // Avalda boolean sõnum, mis näitab kas takistus on tuvastatud
    void publishDetectionState(bool detected) {
        std_msgs::Bool msg;
        msg.data = detected;
        detection_state_pub_.publish(msg);
    }

    // Publish visualization markers for each cluster
    // Avalda iga klastri jaoks visualiseerimismarker
    void publishMarkers(const std::vector<pcl::PointIndices>& clusters,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        const std_msgs::Header& header) {
        visualization_msgs::MarkerArray markers;
        int id = 0;

        for (const auto& indices : clusters) {
            // Compute centroid of the cluster
            // Arvuta klastri keskkoht
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
            
            // Create a red sphere marker at the centroid
            // Loob punase kerakujuline marker klastri keskpunkti
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
            marker.lifetime = ros::Duration(0.2); // short-lived marker / lühiajaline marker
            markers.markers.push_back(marker);
        }
        // Publish markers for RViz
        // Avalda markerid RViz-is visualiseerimiseks
        marker_pub_.publish(markers);
    }
};

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "radar_obstacle_detection");
    RadarObstacleNode node;
    ros::spin(); // Keep it continous / Hoia node pidevalt töös
    return 0;
}

