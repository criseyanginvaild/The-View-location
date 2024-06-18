#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>

class CoordinatesTransformer {
public:
    CoordinatesTransformer(ros::NodeHandle& nh)
        : nh_(nh), tfListener_(tfBuffer_) {
        // 订阅车体坐标系下的坐标
        bodyCoordsSub_ = nh_.subscribe("/body_coordinates", 1000, &CoordinatesTransformer::bodyCoordsCallback, this);
        // 发布转换后的世界坐标系下的坐标
        worldCoordsPub_ = nh_.advertise<geometry_msgs::PointStamped>("/world_coordinates", 1000);
    }

    void bodyCoordsCallback(const geometry_msgs::PointStamped& msg) {
        try {
            // 查找转换关系
            geometry_msgs::TransformStamped transformStamped = tfBuffer_.lookupTransform("map", "base_link", ros::Time(0));

            // 将PointStamped转换为Eigen::Vector4f
            Eigen::Vector4f ptVeh(msg.point.x, msg.point.y, msg.point.z, 1.0);

            // 应用转换
            Eigen::Matrix4f transMat = tf2::transformToEigen(transformStamped.transform).matrix().cast<float>();
            Eigen::Vector4f ptGrd = transMat * ptVeh;

            // 发布转换后的坐标
            geometry_msgs::PointStamped ptWorld;
            ptWorld.header.stamp = ros::Time::now();
            ptWorld.header.frame_id = "world_map";
            ptWorld.point.x = ptGrd[0];
            ptWorld.point.y = ptGrd[1];
            ptWorld.point.z = ptGrd[2];
            worldCoordsPub_.publish(ptWorld);
            
            ROS_INFO_STREAM("ptWorld:" << "x:" <<ptWorld.point.x << ", y:" << ptWorld.point.y << ", z:" << ptWorld.point.z);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber bodyCoordsSub_;
    ros::Publisher worldCoordsPub_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "coordinates_transform_node");
    ros::NodeHandle nh;

    CoordinatesTransformer transformer(nh);

    ros::spin();
    return 0;
}

