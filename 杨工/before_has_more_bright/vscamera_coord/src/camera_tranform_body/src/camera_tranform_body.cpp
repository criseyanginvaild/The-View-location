#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Dense>
#include <sstream>
#include <string>
#include <std_msgs/String.h>


class CoordinateTransformer {
public:
    CoordinateTransformer() {
        // 初始化ROS节点句柄
        ros::NodeHandle nh;

        // 订阅相机系下的点坐标
        camera_coordinates_sub_ = nh.subscribe("camera_coord_pos", 1000, &CoordinateTransformer::cameraCoordinatesCallback, this);

        // 发布机体坐标系下的点坐标
        body_coordinates_pub_ = nh.advertise<geometry_msgs::PointStamped>("body_coordinates", 1000);

        // 定义旋转矩阵和平移向量
        rotationMatrix = (Eigen::Matrix3d() << 0,-0.5,-0.866,1,0,0,0,-0.866,0.5).finished();
        translationVector = (Eigen::Vector3d() << -0.41, -1.35, 1.99).finished();
    }

    void cameraCoordinatesCallback(const std_msgs::String::ConstPtr& msg) {
            std::istringstream iss(msg->data);
            double x, y, z;
            iss >> x >> y >> z;

            Eigen::Vector3d point_camera(x/1000, z/1000, -y/1000);

            Eigen::Vector3d point_body = rotationMatrix * point_camera + translationVector;

            geometry_msgs::PointStamped point_body_msg;
            point_body_msg.header.stamp = ros::Time::now();
            point_body_msg.header.frame_id = "body_frame";
            point_body_msg.point.x = -point_body.y();
            point_body_msg.point.y = point_body.x();
            point_body_msg.point.z = point_body.z();

            std::cout << "body point left hand" <<point_body.x() << point_body.y() <<  std::endl;

            body_coordinates_pub_.publish(point_body_msg);
    }

private:
    ros::Subscriber camera_coordinates_sub_;
    ros::Publisher body_coordinates_pub_;
    Eigen::Matrix3d rotationMatrix;
    Eigen::Vector3d translationVector;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "coordinate_transformer");

    CoordinateTransformer transformer;

    ros::spin();

    return 0;
}
