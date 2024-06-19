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

        std::vector<double> rotationMatrix_data;
        std::vector<double> translationvector_data;

        // For a 3x3 matrix, we expect 9 elements
        if (!nh.getParam("camera_tranform_body_node/rotation_matrix/data", rotationMatrix_data) || rotationMatrix_data.size() != 9) {
            ROS_ERROR("Failed to get 'rotation_matrix/data' or size is not 9");
            throw std::runtime_error("Failed to get 'rotation_matrix/data' or size is not 9");
        }


        // For a translation vector, we expect 3 elements
        if (!nh.getParam("camera_tranform_body_node/translation_vector/data", translationvector_data) || translationvector_data.size() != 3) {
            ROS_ERROR("Failed to get 'translation_vector/data' or size is not 3");
            throw std::runtime_error("Failed to get 'translation_vector/data' or size is not 3");
        }

        // Use the data read from the parameters to populate the Eigen structures
        rotationMatrix << rotationMatrix_data[0], rotationMatrix_data[1], rotationMatrix_data[2],
                        rotationMatrix_data[3], rotationMatrix_data[4], rotationMatrix_data[5],
                        rotationMatrix_data[6], rotationMatrix_data[7], rotationMatrix_data[8];

        translationVector << translationvector_data[0], translationvector_data[1], translationvector_data[2];

    }


    void cameraCoordinatesCallback(const std_msgs::String::ConstPtr& msg) {
            std::istringstream iss(msg->data);
            double x, y, z;
            iss >> x >> y >> z;

            Eigen::Vector3d point_camera(x/1000, z/1000, -y/1000);

            Eigen::Vector3d point_body = rotationMatrix * point_camera + translationVector;

            std::cout <<  rotationMatrix << std::endl;
            std::cout <<  translationVector << std::endl;

            geometry_msgs::PointStamped point_body_msg;
            point_body_msg.header.stamp = ros::Time::now();
            point_body_msg.header.frame_id = "body_frame";
            point_body_msg.point.x = -point_body.y();
            point_body_msg.point.y = point_body.x();
            point_body_msg.point.z = point_body.z();

            std::cout << "please add those data to your fork_vehicle.yaml file" << 
            " " << -point_body.x() << " " << -point_body.y() << " " << -point_body.z() << std::endl;

            // std::cout << "body_coordinate can get the the camera point : " << point_body_msg.point.x
            // << " " << point_body_msg.point.y << " " << point_body_msg.point.z  <<  std::endl;

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
