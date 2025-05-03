#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include "cev_msgs/msg/sensor_collect.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
using std::placeholders::_1;
class AutobrakeNode : public rclcpp::Node {
public:
    AutobrakeNode(): Node("autobrake"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
            std::bind(&AutobrakeNode::publishVelocity, this));
        sensor_collect_sub_ = this->create_subscription<cev_msgs::msg::SensorCollect>(
            "sensor_collect", 1, std::bind(&AutobrakeNode::updateSensor, this, _1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 1,
            std::bind(&AutobrakeNode::updateLidar, this, _1));
    }
private:  // TODO: Make these constants configurable and use transforms instead of direct lidar
    const float VEHICLE_LENGTH = 0.185;
    const float VEHICLE_WIDTH = 0.18;
    const float MAX_VEL = 1.5;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<cev_msgs::msg::SensorCollect>::SharedPtr sensor_collect_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr rc_movement_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr forward_brake_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    sensor_msgs::msg::LaserScan::SharedPtr lidar_data_;
    float velocity_;
    float steering_angle_;

    void updateLidar(sensor_msgs::msg::LaserScan::SharedPtr data) {
        lidar_data_=data;
    }

    void updateSensor(const cev_msgs::msg::SensorCollect::SharedPtr data) {
        velocity_ = data->velocity;
        steering_angle_ = data->steering_angle;
    }

    void publishVelocity() {
        float distance = stopDistance(lidar_data_);
        float velocity = safeVelocity(distance);
        // publish
    }

    float stopDistance(const sensor_msgs::msg::LaserScan::SharedPtr data) {
        // tune these
        float base_buffer = 5.0;
        float look_ahead_distance = std::min(
            data->range_max,
            static_cast<float>(std::max(VEHICLE_LENGTH * 1.5, std::abs(velocity_) * 1.8))
        );
        float min_safe_distance = VEHICLE_LENGTH * 0.75;
        float half_length = VEHICLE_LENGTH / 2.0;
        float half_width = VEHICLE_WIDTH / 2.0;
        
        float closest_arc = std::numeric_limits<float>::max(); 

        geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_.lookupTransform("base_link", "laser", tf2::TimePointZero);
            for (int i = 0; i < data->ranges.length; i++) {
                float angle = data->angle_min + i*data->angle_increment + tf_buffer_.

                if (std::abs(steering_angle_)<0.1) {
                    // dir = 1 if self.velocity >= 0 else -1
                    // for px, py in self.lidar_points:
                    //     dx, dy = px - self.x, py - self.y
                    //     arc = (dx*ch + dy*sh) * dir
                    //     lateral = abs(-dx*sh + dy*ch)
                    //     if 0 <= arc < look_ahead_distance and lateral <= (half_width + base_buffer):
                    //         closest_arc = min(closest_arc, arc)
                } else {
                // calculate center
                // transform via laser to base_link
                // do calculations based on new center of mass
                return -1.0;
                }
            }
    }

    float safeVelocity(float distance) {
        if closest_arc == float('inf'):
        return self.max_velocity
        if closest_arc <= min_safe_distance:
        return 0.0

        ratio = (closest_arc - min_safe_distance / 2.0) / (look_ahead_distance - min_safe_distance / 2.0)
        ratio = max(0.0, min(1.0, ratio))
        factor = -0.8 * ratio**3 + 1.65 * ratio**2 + 0.15 * ratio
        factor = max(0.0, min(1.0, factor))
    
        return self.max_velocity * factor * (1 if self.velocity >= 0 else -1)
    }
}
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutobrakeNode>());
    rclcpp::shutdown();
    return 0;
}