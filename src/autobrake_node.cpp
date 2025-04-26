#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include "cev_msgs/msg/sensor_collect.hpp"
#include <cmath>
class AutobrakeNode : public rclcpp::Node {
public:
    AutobrakeNode(): Node("autobrake") {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
            std::bind(&AutobrakeNode::publishBrake, this));
        sensor_collect_sub_ = this->create_subscription<cev_msgs::msg::SensorCollect>(
            "sensor_collect", 1, std::bind(&AutobrakeNode::setVars, this, _1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 1,
            std::bind(&AutobrakeNode::checkCollision, this, _1));
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

    void publishVelocity(const sensor_msgs::msg::LaserScan::SharedPtr data) {
        float distance = stopDistance(data);
        float velocity = safeVelocity(distance);
        // publish
    }

    float stopDistance(const sensor_msgs::msg::LaserScan::SharedPtr data) {
    float base_buffer = 5.0;
    float look_ahead_distance = std::min(
        self.lidar_range,
        max(self.length * 1.5, abs(self.velocity) * 1.8)
    );
    float min_safe_distance = self.length * 0.75;
    float half_length = VEHICLE_LENGTH / 2.0;
    float half_width = VEHICLE_WIDTH / 2.0;
    
    float closest_arc = std::numeric_limits<float>::max(); 

    float ch = std::cos(heading_);
    float sh = std::sin(heading_);

    if (std::abs(steering_angle_)<0.1) {
        for (const auto& p : lidar_points_) {
            // transform from /tf
            
        }
    }

    if abs(self.steering_angle) < 1e-6:
        dir = 1 if self.velocity >= 0 else -1
        for px, py in self.lidar_points:
            dx, dy = px - self.x, py - self.y
            arc = (dx*ch + dy*sh) * dir
            lateral = abs(-dx*sh + dy*ch)
            if 0 <= arc < look_ahead_distance and lateral <= (half_width + base_buffer):
                closest_arc = min(closest_arc, arc)

    # Turning
    else:
        R = self.wheelbase / math.tan(abs(self.steering_angle))
        turn_sign = 1 if self.steering_angle > 0 else -1
        # self.x, self.y are the center of the car
        # we use this reference point so the front edge of the car is entirely within the two circles
        icr_x = self.x - turn_sign * R * math.sin(self.heading)
        icr_y = self.y + turn_sign * R * math.cos(self.heading)
        inner_rad = R-half_width
        outer_rad = math.hypot(R+half_width, half_length)
        motion_dir = 1 if self.velocity >= 0 else -1
        travel_sign = turn_sign * motion_dir
        turn_factor = max(0.3, 1 - abs(self.steering_angle) / math.pi)
        buffer = base_buffer * turn_factor

        angular_span = look_ahead_distance / R
        theta0 = math.atan2(self.y - icr_y, self.x - icr_x)
        for px, py in self.lidar_points:
            point_radius = math.hypot(px-icr_x, py-icr_y)
            if (outer_rad + buffer) > point_radius > (inner_rad - buffer):
                theta_p = math.atan2(py - icr_y, px - icr_x)
                delta = (theta_p - theta0) * travel_sign % (2 * math.pi)
                arc = delta * R
                if arc < look_ahead_distance:
                    closest_arc = min(closest_arc, arc)
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

    void setVars(const cev_msgs::msg::SensorCollect::SharedPtr data) {
        velocity_ = data->velocity;
        steering_angle_ = data->steering_angle;
    }
}
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutobrakeNode>());
    rclcpp::shutdown();
    return 0;
}