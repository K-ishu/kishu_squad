#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

struct Waypoint
{
    double x;
    double y;
    double z;      // PX4 NED: negative altitude
    double yaw;
};

class GoToPoint : public rclcpp::Node
{
public:
    GoToPoint() : Node("go_to_point")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_profile.history, 5),
            qos_profile
        );

        local_position_sub_ = create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1",
            qos,
            std::bind(&GoToPoint::localPositionCallback, this, std::placeholders::_1)
        );

        attitude_sub_ = create_subscription<VehicleAttitude>(
            "/fmu/out/vehicle_attitude",
            qos,
            std::bind(&GoToPoint::attitudeCallback, this, std::placeholders::_1)
        );

        offboard_mode_pub_ =
            create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

        trajectory_pub_ =
            create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        vehicle_command_pub_ =
            create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        initializeWaypoints();

        timer_ = create_wall_timer(20ms, std::bind(&GoToPoint::timerCallback, this));

        RCLCPP_INFO(get_logger(), "Professional 7+ waypoint offboard trajectory planner started.");
    }

private:
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_sub_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    VehicleLocalPosition current_position_{};
    VehicleAttitude current_attitude_{};

    bool position_received_{false};
    bool offboard_started_{false};
    bool mission_finished_{false};
    bool land_command_sent_{false};
    bool landing_started_{false};

    int setpoint_counter_{0};

    std::vector<Waypoint> waypoints_;

    const double dt_{0.02};
    const double total_time_{56.0};
    double t_{0.0};

    void initializeWaypoints()
    {
        waypoints_ = {
            {0.0,   0.0,  -5.0,  0.0},
            {5.0,   0.0,  -5.0,  0.0},
            {7.0,   4.0,  -6.0,  0.7},
            {3.0,   8.0,  -6.0,  1.57},
            {-2.0,  7.0,  -5.0,  2.2},
            {-6.0,  3.0,  -5.0,  3.14},
            {-4.0, -3.0,  -6.0, -2.2},
            {0.0,   0.0,  -5.0,  0.0}
        };
    }

    void localPositionCallback(const VehicleLocalPosition::SharedPtr msg)
    {
        current_position_ = *msg;
        position_received_ = true;
    }

    void attitudeCallback(const VehicleAttitude::SharedPtr msg)
    {
        current_attitude_ = *msg;
    }

    void timerCallback()
    {
        if (landing_started_) {
            return;
        }

        publishOffboardControlMode();

        if (!position_received_) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Waiting for vehicle_local_position..."
            );
            return;
        }

        if (setpoint_counter_ < 50) {
            publishHoldSetpoint();
            setpoint_counter_++;
            return;
        }

        if (!offboard_started_) {
            engageOffboardMode();
            arm();
            offboard_started_ = true;
            RCLCPP_INFO(get_logger(), "Offboard mode requested and vehicle armed.");
        }

        if (!mission_finished_) {
            publishTrajectory();
        }
    }

    void publishOffboardControlMode()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = true;
        msg.acceleration = true;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = now().nanoseconds() / 1000;
        offboard_mode_pub_->publish(msg);
    }

    void publishHoldSetpoint()
    {
        TrajectorySetpoint msg{};
        msg.position = {
            static_cast<float>(current_position_.x),
            static_cast<float>(current_position_.y),
            static_cast<float>(current_position_.z)
        };
        msg.velocity = {0.0f, 0.0f, 0.0f};
        msg.acceleration = {0.0f, 0.0f, 0.0f};
        msg.yaw = 0.0f;
        msg.yawspeed = 0.0f;
        msg.timestamp = now().nanoseconds() / 1000;
        trajectory_pub_->publish(msg);
    }

    void publishTrajectory()
    {
        Waypoint p{};
        Waypoint v{};
        Waypoint a{};

        computeCatmullRomTrajectory(t_, p, v, a);

        TrajectorySetpoint msg{};
        msg.position = {
            static_cast<float>(p.x),
            static_cast<float>(p.y),
            static_cast<float>(p.z)
        };
        msg.velocity = {
            static_cast<float>(v.x),
            static_cast<float>(v.y),
            static_cast<float>(v.z)
        };
        msg.acceleration = {
            static_cast<float>(a.x),
            static_cast<float>(a.y),
            static_cast<float>(a.z)
        };
        msg.yaw = static_cast<float>(p.yaw);
        msg.yawspeed = static_cast<float>(v.yaw);
        msg.timestamp = now().nanoseconds() / 1000;

        trajectory_pub_->publish(msg);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Trajectory t=%.2f s | pos=[%.2f %.2f %.2f] | vel_norm=%.2f | acc_norm=%.2f | yaw=%.2f",
            t_, p.x, p.y, p.z,
            std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z),
            std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z),
            p.yaw
        );

        t_ += dt_;

        if (t_ >= total_time_) {
            mission_finished_ = true;

            if (!land_command_sent_) {
                publishFinalStop();
                land();
                land_command_sent_ = true;
                landing_started_ = true;
            }

            RCLCPP_INFO(get_logger(), "Trajectory completed. LAND command sent. Offboard setpoints stopped.");
        }
    }

    void publishFinalStop()
    {
        const Waypoint &p = waypoints_.back();

        TrajectorySetpoint msg{};
        msg.position = {
            static_cast<float>(p.x),
            static_cast<float>(p.y),
            static_cast<float>(p.z)
        };
        msg.velocity = {0.0f, 0.0f, 0.0f};
        msg.acceleration = {0.0f, 0.0f, 0.0f};
        msg.yaw = static_cast<float>(p.yaw);
        msg.yawspeed = 0.0f;
        msg.timestamp = now().nanoseconds() / 1000;

        trajectory_pub_->publish(msg);
    }

    void land()
    {
        VehicleCommand msg{};
        msg.command = VehicleCommand::VEHICLE_CMD_NAV_LAND;
        msg.param1 = 0.0f;
        msg.param2 = 0.0f;
        msg.param3 = 0.0f;
        msg.param4 = NAN;
        msg.param5 = NAN;
        msg.param6 = NAN;
        msg.param7 = NAN;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);

        RCLCPP_WARN(get_logger(), "Trajectory finished. LAND command sent. Offboard setpoints stopped.");
    }

    void computeCatmullRomTrajectory(double t, Waypoint &p, Waypoint &v, Waypoint &a)
    {
        const int n = static_cast<int>(waypoints_.size());
        const int segments = n - 1;
        const double segment_time = total_time_ / static_cast<double>(segments);

        int i = static_cast<int>(std::floor(t / segment_time));
        i = std::clamp(i, 0, segments - 1);

        double tau = (t - i * segment_time) / segment_time;
        tau = std::clamp(tau, 0.0, 1.0);

        int i0 = std::max(i - 1, 0);
        int i1 = i;
        int i2 = i + 1;
        int i3 = std::min(i + 2, n - 1);

        catmullRom(waypoints_[i0], waypoints_[i1], waypoints_[i2], waypoints_[i3],
                   tau, segment_time, p, v, a);

        if (t >= total_time_ - dt_) {
            p = waypoints_.back();
            v = {0.0, 0.0, 0.0, 0.0};
            a = {0.0, 0.0, 0.0, 0.0};
        }
    }

    void catmullRom(
        const Waypoint &p0, const Waypoint &p1,
        const Waypoint &p2, const Waypoint &p3,
        double u, double segment_time,
        Waypoint &p, Waypoint &v, Waypoint &a)
    {
        p.x = catmullPosition(p0.x, p1.x, p2.x, p3.x, u);
        p.y = catmullPosition(p0.y, p1.y, p2.y, p3.y, u);
        p.z = catmullPosition(p0.z, p1.z, p2.z, p3.z, u);
        p.yaw = catmullPosition(p0.yaw, p1.yaw, p2.yaw, p3.yaw, u);

        v.x = catmullVelocity(p0.x, p1.x, p2.x, p3.x, u) / segment_time;
        v.y = catmullVelocity(p0.y, p1.y, p2.y, p3.y, u) / segment_time;
        v.z = catmullVelocity(p0.z, p1.z, p2.z, p3.z, u) / segment_time;
        v.yaw = catmullVelocity(p0.yaw, p1.yaw, p2.yaw, p3.yaw, u) / segment_time;

        a.x = catmullAcceleration(p0.x, p1.x, p2.x, p3.x, u) / (segment_time * segment_time);
        a.y = catmullAcceleration(p0.y, p1.y, p2.y, p3.y, u) / (segment_time * segment_time);
        a.z = catmullAcceleration(p0.z, p1.z, p2.z, p3.z, u) / (segment_time * segment_time);
        a.yaw = catmullAcceleration(p0.yaw, p1.yaw, p2.yaw, p3.yaw, u) / (segment_time * segment_time);
    }

    double catmullPosition(double p0, double p1, double p2, double p3, double u)
    {
        return 0.5 * (
            2.0 * p1 +
            (-p0 + p2) * u +
            (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u * u +
            (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u * u * u
        );
    }

    double catmullVelocity(double p0, double p1, double p2, double p3, double u)
    {
        return 0.5 * (
            (-p0 + p2) +
            2.0 * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u +
            3.0 * (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u * u
        );
    }

    double catmullAcceleration(double p0, double p1, double p2, double p3, double u)
    {
        return 0.5 * (
            2.0 * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) +
            6.0 * (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u
        );
    }

    void engageOffboardMode()
    {
        VehicleCommand msg{};
        msg.param1 = 1.0;
        msg.param2 = 6.0;
        msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
    }

    void arm()
    {
        VehicleCommand msg{};
        msg.param1 = 1.0;
        msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting professional 7+ waypoint offboard trajectory planner..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoToPoint>());
    rclcpp::shutdown();
    return 0;
}
