#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

using namespace std::chrono_literals;

class ForceLand : public rclcpp::Node
{
public:
    ForceLand()
    : Node("force_land"),
      need_land(false),
      landing_procedure_started(false),
      vehicle_landed(false)
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_profile.history, 5),
            qos_profile
        );

        local_position_subscription_ =
            this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position_v1",
                qos,
                std::bind(&ForceLand::height_callback, this, std::placeholders::_1)
            );

        land_detected_subscription_ =
            this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
                "/fmu/out/vehicle_land_detected",
                qos,
                std::bind(&ForceLand::land_detected_callback, this, std::placeholders::_1)
            );

        publisher_ =
            this->create_publisher<px4_msgs::msg::VehicleCommand>(
                "/fmu/in/vehicle_command",
                10
            );

        timer_ =
            this->create_wall_timer(
                10ms,
                std::bind(&ForceLand::activate_switch, this)
            );
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_subscription_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool need_land;
    bool landing_procedure_started;
    bool vehicle_landed;

    void height_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
    {
        float altitude_enu = -msg->z;

        std::cout << "Current drone height: "
                  << altitude_enu
                  << " meters"
                  << std::endl;

        if (altitude_enu >= 20.0f && !landing_procedure_started)
        {
            need_land = true;
        }
    }

    void land_detected_callback(const px4_msgs::msg::VehicleLandDetected::UniquePtr msg)
    {
        vehicle_landed = msg->landed;

        if (vehicle_landed)
        {
            landing_procedure_started = false;
            need_land = false;

            std::cout << "Landing completed. Force-land trigger is re-enabled."
                      << std::endl;
        }
    }

    void activate_switch()
    {
        if (need_land && !landing_procedure_started)
        {
            std::cout << "Drone height exceeded 20 meters threshold. Landing forced."
                      << std::endl;

            auto command = px4_msgs::msg::VehicleCommand();

            command.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
            command.param1 = 0.0;
            command.param2 = 0.0;
            command.target_system = 1;
            command.target_component = 1;
            command.source_system = 1;
            command.source_component = 1;
            command.from_external = true;

            publisher_->publish(command);

            need_land = false;
            landing_procedure_started = true;
        }
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting force_land node..." << std::endl;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceLand>());
    rclcpp::shutdown();

    return 0;
}
