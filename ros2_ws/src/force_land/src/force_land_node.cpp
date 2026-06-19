#include <chrono>
#include <functional>
#include <memory>

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
      state_(State::IDLE),
      altitude_enu_(0.0f),
      vehicle_landed_(false)
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

        vehicle_command_publisher_ =
            this->create_publisher<px4_msgs::msg::VehicleCommand>(
                "/fmu/in/vehicle_command",
                10
            );

        timer_ =
            this->create_wall_timer(
                100ms,
                std::bind(&ForceLand::fsm_update, this)
            );

        RCLCPP_INFO(this->get_logger(), "Force land FSM node started.");
        RCLCPP_INFO(this->get_logger(), "Initial state: IDLE");
    }

private:
    enum class State
    {
        IDLE,
        MONITORING,
        LAND_COMMAND_SENT,
        WAITING_FOR_TOUCHDOWN
    };

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_subscription_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_;
    float altitude_enu_;
    bool vehicle_landed_;

    static constexpr float ALTITUDE_THRESHOLD_M = 20.0f;

    void height_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
    {
        altitude_enu_ = -msg->z;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Current altitude ENU: %.2f m",
            altitude_enu_
        );

        if (state_ == State::IDLE && !vehicle_landed_) {
            state_ = State::MONITORING;
            RCLCPP_INFO(this->get_logger(), "State changed: IDLE -> MONITORING");
        }
    }

    void land_detected_callback(const px4_msgs::msg::VehicleLandDetected::UniquePtr msg)
    {
        vehicle_landed_ = msg->landed;

        if (vehicle_landed_ && state_ == State::WAITING_FOR_TOUCHDOWN) {
            state_ = State::IDLE;
            RCLCPP_INFO(this->get_logger(), "Landing completed.");
            RCLCPP_INFO(this->get_logger(), "State changed: WAITING_FOR_TOUCHDOWN -> IDLE");
        }
    }

    void fsm_update()
    {
        switch (state_) {
        case State::IDLE:
            break;

        case State::MONITORING:
            if (altitude_enu_ >= ALTITUDE_THRESHOLD_M) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Altitude threshold exceeded: %.2f m >= %.2f m. Sending LAND command.",
                    altitude_enu_,
                    ALTITUDE_THRESHOLD_M
                );

                publish_land_command();
                state_ = State::LAND_COMMAND_SENT;
                RCLCPP_INFO(this->get_logger(), "State changed: MONITORING -> LAND_COMMAND_SENT");
            }
            break;

        case State::LAND_COMMAND_SENT:
            state_ = State::WAITING_FOR_TOUCHDOWN;
            RCLCPP_INFO(this->get_logger(), "State changed: LAND_COMMAND_SENT -> WAITING_FOR_TOUCHDOWN");
            break;

        case State::WAITING_FOR_TOUCHDOWN:
            /*
             * Critical homework requirement:
             * While the vehicle has not landed, the node does not re-trigger
             * the LAND command even if the pilot retakes manual control and
             * climbs above the altitude threshold again.
             */
            break;
        }
    }

    void publish_land_command()
    {
        px4_msgs::msg::VehicleCommand command{};

        command.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
        command.param1 = 0.0f;
        command.param2 = 0.0f;
        command.param3 = 0.0f;
        command.param4 = 0.0f;
        command.param5 = 0.0;
        command.param6 = 0.0;
        command.param7 = 0.0f;
        command.target_system = 1;
        command.target_component = 1;
        command.source_system = 1;
        command.source_component = 1;
        command.from_external = true;

        vehicle_command_publisher_->publish(command);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceLand>());
    rclcpp::shutdown();

    return 0;
}
