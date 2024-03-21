#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
//#include "sensor_msgs/msg/joy.hpp"
#include "opencn_communication_interfaces/srv/pins.hpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing MotionControlNode");

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("motion_control_node");

    rclcpp::Client<opencn_communication_interfaces::srv::Pins>::SharedPtr client = node->create_client<opencn_communication_interfaces::srv::Pins>("opencn_pins");

    auto request = std::make_shared<opencn_communication_interfaces::srv::Pins::Request>();

    // Array of Pin
    std::vector<opencn_communication_interfaces::msg::Pin> pins;

    opencn_communication_interfaces::msg::Pin pin1;
    pin1.pin_class = opencn_communication_interfaces::msg::Pin::CMPINI32;
    pin1.cmpini32.value = 42;
    pin1.name = "CMPini32 test pin";
    pin1.transaction_type = opencn_communication_interfaces::msg::Pin::SET;
    pins.push_back(pin1);

    opencn_communication_interfaces::msg::Pin pin2;
    pin2.pin_class = opencn_communication_interfaces::msg::Pin::CMPINBIT;
    pin2.name = "CMPinBit test pin";
    pin2.transaction_type = opencn_communication_interfaces::msg::Pin::GET;
    pins.push_back(pin2);

    request->pins = pins;

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto future_result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, future_result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        // Get result as pointer due to a ROS bug (https://github.com/ros2/rclcpp/issues/1968)
        auto result = *future_result.get();
        // Check if the service call was successful.
        if (result.success) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call was successful");
            // Print pins size
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pins size: %ld", result.pins.size());
            // Print value of each pin with GET transaction type
            for (long unsigned int i = 0; i < result.pins.size(); i++) {
                // Print the name of the pin
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pin name: %s", result.pins[i].name.c_str());
                if (result.pins[i].transaction_type == opencn_communication_interfaces::msg::Pin::GET) {
                    // Print the value of the pin depending on its class
                    switch (result.pins[i].pin_class) {
                        case opencn_communication_interfaces::msg::Pin::CMPINI32:
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CMPINI32 pin value: %d", result.pins[i].cmpini32.value);
                            break;
                        case opencn_communication_interfaces::msg::Pin::CMPINU32:
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CMPINU32 pin value: %d", result.pins[i].cmpinu32.value);
                            break;
                        case opencn_communication_interfaces::msg::Pin::CMPINBIT:
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CMPINBIT pin value: %d", result.pins[i].cmpinbit.value);
                            break;
                        case opencn_communication_interfaces::msg::Pin::CMPINFLOAT:
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CMPINFLOAT pin value: %f", result.pins[i].cmpinfloat.value);
                            break;
                        default:
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown pin class");
                    }
                }
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call was not successful");
        }


    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
    }


    rclcpp::shutdown();

    return 0;
}
