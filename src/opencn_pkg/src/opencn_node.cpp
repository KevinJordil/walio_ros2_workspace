#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "opencn_communication_interfaces/srv/pins.hpp"
#include "opencn_communication_interfaces/srv/init.hpp"

#include <capnp/ez-rpc.h>

#include "opencn_pkg/component_specific.h"
#include "opencn_pkg/transaction.h"

unsigned int Lcec_epos4::_numLcec = 0;

class OpencnPkg : public rclcpp::Node
{

public:
    OpencnPkg() : Node("opencn_pkg") {
        pins_service = this->create_service<opencn_communication_interfaces::srv::Pins>("opencn_pins",
            std::bind(&OpencnPkg::pins_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        init_service = this->create_service<opencn_communication_interfaces::srv::Init>("opencn_init",
            std::bind(&OpencnPkg::init_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:

    void init_callback(const std::shared_ptr<opencn_communication_interfaces::srv::Init::Request> request,
                       const std::shared_ptr<opencn_communication_interfaces::srv::Init::Response> response)
    {
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Init callback");
        this->client = new capnp::EzRpcClient(request->ip_address, request->port);

        // Check if device is EPoS4
        if(request->device == "EPOS4")
        {
            //Components Initialisation
            MuxNTo1F mux("target", client, 4);
            SimplePG simple_pg(client, 3);
            Lcec_epos4 epos4("EPOS4", client);
        }
        else
        {
            delete client;
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown device");
            response->success = false;
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Init callback success");
        response->success = true;
    }

    void pins_callback(const std::shared_ptr<opencn_communication_interfaces::srv::Pins::Request> request,
                       const std::shared_ptr<opencn_communication_interfaces::srv::Pins::Response> response) const
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pins callback");
        // Copy pins to response
        response->pins = request->pins;
        // Initialize transactions
        Transactions transactions(client);
        // Create an array of openCN pins pointer to get results after transactions
        CMPin *opencn_pins[request->pins.size()];

        // Define each pin
        for(int i = 0; i < request->pins.size(); i++)
        {
            auto &pin = response->pins[i];

            if(pin.pin_class == opencn_communication_interfaces::msg::Pin::CMPINI32){
                CMPinI32 *cmpini32 = new CMPinI32(pin.name, client);
                if(pin.transaction_type == opencn_communication_interfaces::msg::Pin::SET) {
                    transactions.add(cmpini32, TransactionType::SET);
                    *cmpini32 = pin.cmpini32.value;
                } else if (pin.transaction_type == opencn_communication_interfaces::msg::Pin::GET)
                    transactions.add(cmpini32, TransactionType::GET);
                else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid transaction type");
                    response->success = false;
                    return;
                }
                opencn_pins[i] = cmpini32;
            } else if (pin.pin_class == opencn_communication_interfaces::msg::Pin::CMPINU32){
                CMPinU32* cmpinu32 = new CMPinU32(pin.name, client);
                if(pin.transaction_type == opencn_communication_interfaces::msg::Pin::SET) {
                    transactions.add(cmpinu32, TransactionType::SET);
                    *cmpinu32 = pin.cmpinu32.value;
                } else if (pin.transaction_type == opencn_communication_interfaces::msg::Pin::GET)
                    transactions.add(cmpinu32, TransactionType::GET);
                else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid transaction type");
                    response->success = false;
                    return;
                }
                opencn_pins[i] = cmpinu32;
            } else if (pin.pin_class == opencn_communication_interfaces::msg::Pin::CMPINBIT){
                CMPinBit* cmpinbit = new CMPinBit(pin.name, client);
                if(pin.transaction_type == opencn_communication_interfaces::msg::Pin::SET) {
                    transactions.add(cmpinbit, TransactionType::SET);
                    *cmpinbit = pin.cmpinbit.value;
                } else if (pin.transaction_type == opencn_communication_interfaces::msg::Pin::GET)
                    transactions.add(cmpinbit, TransactionType::GET);
                else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid transaction type");
                    response->success = false;
                    return;
                }
                opencn_pins[i] = cmpinbit;
            } else if (pin.pin_class == opencn_communication_interfaces::msg::Pin::CMPINFLOAT){
                CMPinFloat* cmpinfloat = new CMPinFloat(pin.name, client);
                if(pin.transaction_type == opencn_communication_interfaces::msg::Pin::SET) {
                    transactions.add(cmpinfloat, TransactionType::SET);
                    *cmpinfloat = pin.cmpinfloat.value;
                } else if (pin.transaction_type == opencn_communication_interfaces::msg::Pin::GET)
                    transactions.add(cmpinfloat, TransactionType::GET);
                else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid transaction type");
                    response->success = false;
                    return;
                }
                opencn_pins[i] = cmpinfloat;
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown pin class");
                response->success = false;
                return;
            }
        }

        transactions.execute();

        // Update pin value for GET transaction type from opencn_pins to response
        for (int i = 0; i < response->pins.size(); i++) {
            if (response->pins[i].transaction_type == opencn_communication_interfaces::msg::Pin::GET) {
                if(response->pins[i].pin_class == opencn_communication_interfaces::msg::Pin::CMPINI32){
                    response->pins[i].cmpini32.value = ((CMPinI32*)opencn_pins[i])->get();
                } else if (response->pins[i].pin_class == opencn_communication_interfaces::msg::Pin::CMPINU32){
                    response->pins[i].cmpinu32.value = ((CMPinU32*)opencn_pins[i])->get();
                } else if (response->pins[i].pin_class == opencn_communication_interfaces::msg::Pin::CMPINBIT){
                    response->pins[i].cmpinbit.value = ((CMPinBit*)opencn_pins[i])->get();
                } else if (response->pins[i].pin_class == opencn_communication_interfaces::msg::Pin::CMPINFLOAT){
                    response->pins[i].cmpinfloat.value = ((CMPinFloat*)opencn_pins[i])->get();
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown pin class");
                    response->success = false;
                    return;
                }
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pins callback success");
        response->success = true;
    }

    rclcpp::Service<opencn_communication_interfaces::srv::Pins>::SharedPtr pins_service;
    rclcpp::Service<opencn_communication_interfaces::srv::Init>::SharedPtr init_service;
    
    capnp::EzRpcClient *client;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing opencn_pkg node");

    rclcpp::spin(std::make_shared<OpencnPkg>());
    rclcpp::shutdown();

    return 0;
}