#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "opencn_communication_interfaces/srv/pins.hpp"

#include <capnp/ez-rpc.h>

#include "opencn_pkg/component_specific.h"
#include "opencn_pkg/transaction.h"


#define DEFAULT_IP_PORT 7002

unsigned int Lcec_epos4::_numLcec = 0;

/*Function prototypes*/
static void sleep_ms(int time);
static void show_usage(std::string name);
int main_menu(Lcec_epos4 *epos4, MuxNTo1F *mux, SimplePG *simple_pg);
int ppm_menu(Lcec_epos4 *epos4, MuxNTo1F *mux, SimplePG *simple_pg);
int transactions_test(capnp::EzRpcClient *client);
int transactions_pins_test(capnp::EzRpcClient *client);

class OpencnPkg : public rclcpp::Node
{

public:
    OpencnPkg() : Node("opencn_pkg") {
        //joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
      //"joy", 10, std::bind(&OpencnPkg::joy_callback, this, std::placeholders::_1));

        pins_service = this->create_service<opencn_communication_interfaces::srv::Pins>("opencn_pins",
            std::bind(&OpencnPkg::pins_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    /*
    Pin :
        uint8 pin_class
        uint8 CMPINL32=0
        uint8 CMPINU32=1
        uint8 CMPINBIT=2
        uint8 CMPINFLOAT=3

        CMPinl32 cmpinl32
        CMPinU32 cmpinu32
        CMPinBit cmpinbit
        CMPinFloat cmpinfloat

        string name

        uint8 transaction_type
        uint8 GET=0
        uint8 SET=1
    */

    void pins_callback(const std::shared_ptr<opencn_communication_interfaces::srv::Pins::Request> request,
                       const std::shared_ptr<opencn_communication_interfaces::srv::Pins::Response> response) const
    {
        // Copy pins
        response->pins = request->pins;
        RCLCPP_INFO(this->get_logger(), "====Request begin====");
        for(int i = 0; i < request->pins.size(); i++)
        {
            // Pin number
            RCLCPP_INFO(this->get_logger(), "==Pin%d", i);
            // pin name 
            RCLCPP_INFO(this->get_logger(), "Name: %s", request->pins[i].name.c_str());
            // Pin class
            if (request->pins[i].pin_class == opencn_communication_interfaces::msg::Pin::CMPINL32)
            {
                RCLCPP_INFO(this->get_logger(), "Pin class: CMPINL32");
            }
            else if (request->pins[i].pin_class == opencn_communication_interfaces::msg::Pin::CMPINU32)
            {
                RCLCPP_INFO(this->get_logger(), "Pin class: CMPINU32");
            }
            else if (request->pins[i].pin_class == opencn_communication_interfaces::msg::Pin::CMPINBIT)
            {
                RCLCPP_INFO(this->get_logger(), "Pin class: CMPINBIT");
            }
            else if (request->pins[i].pin_class == opencn_communication_interfaces::msg::Pin::CMPINFLOAT)
            {
                RCLCPP_INFO(this->get_logger(), "Pin class: CMPINFLOAT");
            }
            // Transaction type
            if (request->pins[i].transaction_type == opencn_communication_interfaces::msg::Pin::GET)
            {
                RCLCPP_INFO(this->get_logger(), "Transaction type: GET");

                // Set value
                if(request->pins[i].pin_class == opencn_communication_interfaces::msg::Pin::CMPINL32)
                {
                    RCLCPP_INFO(this->get_logger(), "Type CMPINL32");
                    response->pins[i].cmpinl32.value = 41;
                }
                else if(request->pins[i].pin_class == opencn_communication_interfaces::msg::Pin::CMPINU32)
                {
                    RCLCPP_INFO(this->get_logger(), "Type CMPINU32");
                    response->pins[i].cmpinu32.value = 41;
                }
                else if(request->pins[i].pin_class == opencn_communication_interfaces::msg::Pin::CMPINBIT)
                {
                    RCLCPP_INFO(this->get_logger(), "Type CMPINBIT");
                    response->pins[i].cmpinbit.value = true;
                }
                else if(request->pins[i].pin_class == opencn_communication_interfaces::msg::Pin::CMPINFLOAT)
                {
                    RCLCPP_INFO(this->get_logger(), "Type CMPINFLOAT");
                    response->pins[i].cmpinfloat.value = 41.41;
                }
            }
            else if (request->pins[i].transaction_type == opencn_communication_interfaces::msg::Pin::SET)
            {
                RCLCPP_INFO(this->get_logger(), "Transaction type: SET");

                // Value
                if(request->pins[i].pin_class == opencn_communication_interfaces::msg::Pin::CMPINL32)
                {
                    RCLCPP_INFO(this->get_logger(), "Value: %d", request->pins[i].cmpinl32.value);
                }
                else if(request->pins[i].pin_class == opencn_communication_interfaces::msg::Pin::CMPINU32)
                {
                    RCLCPP_INFO(this->get_logger(), "Value: %d", request->pins[i].cmpinu32.value);
                }
                else if(request->pins[i].pin_class == opencn_communication_interfaces::msg::Pin::CMPINBIT)
                {
                    RCLCPP_INFO(this->get_logger(), "Value: %d", request->pins[i].cmpinbit.value);
                }
                else if(request->pins[i].pin_class == opencn_communication_interfaces::msg::Pin::CMPINFLOAT)
                {
                    RCLCPP_INFO(this->get_logger(), "Value: %f", request->pins[i].cmpinfloat.value);
                }
            }           
        }
        RCLCPP_INFO(this->get_logger(), "====Request end====");
        response->success = true;
    }
/*
    void joy_callback(const sensor_msgs::msg::Joy & msg) const
    {
        // Print buttons status
        RCLCPP_INFO(this->get_logger(), "Buttons status, begin");
        RCLCPP_INFO(this->get_logger(), "Button A: %d", msg.buttons[0]);
        RCLCPP_INFO(this->get_logger(), "Button B: %d", msg.buttons[1]);
        RCLCPP_INFO(this->get_logger(), "Button X: %d", msg.buttons[2]);
        RCLCPP_INFO(this->get_logger(), "Button Y: %d", msg.buttons[3]);
        RCLCPP_INFO(this->get_logger(), "Button LB: %d", msg.buttons[4]);
        RCLCPP_INFO(this->get_logger(), "Button RB: %d", msg.buttons[5]);
        RCLCPP_INFO(this->get_logger(), "Button status, end");
    }
    */

    //rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;

    rclcpp::Service<opencn_communication_interfaces::srv::Pins>::SharedPtr pins_service;


};




int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing opencn_pkg node");

    rclcpp::spin(std::make_shared<OpencnPkg>());
    rclcpp::shutdown();

    /*
    unsigned int port;

    if ((argc < 2) || (argc > 3))
    {
        show_usage(argv[0]);
        return 1;
    }

    std::string ip = argv[1];

    if (argc == 3)
        port = std::stoul(argv[2]);
    else
        port = DEFAULT_IP_PORT;

    capnp::EzRpcClient *client = new capnp::EzRpcClient(ip, port);

    //Components Initialisation
    MuxNTo1F mux("target", client, 4);
    SimplePG simple_pg(client, 3);
    Lcec_epos4 epos4("EPOS4", client);

    // Main menu
    // main_menu(&epos4, &mux, &simple_pg);
    transactions_test(client);
    */

    return 0;
}

/*Function definitions*/
static void sleep_ms(int time)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
}

static void show_usage(std::string name)
{
    std::cerr << "Usage: " << name << " IP [PORT]" << std::endl;
}

int transactions_test(capnp::EzRpcClient *client)
{
    Transactions transactions(client);

    // Nom, type, valeur(ecrire, lire)
    // Transaction

    CMPinBit set_ppm{"lcec.0.EPOS4.set-mode-ppm", client};
    CMPinBit set_inactive{"lcec.0.EPOS4.set-mode-inactive", client};
    CMPinFloat set_target{"mux.target.vector3.input0", client};
    CMPinU32 set_mux{"mux.target.selector", client};
    CMPinBit ppm_start{"lcec.0.EPOS4.ppm.start", client};

    CMPinFloat get_pos{"lcec.0.EPOS4.actual-position", client};
    CMPinBit get_idle{"lcec.0.EPOS4.idle", client};
    CMPinBit get_mode_ppm{"lcec.0.EPOS4.in-mode-ppm", client};
    CMParamFloat pos_scale{"lcec.0.EPOS4.pos-scale", client};
    CMParamFloat modulo{"lcec.0.EPOS4.modulo", client};

    Transactions setTransactions(client);
    Transactions getTransactions(client);

    std::cout << "Adding transactions" << std::endl;
    setTransactions.add(&set_ppm, TransactionType::SET);
    setTransactions.add(&set_target, TransactionType::SET);
    setTransactions.add(&ppm_start, TransactionType::SET);
    setTransactions.add(&modulo, TransactionType::SET);

    getTransactions.add(&get_mode_ppm, TransactionType::GET);
    getTransactions.add(&get_pos, TransactionType::GET);
    getTransactions.add(&get_idle, TransactionType::GET);
    getTransactions.add(&pos_scale, TransactionType::GET);
    getTransactions.add(&modulo, TransactionType::GET);

    set_mux.set(3);

    set_ppm = true;
    set_target = 50;
    setTransactions.execute();

    sleep_ms(1000);

    for (int i = 0; i < 10; i++)
    {
        getTransactions.execute();
        if (get_mode_ppm)
            break;

        sleep_ms(1000);
    }

    if (!get_mode_ppm)
    {
        std::cout << "Error: Could not set mode to PPM" << std::endl;
        return 1;
    }

    ppm_start = true;
    setTransactions.execute();

    sleep_ms(500);

    getTransactions.execute();

    while (!get_idle)
    {
        getTransactions.execute();
        std::cout << "get_mode_ppm: " << get_mode_ppm << std::endl;
        std::cout << "get_pos: " << get_pos << std::endl;
        std::cout << "get_idle: " << get_idle << std::endl;
        std::cout << "pos_scale: " << pos_scale << std::endl;
        std::cout << "modulo: " << modulo << std::endl;
        sleep_ms(200);
    }

    std::cout << "Set modulo : " << std::endl;
    modulo = 100;
    set_target = 210;
    setTransactions.execute();

    sleep_ms(1000);

    ppm_start = true;
    setTransactions.execute();

    sleep_ms(500);

    getTransactions.execute();

    while (!get_idle)
    {
        getTransactions.execute();
        std::cout << "get_mode_ppm: " << get_mode_ppm << std::endl;
        std::cout << "get_pos: " << get_pos << std::endl;
        std::cout << "get_idle: " << get_idle << std::endl;
        std::cout << "pos_scale: " << pos_scale << std::endl;
        std::cout << "modulo: " << modulo << std::endl;
        sleep_ms(200);
    }

    modulo.set(0);
    set_inactive.set(true);

    return 0;
}

int main_menu(Lcec_epos4 *epos4, MuxNTo1F *mux, SimplePG *simple_pg)
{
    while (1)
    {

        std::cout << "Main menu - Select mode" << std::endl;
        std::cout << "1 - Position mode (PPM)" << std::endl;
        std::cout << "2 - Velocity mode (PVM)" << std::endl;
        std::cout << "3 - Position mode (CSP)" << std::endl;
        std::cout << "4 - Velocity mode (CSV)" << std::endl;
        std::cout << "5 - Torque mode (CST)" << std::endl;
        std::cout << "6 - Homing mode (HM)" << std::endl;
        std::cout << "7 - Print mux" << std::endl;
        std::cout << "8 - Print simple_pg" << std::endl;
        std::cout << "9 - Print lcec" << std::endl;
        std::cout << "q - Quit" << std::endl;

        char mode;
        std::cin >> mode;
        std::cout << std::endl;
        if (mode == 'q')
            return 0;
        else if (mode == '1')
        {
            if (ppm_menu(epos4, mux, simple_pg) == 0)
                epos4->setMode(MODE_INACTIVE);
            else
                std::cout << "Error in position mode" << std::endl;
        }
        else if (mode == '2')
        {
            std::cout << "Velocity mode (PVM)" << std::endl;
            std::cout << "Not implemented yet" << std::endl; // TODO
        }
        else if (mode == '3')
        {
            std::cout << "Position mode (CSP)" << std::endl;
            std::cout << "Not implemented yet" << std::endl; // TODO
        }
        else if (mode == '4')
        {
            std::cout << "Velocity mode (CSV)" << std::endl;
            std::cout << "Not implemented yet" << std::endl; // TODO
        }
        else if (mode == '5')
        {
            std::cout << "Torque mode (CST)" << std::endl;
            std::cout << "Not implemented yet" << std::endl; // TODO
        }
        else if (mode == '6')
        {
            std::cout << "Homing mode (HM)" << std::endl;
            std::cout << "Not implemented yet" << std::endl; // TODO
        }
        else if (mode == '7')
        {
            std::cout << "Print mux" << std::endl;
            mux->print();
        }
        else if (mode == '8')
        {
            std::cout << "Print simple_pg" << std::endl;
            simple_pg->print();
        }
        else if (mode == '9')
        {
            std::cout << "Print lcec" << std::endl;
            epos4->print();
        }
        else
        {
            std::cout << "Invalid mode" << std::endl;
        }
        std::cout << std::endl;
    }
}

int ppm_menu(Lcec_epos4 *epos4, MuxNTo1F *mux, SimplePG *simple_pg)
{
    hal_data_u value;
    float value_f;
    bool is_idle = true;
    mux->setSelector(3);
    epos4->setMode(MODE_PPM);

    while (1)
    {
        std::cout << "Position mode (PPM) - Select action" << std::endl;
        std::cout << "1 - Set position" << std::endl;
        std::cout << "2 - Start" << std::endl;
        std::cout << "3 - Stop" << std::endl;
        std::cout << "4 - Quit stop" << std::endl;
        std::cout << "5 - Get mux pins" << std::endl;
        std::cout << "6 - Get lcec pins" << std::endl;
        std::cout << "7 - Get lcec parameters" << std::endl;
        std::cout << "q - Quit" << std::endl;

        char action;
        std::cin >> action;
        if (action == 'q')
        {
            is_idle = epos4->isIdle();
            if (is_idle)
                return 0;
            else
                std::cout << "Wait until " << epos4 << "is idle." << std::endl;
        }
        else if (action == '1')
        {
            double position;
            std::cout << " 1: Position value (turns): ";
            std::cin >> position;
            mux->setInput(3, position);
        }
        else if (action == '2')
        {
            value.b = 1;
            std::cout << "Start" << std::endl;
            is_idle = epos4->isIdle();
            if (is_idle)
                epos4->setPin("lcec.0.EPOS4.ppm.start", value);
            else
                std::cout << "EPOS4 is not idle" << std::endl;
        }
        else if (action == '3')
        {
            value.b = 1;
            std::cout << "Stop" << std::endl;
            epos4->setPin("lcec.0.EPOS4.ppm.stop", value);
        }
        else if (action == '4')
        {
            std::cout << "Quit stop" << std::endl;
            epos4->quickStop();
        }
        else if (action == '5')
        {
            std::cout << "Get mux pins" << std::endl;
            mux->print();
        }
        else if (action == '6')
        {
            std::cout << "Get lcec pins" << std::endl;
            epos4->print();
        }
        else if (action == '7')
        {
            std::cout << "Get lcec parameters" << std::endl;
            epos4->printParameters();
        }
        else
        {
            std::cout << "Invalid action" << std::endl;
        }
    }
}
