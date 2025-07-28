#include "walle_firmware/walle_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace walle_firmware
{

WalleInterface::WalleInterface()
{
}


WalleInterface::~WalleInterface()
{
    if(arduino_.IsOpen())
    {
        try
        {
            arduino_.Close();
        }
        catch(...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("WalleInterface"), "Error while closing the serial port"<< port_);
        }
    }
}


CallbackReturn WalleInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);

    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    try 
    {
        port_ = info_.hardware_parameters.at("port");
    }

    catch (const std::out_of_range &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("WalleInterface"), "No Serial Port provided! Aborting!!! ");
        return CallbackReturn::FAILURE;
    }

    velocity_commands_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
    velocity_states_.reserve(info_.joints.size());
    last_run_ = rclcpp::Clock().now();


    return CallbackReturn::SUCCESS;
}



std::vector<hardware_interface::StateInterface> WalleInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Provide only a position Interafce


    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
    }

    return state_interfaces;

}


std::vector<hardware_interface::CommandInterface> WalleInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Provide only a position Interafce

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
    }

    return command_interfaces;
}

CallbackReturn WalleInterface::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("WalleInterface"), "Starting the Robot hardware");

    // Reset commands and states
    
    velocity_states_ = { 0.0,0.0 };
    position_states_ = { 0.0,0.0 };
    velocity_commands_ = { 0.0, 0.0};


    try
    {
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_1152000);    
    }

    catch(...)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("WalleInterface"), "Something went wrong while interacting with port "<< port_);
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("WalleInterface"), "Hardware started , ready to take commands");
    return CallbackReturn::SUCCESS;

}


CallbackReturn WalleInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("WalleInterface"), "Hardware Stopping!!! ");

    if (arduino_.IsOpen())
    {
        try
        {
            arduino_.Close();
        }

        catch(...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("WalleInterface"), "Something went wrong while closing with port" << port_);
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("WalleInterface"), "Hardware Stopped");
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type WalleInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (arduino_.IsDataAvailable())
    {
        auto dt = (rclcpp::Clock().now() - last_run_).seconds();
        std::string message;
        arduino_.ReadLine(message);
        std::stringstream ss(message);
        std::string res;
        int multiplier = 1;


        while (std::getline(ss, res, ','))
        {
            multiplier = res.at(1) == 'p' ? 1 : -1;

            if (res.at(0) == 'r')
            {
                velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
                position_states_.at(0) += velocity_states_.at(0) * dt;
            }

            else if (res.at(0) == 'l')
            {
                velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
                position_states_.at(1) += velocity_states_.at(1) * dt;
            }
        }

        last_run_ = rclcpp::Clock().now();
    }

    return hardware_interface::return_type::OK;
}


hardware_interface::return_type WalleInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    std::stringstream message_stream;

    char right_wheel_sign = velocity_commands_.at(0) >=0 ? 'p' : 'n';
    char left_wheel_sign = velocity_commands_.at(1) >0 ? 'p' : 'n';

    std::string compensate_zeros_right = "";
    std::string compensate_zeros_left = "";

    if (std::abs(velocity_commands_.at(0)) < 10.0)
    {
        compensate_zeros_right = "0";
    }
    else
    {
        compensate_zeros_right = "";
    }

    if (std::abs(velocity_commands_.at(1)) < 10.0)
    {
        compensate_zeros_left = "0";
    }
    else 
    {
        compensate_zeros_left = "";
    }

    message_stream << std::fixed << std::setprecision(2) 
    << "r" << right_wheel_sign << compensate_zeros_right <<std::abs(velocity_commands_.at(0))
    << ",l" << left_wheel_sign << compensate_zeros_left <<std::abs(velocity_commands_.at(1)) << ",";
    

    try 
    {
        arduino_.Write(message_stream.str());
    }

    catch(...)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("WalleInterface"), "something went wrong while sending message "
                                                << message_stream.str() << " to the port "<< port_);
        
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;

}

} // namespace walle_firmware

PLUGINLIB_EXPORT_CLASS(walle_firmware::WalleInterface, hardware_interface::SystemInterface)
