#include "tinker_chassis_controller/chassis_motor.hpp"

using namespace tinker_chassis_controller;

ChassisMotor::ChassisMotor(
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command
    )
    : position_state_(position_state)
    , velocity_state_(velocity_state)
    , velocity_command_(velocity_command)
{

}

void ChassisMotor::set_velocity(double value)
{
    velocity_command_.get().set_value(value);
}

double ChassisMotor::get_velocity(void)
{
    velocity_command_.get().get_value();
}