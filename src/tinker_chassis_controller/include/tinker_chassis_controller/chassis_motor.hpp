#ifndef __TINKER_CHASSIS_MOTOR__H_
#define __TINKER_CHASSIS_MOTOR__H_

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>


namespace tinker_chassis_controller
{
    class ChassisMotor
    {
        public:
            ChassisMotor(
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state,
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state,
                std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command
                );

            void set_velocity(double value);
            double get_velocity(void);

        private:
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state_;
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state_;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command_;

    };
}


#endif // __TINKER_CHASSIS_MOTOR__H_