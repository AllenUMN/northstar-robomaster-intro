#ifndef STANDARD_DRIVERS_HPP_
#define STANDARD_DRIVERS_HPP_

#include "tap/drivers.hpp"

#include "robot/control_operator_interface.hpp"

namespace src::standard
{
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers() : tap::Drivers(), controlOperatorInterface(this) {}

public:
    control::ControlOperatorInterface controlOperatorInterface;
};  // class src::standard::Drivers
}  // namespace src::standard

#endif  // STANDARD_DRIVERS_HPP_
