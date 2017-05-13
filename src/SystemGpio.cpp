#include "robot_wheel_speeds/SystemGpio.h"

#include <iostream>
#include <fstream>
#include <unistd.h>


SystemGPIO::SystemGPIO(const std::vector<int>& pins)
    : m_pins(pins)
{
    for (const int pin : m_pins)
    {
        if (!SetupPin(pin))
        {
            std::cerr << "Couldn't set up pin " << pin << std::endl;
        }
    }
    wiringPiSetupSys();
}

SystemGPIO::~SystemGPIO()
{
    for (const int pin : m_pins)
    {
        if(!TearDownPin(pin))
        {
            std::cerr << "Couldn't tear down pin " << pin << std::endl;
        }
    }
}

bool SystemGPIO::SetupPin(int pin)
{
    std::ofstream exp("/sys/class/gpio/export");
    if (exp.good())
    {
        exp << std::to_string(pin).c_str();
        exp.close();

        std::string poll = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
        while (access(poll.c_str(), W_OK) != 0)
        {
            usleep(1);
        }
        return true;
    }
    return false;
}

bool SystemGPIO::TearDownPin(int pin)
{
    std::ofstream unexp("/sys/class/gpio/unexport");
    if (unexp.good())
    {
        unexp << std::to_string(pin).c_str();
        return true;
    }
    return false;
}
