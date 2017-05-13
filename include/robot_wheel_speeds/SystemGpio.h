#pragma once
#include <wiringPi.h>
#include <vector>

class SystemGPIO
{
public:
    SystemGPIO(const std::vector<int>& pins);

    ~SystemGPIO();

private:
    bool SetupPin(int pin);

    bool TearDownPin(int pin);


    std::vector<int> m_pins;
};


