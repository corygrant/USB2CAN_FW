#pragma once

#define LED_BLINK_SPLIT 200
#define LED_BLINK_PAUSE 1500

#include "port.h"

class Led
{
public:
    Led(ioline_t type)
    {
        m_line = type;
    };

    void Update();
    void Blink(uint16_t nLength);

    private:
        ioline_t m_line;

        uint32_t nUntil = 0;
};