#include "led.h"

void Led::Update()
{
  uint32_t now = SYS_TIME;

  if(now < nUntil) {
    palSetLine(m_line);
  }
  else {
    palClearLine(m_line);
  }
}

void Led::Blink(uint16_t nLength)
{
    nUntil = SYS_TIME + nLength;
}