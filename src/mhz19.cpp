#include "mhz19.h"


MHZ19::MHZ19(int rx, int tx, int pwm)
: m_PWMPin(pwm), m_LastPWMReading(-1)
{
    if (rx >= 0 && tx >= 0)
      m_Serial.begin(9600, SWSERIAL_8N1, 14, 21, false);
    else
      attachInterruptArg(digitalPinToInterrupt(m_PWMPin), InterruptHigh, this, RISING);
}

bool MHZ19::IsPreheated()
{
    return true;
  if (m_IsHeated)
    return true;
  m_IsHeated = millis() - m_StartTime > 1000 * 180;
  return m_IsHeated;
}

void MHZ19::InterruptHigh(void* arg)
{
  MHZ19* mhz = (MHZ19*)arg;
  mhz->m_LastHighTime = micros();
  detachInterrupt(digitalPinToInterrupt(mhz->m_PWMPin));
  attachInterruptArg(digitalPinToInterrupt(mhz->m_PWMPin), MHZ19::InterruptLow, arg, FALLING);
}

void MHZ19::InterruptLow(void* arg)
{
  MHZ19* mhz = (MHZ19*)arg;
  unsigned long now = micros();
  detachInterrupt(digitalPinToInterrupt(mhz->m_PWMPin));
  attachInterruptArg(digitalPinToInterrupt(mhz->m_PWMPin), InterruptHigh, arg, RISING);
  unsigned long highTimeMS = (now - mhz->m_LastHighTime) / 1000;
  mhz->m_LastPWMReading = (highTimeMS - 2) * 5;
}

int MHZ19::GetCO2()
{
  if (!IsPreheated())
    return -1;

  if (m_PWMPin >= 0)
    return m_LastPWMReading;
          
  unsigned char request[9];
  memset(request, 0, sizeof(request));
  request[0] = 0xFF;
  request[1] = 0x01;
  request[2] = 0x86;
  request[8] = ComputeChecksum(request);

  m_Serial.write(request, sizeof(request));

  unsigned char response[9];
  int startTime = millis();
  while (m_Serial.available() != sizeof(response))
  {
    if (millis() - startTime > 100)
      return -2;
    delay(10);
  }
  m_Serial.readBytes(response, sizeof(response));
  unsigned char responseChecksum = ComputeChecksum(response);
  if (responseChecksum != response[8])
  {
    Serial.println("Response checksum failed");
    return -3;
  }

  int ret = response[2];
  ret <<= 8;
  ret += response[3];
  return ret;
}