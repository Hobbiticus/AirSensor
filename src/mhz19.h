#pragma once
#include <SoftwareSerial.h>


class MHZ19
{
  public:
    MHZ19(int rx, int tx, int pwm);
    int GetCO2();
    bool IsPreheated();

  private:
    SoftwareSerial m_Serial;
    bool m_IsHeated;
    unsigned long m_StartTime;

    static unsigned char ComputeChecksum(unsigned char* data)
    {
      unsigned char checksum = 0;
      for (int i = 1; i < 8; i++)
        checksum += data[i];
      checksum = 0xff - checksum;
      checksum++;

      return checksum;
    }
    static void InterruptHigh(void* arg);
    static void InterruptLow(void* arg);
    int m_PWMPin;
    volatile int m_LastPWMReading;
    volatile unsigned long m_LastHighTime;
};
