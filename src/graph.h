#pragma once

#include <SoftwareSerial.h>

struct ChannelParams
{
    ChannelParams()
    : m_Start(0), m_Step(0), m_Scale(0)
    {
        m_AxisLabel[0] = '\0';
    }
  int m_Start;
  double m_Step;
  double m_Scale;
  char m_AxisLabel[16];
};

#define NUM_GRAPH_CHANNELS 2
struct GraphParams
{
  ChannelParams m_Channels[NUM_GRAPH_CHANNELS];
};

class Graph
{
public:
    Graph(SoftwareSerial& displaySerial);
    void SetAxisLabels(GraphParams& params);
    void SendValuesToWaveform(int channel, int numValues, uint8_t* values);
    
private:
    SoftwareSerial& m_DisplaySerial;
};
