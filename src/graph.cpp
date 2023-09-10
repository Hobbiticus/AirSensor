#include "graph.h"
#include "nextion.h"

//these must match the gdw/gdh of the waveform control
#define GRID_WIDTH 50
#define GRID_HEIGHT 26
//these must match the w/h of the waveform control
#define WAVEFORM_WIDTH 256
#define WAVEFORM_HEIGHT 188
const static int YAxisLines = (WAVEFORM_HEIGHT / GRID_HEIGHT) + 1;

Graph::Graph(SoftwareSerial& displaySerial)
: m_DisplaySerial(displaySerial)
{

}

void Graph::SetAxisLabels(GraphParams& params)
{
    for (int i = 0; i < YAxisLines; i++)
    {
        m_DisplaySerial.printf("y%d.val=%d", i, (int)(params.m_Channels[0].m_Start + i * params.m_Channels[0].m_Step));
        m_DisplaySerial.write(EOS, 3);
    }
    for (int c = 1; c < NUM_GRAPH_CHANNELS; c++)
    {
        if (params.m_Channels[c].m_Step == 0)
            continue;
        if (params.m_Channels[c].m_Start != params.m_Channels[0].m_Start ||
            params.m_Channels[c].m_Step != params.m_Channels[0].m_Step)
        {
            for (int i = 0; i < YAxisLines; i++)
            {
                m_DisplaySerial.printf("y%d%d.val=%d", i, c + 1, (int)(params.m_Channels[c].m_Start + i * params.m_Channels[c].m_Step));
                m_DisplaySerial.write(EOS, 3);
            }
            //hack for humidity
            if (params.m_Channels[c].m_Start == 0 && params.m_Channels[c].m_Step == 20)
            {
                m_DisplaySerial.printf("y62.pco=0");
                m_DisplaySerial.write(EOS, 3);
                m_DisplaySerial.printf("y72.pco=0");
                m_DisplaySerial.write(EOS, 3);
            }
        }
    }
    for (int c = 0; c < NUM_GRAPH_CHANNELS; c++)
    {
        if (params.m_Channels[c].m_AxisLabel[0] != '\0')
        {
            m_DisplaySerial.printf("cScope%d.txt=\"%s\"", c + 1, params.m_Channels[c].m_AxisLabel);
            m_DisplaySerial.write(EOS, 3);
            m_DisplaySerial.printf("vis cScope%d,1", c + 1);
            m_DisplaySerial.write(EOS, 3);
        }
        else
        {
            m_DisplaySerial.printf("vis cScope%d,0", c + 1);
            m_DisplaySerial.write(EOS, 3);
        }
    }
    //m_DisplaySerial.printf("vis cScope3,0");
    //m_DisplaySerial.write(EOS, 3);
}

static uint8_t ValueToDisplayValue(int value, int start, double step, double valueScale)
{
  double displayValue = value;
  displayValue *= valueScale;
  displayValue -= start;
  displayValue *= GRID_HEIGHT;
  displayValue /= step;
  //clamp to valid values of uint8_t
  if (displayValue < 0)
    displayValue = 0;
  if (displayValue > 255)
    displayValue = 255;
  return (uint8_t)displayValue;
}

void Graph::SendValuesToWaveform(int channel, int numValues, uint8_t* values)
{
  //ctrl ID, channel, array length
  // Serial.printf("addt 1,0,%d", valuesUsed);
  // for (int i = 0; i < valuesUsed; i++)
  //   Serial.printf(" %hhu", displayValues[i]);
  // Serial.println("");
  m_DisplaySerial.printf("addt 1,%d,%d", channel, numValues); // we are sending an array of this size to the waveform
  m_DisplaySerial.write(EOS, 3);
  //wait for GO signal
  Serial.println("Waiting for GO");
  uint8_t reply[4];
  int waitStartTime = millis();
  while (m_DisplaySerial.available() < 4)
  {
    delay(1);
    //have had this stall here before
    if (millis() - waitStartTime > 200)
    {
      Serial.println("GO never received");
      return;
    }
  }
  m_DisplaySerial.readBytes(reply, 4);
  int waitEndTime = millis();
  Serial.printf("Waited %d ms for GO signal\n", waitEndTime - waitStartTime);
  if (memcmp(reply, GO, 4) != 0)
  {
    Serial.printf("GO signal not received, got %02hhx %02hhx %02hhx %02hhx", reply[0], reply[1], reply[2], reply[3]);
    return;
  }

  //now send the values
  m_DisplaySerial.write(values, numValues);
  //wait for OK signal
  Serial.println("Waiting for OK");
  waitStartTime = millis();
  while (m_DisplaySerial.available() < 4)
    delay(1);
  m_DisplaySerial.readBytes(reply, 4);
  waitEndTime = millis();
  Serial.printf("Waited %d ms for AOK signal\n", waitEndTime - waitStartTime);
  if (memcmp(reply, AOK, 4) != 0)
  {
    Serial.printf("OK signal not received, got %02hhx %02hhx %02hhx %02hhx", reply[0], reply[1], reply[2], reply[3]);
    return;
  }
}
