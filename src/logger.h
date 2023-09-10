#include <Arduino.h>
#include <SD.h>

typedef int VALUE;

#define INVALID_VALUE 0x80808080
#define INVALID_VALUE_CHAR 0x80

class Logger
{
public:
    Logger(const char* filename, time_t periodSecs, int numPeriods);
    ~Logger();

    static bool begin(int cs);
    static void end();
    
    void AddData(time_t when, VALUE data);
    void GetData(VALUE* buffer, int* numEntries, time_t* startTime);
    
private:
    char m_Filename[64];
    
    time_t m_StartTime;
    int m_FirstPeriodIndex;
    time_t m_PeriodSecs;
    int m_NumPeriods;

    //SemaphoreHandle_t m_Lock;

    VALUE* m_Data;
};