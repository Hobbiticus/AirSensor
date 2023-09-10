#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "logger.h"
#include <Arduino.h>

#define DO_LOGGING


void Lock(SemaphoreHandle_t lock)
{
    while (!xSemaphoreTake(lock, 256))
    {}
}

void Unlock(SemaphoreHandle_t lock)
{
    xSemaphoreGive(lock);
}

bool Logger::begin(int cs)
{
#ifdef DO_LOGGING
    return SD.begin(cs);
#else
    return true;
#endif
}

void Logger::end()
{
#ifdef DO_LOGGING
    SD.end();
#endif
}


Logger::Logger(const char* filename, time_t periodSecs, int numPeriods)
{
    //m_Lock = xSemaphoreCreateMutex();
    strncpy(m_Filename, filename, sizeof(m_Filename));
#ifdef DO_LOGGING

    File file = SD.open(m_Filename, O_READ);
    if (file)
    {
        file.read(&m_StartTime, sizeof(m_StartTime));
        file.read(&m_FirstPeriodIndex, sizeof(m_FirstPeriodIndex));
        file.read(&m_PeriodSecs, sizeof(m_PeriodSecs));
        file.read(&m_NumPeriods, sizeof(m_NumPeriods));
        
        //Serial.printf("first idx: %d, num periods: %d\n", m_FirstPeriodIndex, m_NumPeriods);

        m_Data = new VALUE[m_NumPeriods];
        
        file.read(m_Data, sizeof(VALUE) * m_NumPeriods);
        // for (int i = 0; i < m_NumPeriods; i++)
        // {
        //     Serial.printf(" %d", m_Data[i]);
        // }
        // Serial.printf("\n");

        file.close();
    }
    else
    {
#endif
        m_PeriodSecs = periodSecs;
        m_NumPeriods = numPeriods;
        m_StartTime = 0;
        m_FirstPeriodIndex = 0;

        m_Data = new VALUE[m_NumPeriods];
        memset(m_Data, INVALID_VALUE_CHAR, sizeof(VALUE) * m_NumPeriods);
#ifdef DO_LOGGING
        
        file = SD.open(m_Filename, O_WRITE | O_CREAT);
        if (!file)
        {
            Serial.printf("Hey, can't open file '%s' for writing!\n", m_Filename);
            return;
        }
        
        file.write((const uint8_t*)&m_StartTime, sizeof(m_StartTime));
        file.write((const uint8_t*)&m_FirstPeriodIndex, sizeof(m_FirstPeriodIndex));
        file.write((const uint8_t*)&m_PeriodSecs, sizeof(m_PeriodSecs));
        file.write((const uint8_t*)&m_NumPeriods, sizeof(m_NumPeriods));
        file.write((const uint8_t*)m_Data, sizeof(VALUE) * m_NumPeriods);
        file.close();
    }
#endif
}

Logger::~Logger()
{
    free(m_Filename);
}


void Logger::AddData(time_t when, VALUE data)
{
    //Lock(m_Lock);
    //write the new data
    time_t timeslice = when / m_PeriodSecs;
    int bin = timeslice % m_NumPeriods;
    //Serial.printf("Adding %d to bin %ld/%d @ %ld\n", data, timeslice, bin, when);
    
#ifdef DO_LOGGING
    const static int headerLength = sizeof(m_StartTime) + sizeof(m_FirstPeriodIndex) +
                                    sizeof(m_PeriodSecs) + sizeof(m_NumPeriods);
                                    
    File file = SD.open(m_Filename, O_WRITE);
    if (!file)
    {
        Serial.printf("EEK! cannot open file %s to add data\n", m_Filename);
        //xSemaphoreGive(m_Lock);
        return;
    }
    file.seek(headerLength + sizeof(VALUE) * bin);
#endif
    m_Data[bin] = data;
#ifdef DO_LOGGING
    file.write((const uint8_t*)&data, sizeof(VALUE));
#endif
    
    time_t firstTimeslice = m_StartTime / m_PeriodSecs;
    if (timeslice - firstTimeslice > m_NumPeriods)
    {
        //need to move the first period index up
        int slicesToMove = (int)(timeslice - firstTimeslice - m_NumPeriods);
        //Serial.printf("Moving first timeslice: %ld - %ld (%ld) >= %d, moving up %d spaces\n", timeslice, firstTimeslice, timeslice - firstTimeslice, m_NumPeriods, slicesToMove);
        
        if (slicesToMove >= m_NumPeriods)
        {
            //clear the rest of the data
            int firstChunkSize = m_NumPeriods - bin - 1;
            int otherChunkSize = m_NumPeriods - firstChunkSize - 1;
            //Serial.printf("Clearing all the things - %d thru %d and 0 thru %d\n", bin + 1, bin + 1 + firstChunkSize, otherChunkSize);
            memset(m_Data + bin + 1, INVALID_VALUE_CHAR, sizeof(VALUE) * firstChunkSize);
#ifdef DO_LOGGING
            file.write((const uint8_t*)m_Data + bin + 1, sizeof(VALUE) * firstChunkSize);
#endif
            memset(m_Data, INVALID_VALUE_CHAR, sizeof(VALUE) * otherChunkSize);
#ifdef DO_LOGGING
            file.seek(headerLength);
            file.write((const uint8_t*)m_Data, sizeof(VALUE) * otherChunkSize);
#endif
        }
        else if (slicesToMove == 1)
        {
            //easy peasy
            //Serial.printf("Easy peasy\n");
        }
        else
        {
            //need to do a partial clear
            //what do we need to clear?!?
            //everything from the last thing we put in here to now
            //Serial.printf("not so easy peasy\n");
            int binsToClear = slicesToMove - 1;
            int clearStartBin = bin - binsToClear;
            if (clearStartBin < 0)
                clearStartBin += m_NumPeriods;
            if (clearStartBin + binsToClear >= m_NumPeriods)
            {
                //two chunks!
                int clearFirstChunkSize = m_NumPeriods - clearStartBin;
                int clearSecondChunkSize = binsToClear - clearFirstChunkSize;
                memset(m_Data + clearStartBin, INVALID_VALUE_CHAR, sizeof(VALUE) * clearFirstChunkSize);
#ifdef DO_LOGGING
                file.seek(headerLength + sizeof(VALUE) * clearStartBin);
                file.write((const uint8_t*)m_Data + clearStartBin, sizeof(VALUE) * clearFirstChunkSize);
#endif
                memset(m_Data, INVALID_VALUE_CHAR, sizeof(VALUE) * clearSecondChunkSize);
#ifdef DO_LOGGING
                file.seek(headerLength);
                file.write((const uint8_t*)m_Data, sizeof(VALUE) * clearSecondChunkSize);
#endif
            }
            else
            {
                memset(m_Data + clearStartBin, INVALID_VALUE_CHAR, sizeof(VALUE) * binsToClear);
#ifdef DO_LOGGING
                file.seek(headerLength + sizeof(VALUE) * clearStartBin);
                file.write((const uint8_t*)m_Data + clearStartBin, sizeof(VALUE) * binsToClear);
#endif
            }
        }
        
        //update start of timeline
        m_StartTime = (timeslice - m_NumPeriods) * m_PeriodSecs;
        m_FirstPeriodIndex = bin + 1;
        if (m_FirstPeriodIndex >= m_NumPeriods)
            m_FirstPeriodIndex = 0;
#ifdef DO_LOGGING
        file.seek(0);
        file.write((const uint8_t*)&m_StartTime, sizeof(m_StartTime));
        file.write((const uint8_t*)&m_FirstPeriodIndex, sizeof(m_FirstPeriodIndex));
#endif
        //Serial.printf("Start time = %lu, index = %d\n", m_StartTime, m_FirstPeriodIndex);
    }
    
#ifdef DO_LOGGING
    file.close();
#endif
    //Unlock(m_Lock);
}

void Logger::GetData(int* buffer, int* numEntries, time_t* startTime)
{
    //Lock(m_Lock);

    *numEntries = m_NumPeriods;
    *startTime = m_StartTime;
    
    int startIndex = m_FirstPeriodIndex;
    //skip over any empty space at the beginning
    // Serial.printf("first entry @ %d = %d\n", startIndex, m_Data[startIndex]);
    // for (int i = 0; i < m_NumPeriods; i++)
    // {
    //     if (m_Data[i] == INVALID_VALUE)
    //         Serial.printf(" XX");
    //     else
    //         Serial.printf(" %d", m_Data[i]);
    // }
    // Serial.printf("\n");
    while (m_Data[startIndex] == INVALID_VALUE)
    {
        startIndex = (startIndex + 1) % m_NumPeriods;
        (*numEntries)--;
        *startTime += m_PeriodSecs;
        //Serial.printf("Skip! next is %d\n", m_Data[startIndex]);
    }
    //Serial.printf("Actual num entries = %d, start index = %d\n", *numEntries, startIndex);
    
    if (startIndex + *numEntries > m_NumPeriods)
    {
        //get the data in 2 chunks
        int firstChunkSize = m_NumPeriods - startIndex;
        //Serial.printf("starting at %d, first chunk = %d, rest = %d\n", startIndex, firstChunkSize, *numEntries - firstChunkSize);
        memcpy(buffer, m_Data + startIndex, sizeof(VALUE) * firstChunkSize);
        memcpy(buffer + firstChunkSize, m_Data, sizeof(VALUE) * (*numEntries - firstChunkSize));
    }
    else
    {
        //do it in one chunk
        //Serial.printf("One chunk, %d -> %d\n", startIndex, *numEntries);
        memcpy(buffer, m_Data + startIndex, sizeof(VALUE) * *numEntries);
    }
    //Unlock(m_Lock);
}
