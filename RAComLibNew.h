//RACom Branch FREERTOS
#ifndef RAComLibNew_H	//tests if RACom_H has not been defined
#define RAComLibNew_H	//define RACom_H

#include "Arduino.h"	//includes the library Arduino.h
#include "SoftwareSerial.h"	 //Includes the library SoftwareSerial.h

/* Kernel includes. */
#include "Arduino_FreeRTOS.h"
#include "timers.h"     
#include "semphr.h"
    
//an enum with constants
enum
{
	//rate of trnsmission
    BAUD_RATE = 9600,
	//pin used
    RX = 8,
    TX = 6,
    SET_PIN = 13,
    JOIN_TIMEOUT = 5000 / portTICK_PERIOD_MS, // 5 seconds
    REPLY_TIMEOUT = 3000 / portTICK_PERIOD_MS,
    NUM_NEXT_POS = 8,
    SPECIAL_ANT_ID = 3,
    BUFFER_DIM = 120,
	MAX_ANTS = 16, // Max number of ants in the antNet
	TASK_DELAY = 200 / portTICK_PERIOD_MS  //delay applied to a task
};
//Define RACom class with relatives methods
class RACom {
public:
    void init(byte id);
    void comunicationMode();
    void commandMode();
    void comAlgo();
    void readBuffer();
    void setupTimers();
    void setupMutex();
    void setNextPosArray(byte replace[]);
    void setMyCurrentPosition(byte pos);
    byte getCurrentPosOfAnt(byte num_ant);
    byte* getRecvPosArray(byte num_ant);
    void setTaskHandle(TaskHandle_t* xHandleRGB, TaskHandle_t* xHandleMotion);
    void setBufferTaskHandle(TaskHandle_t* xHandleBuffer, TaskHandle_t* xHandleComm);
    void setStartAndStop(byte state); // 0 = stop, 1 = start
    byte getStartAndStop();
    void setAntMode(byte mode); // 0 = default ant mode
    byte getAntMode();


private:
    // methods for comAlgo
    void Join();
    void Send();
    void Receive();
    void Hello();
    void ReceiveHello();
    void BuildTable();
    void UpdatePrevSucc();
    void startTimer();
    int getFirstAnt();
    static void comTimerCallback(TimerHandle_t xTimer);
	void Special_Turn();
    void Death(int mit);
    int count_underscores(char *s);
};

#endif