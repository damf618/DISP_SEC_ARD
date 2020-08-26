// Include CoopTask since we want to manage multiple tasks.
#include <CoopTask.h>
#include <CoopSemaphore.h>
#include <CoopMutex.h>
#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>


#define USE_BUILTIN_TASK_SCHEDULER

#define DEBUG

#define nodeID 2
#define COMM_TRIES 3
#define ALARM_GPIO 5
#define ANALOG_LIM 20
#define RESET_GPIO 4
#define BAUD_RATE 57600
#define alarm_code 987
#define fail_code 654
#define normal_code 321
#define alarm_fail_code 258
#define CHANNEL_NO1 117 
#define CHANNEL_NO2 120
#define CHANNEL_NO3 122
#define N_STATES 4
#define PERIOD_TIME 500
#define UPDATE_TIME 300
#define COMM_WAITING_TIME 0 
#define COMM_ERROR_TOLERANCE 150
#define RF_START_UP_TIME 2000
#define MAX_ANALOG_VAL 1023
#define MIN_ANALOG_VAL 0
#define INTEGER_VOLT_VAL 33
#define N_TRIES_STATE 7

CoopMutex serialMutex;
CoopMutex spiMutex;
CoopSemaphore taskSema(1, 1);

uint8_t channels1[] = {CHANNEL_NO1,CHANNEL_NO2,CHANNEL_NO3};
uint8_t channelCounter = 0;
uint8_t Max_Fails_counter = 0;
uint8_t Alarm_State=0;
uint8_t Fail_State=0;

/**** Configure the nrf24l01 CE and CS pins ****/
RF24 radio(16,0);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

const int analogInPin = A0;
uint32_t displayTimer = 0;
uint32_t ReadTimer = 0;
bool hoopingchannel=true;
int counter=0;

int Code[N_STATES]={alarm_code,fail_code,normal_code,alarm_fail_code};
String TCode[N_STATES]={"Alarma","Falla","Normal","Alarma_Falla"};

CoopTask<void>* task1;
CoopTask<void>* task2;
CoopTask<void, CoopTaskStackAllocatorFromLoop<>>* taskReport;


/* --- Rfresh the contact State ---*/
void Contact_Validation()
{
  Alarm_State = digitalRead(ALARM_GPIO);
  Fail_State = analogRead(analogInPin);
  Fail_State = map(Fail_State, MIN_ANALOG_VAL, MAX_ANALOG_VAL, MIN_ANALOG_VAL, INTEGER_VOLT_VAL);
    
  if((Alarm_State== HIGH)||(Fail_State>=ANALOG_LIM))
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
#if defined(DEBUG)
    {
    CoopMutexLock serialLock(serialMutex);          
    Serial.println(F("EVENT FROM ALARM or FAIL CONTACT!! "));
    }
#endif    
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
#if defined(DEBUG)
    {
    CoopMutexLock serialLock(serialMutex);          
    Serial.println(F("NORMAL_ALARM_BUTTON"));
    }
#endif
  }
}

/* --- TASK 1 Control ---*/
void Control() noexcept
{
  bool val;
  for (;;) // explicitly run forever without returning
  {
    taskSema.wait();
    // Send an 'M' type message containing the current millis()
#if defined(DEBUG)
    {
    CoopMutexLock serialLock(serialMutex);          
    Serial.println(F("New Cycle "));
    }
#endif      
    for (int i=0;i<N_STATES;i++)
    {
      for (int j=0;j<N_TRIES_STATE;j++)
      {
#if defined(DEBUG)
        {
        CoopMutexLock serialLock(serialMutex);                
        Serial.print(F("Codigo --> "));
        Serial.println(TCode[i]);
        Serial.print(F(" Status: "));
        }
#endif  
        {
        CoopMutexLock Lock(spiMutex);
        val= mesh.write(&Code[i], 'M', sizeof(int));   
        }
        
        if (!val)
        {
          {
          CoopMutexLock Lock(spiMutex);
          val= mesh.checkConnection();   
          }
          // If a write fails, check connectivity to the mesh network
          if ( !val )
          {
            counter++;
            if(counter>=COMM_TRIES)
            {
              counter=0;
              // refresh the network address
              {
              CoopMutexLock serialLock(serialMutex);
              Serial.println(F("Renewing Address"));
              }
              {
              CoopMutexLock Lock(spiMutex);
              val=mesh.renewAddress(COMM_WAITING_TIME);
              }
              if(!val)
              {
                // If address renewal fails, reconfigure the radio and restart the mesh
                // This allows recovery from most if not all radio errors
                if(hoopingchannel==true)
                {
#if defined(DEBUG)
                {
                  CoopMutexLock serialLock(serialMutex);                          
                  Serial.print(F("Channel: "));
                  Serial.println(channels1[channelCounter]);
                }
#endif
                  {
                  CoopMutexLock Lock(spiMutex);
                  mesh.begin(channels1[channelCounter],RF24_1MBPS,COMM_WAITING_TIME);
                  }
                  channelCounter = channelCounter >= 2 ? 0 : ++channelCounter; 
                  Max_Fails_counter++;
#if defined(DEBUG)
                {
                  CoopMutexLock serialLock(serialMutex);                        
                  Serial.print(F("ERRORES: "));
                  Serial.println(Max_Fails_counter);
                }
#endif
                  if( Max_Fails_counter >= COMM_ERROR_TOLERANCE)
                  {
                    digitalWrite(RESET_GPIO, HIGH);
                    Max_Fails_counter = 0;
                  }
                }
                else
                {
                  {
                  CoopMutexLock Lock(spiMutex);
                  mesh.begin(CHANNEL_NO2);
                  }
                }
             }else
             {
               
              {
              CoopMutexLock serialLock(serialMutex);
              Serial.println(F("Send fail, Test OK"));
              }
             }
           }
         }
       }else
       {
#if defined(DEBUG)
       {
        CoopMutexLock serialLock(serialMutex);          
        Serial.println(" OK ");
       }
#endif
       counter=0;
       //delay(PERIOD_TIME);
       }
     }
   }
  delay(PERIOD_TIME);
 }
}

/* --- TASK 2 Update The System---*/
void Update_System() noexcept
{
   for(;;){
    // Send to the master node every 
    {
    CoopMutexLock Lock(spiMutex);
    mesh.update();
    }
    taskSema.post();
    delay(UPDATE_TIME);    

    // Refresh The ALARM/FAIL Contact states
    Contact_Validation();
   }
}


/* --- Stack Management ---*/
void printStackReport(CoopTaskBase* task)
{
  if (!task)
    return;
  {
  CoopMutexLock serialLock(serialMutex);  
  Serial.print(task->name().c_str());
  Serial.print(F(" free stack = "));
  Serial.println(task->getFreeStack());
  }
}

void printReport()
{
   printStackReport(task1);
   printStackReport(task2);
}

// Task no.2: Report of available stack.
void loop4() noexcept
{
    for (;;) // explicitly run forever without returning
    {
        delay(5000);
        printReport();
        yield();
    }
}

void setup() {
  pinMode(RESET_GPIO, OUTPUT);
  pinMode(ALARM_GPIO, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(RESET_GPIO, LOW);    // turn the LED off by making the voltage LOW
  Serial.begin(BAUD_RATE);
  // put your setup code here, to run once:
  mesh.setNodeID(nodeID);
  // Connect to the mesh
  Serial.println(F("Connecting to the mesh..."));
  mesh.begin(CHANNEL_NO2,RF24_1MBPS,RF_START_UP_TIME);

  task1 = new CoopTask<void>(F("Task1"), Control,0x5DC); //400 FREE
  if (!*task1)
    Serial.println("CoopTask T1 out of stack");
    
  task2 = new CoopTask<void>(F("Task1"), Update_System,0x400); 
  if (!*task2)
    Serial.println("CoopTask T2 out of stack");  
  
  taskReport = new CoopTask<void, CoopTaskStackAllocatorFromLoop<>>(F("TaskReport"), loop4,0x400);
  if (!*taskReport)
    Serial.println("CoopTask T4 out of stack");
    
  CoopTaskBase::useBuiltinScheduler();
    
  if (!task1->scheduleTask())
    Serial.printf("Could not schedule task %s\n", task1->name().c_str());
    
  if (!task2->scheduleTask())
    Serial.printf("Could not schedule task %s\n", task2->name().c_str());  
  
  if (!taskReport->scheduleTask())
    Serial.printf("Could not schedule task %s\n", taskReport->name().c_str());
}

void loop()
{
  // put your main code here, to run repeatedly:
  runCoopTasks();
}
