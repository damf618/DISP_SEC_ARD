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
#define PERIOD_TIME 50
#define UPDATE_TIME 30
#define COMM_WAITING_TIME 0 
#define COMM_ERROR_TOLERANCE 150
#define RF_START_UP_TIME 2000
#define MAX_ANALOG_VAL 1023
#define MIN_ANALOG_VAL 0
#define INTEGER_VOLT_VAL 33
#define N_TRIES_STATE 7
#define RECOVERY_TIME 15
#define RF_COMM_TIME 15
#define TRUE 1
#define FALSE 0

typedef enum RF_FSM_s { SEND, RF_MANAGER, RECOVERY}RF_FSM_t;
typedef enum RF_MAINT_FSM_s { TRYING, RENEWING, RESET}RF_MAINT_FSM_t;

typedef struct struct_s {
  bool Maintenance;
  RF_FSM_t state;
  RF_MAINT_FSM_t mstate;
  int counter;
  int Max_Fails_counter;
  int channelCounter;
  uint8_t Alarm_State;
  uint8_t Fail_State;
}disp_sec_t;

RF24 radio((int)16,(int)0);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

disp_sec_t sec;
const int analogInPin = A0;
const uint8_t channels1[3] = {CHANNEL_NO1,CHANNEL_NO2,CHANNEL_NO3};
const int Code[N_STATES]={alarm_code,fail_code,normal_code,alarm_fail_code};
const String TCode[N_STATES]={"Alarma","Falla","Normal","Alarma_Falla"};

CoopMutex serialMutex;
CoopMutex spiMutex;
CoopSemaphore taskUpdate(1,1);
CoopSemaphore RF_Maintenance(1,1);

CoopTask<void>* task1;
CoopTask<void>* task2;
CoopTask<void>* task3;
CoopTask<void, CoopTaskStackAllocatorFromLoop<>>* taskReport;

bool RF_Send(disp_sec_t * sec,int i)
{
  bool rtn = FALSE;
  {
    CoopMutexLock Lock(spiMutex);
    rtn= mesh.write(&Code[i], 'M', sizeof(int));   
    }
  return rtn;
}

void MESH_CONNECT_OK()
{
  {
    CoopMutexLock serialLock(serialMutex);                
    Serial.println(F("Message was not Sent, TEST OK!"));
    }   
}

void MESSAGE_SENT_OK()
{
  {
    CoopMutexLock serialLock(serialMutex);                
    Serial.print(F("Message Sent"));
    }   
}

bool Send_Maintenance_Validation(disp_sec_t * sec)
{
  RF_Maintenance.wait();
  if (FALSE != sec->Maintenance)
  {
    RF_Maintenance.post();
    yield();
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

bool Maintenance_Validation(disp_sec_t * sec)
{
  RF_Maintenance.wait();
  if (TRUE != sec->Maintenance)
  {
    RF_Maintenance.post();
    yield();
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

void Send_Validation(disp_sec_t * sec, bool arg1)
{
  bool val = FALSE;
  if (!arg1)
  {
  // If a write fails, check connectivity to the mesh network  
  {
  CoopMutexLock Lock(spiMutex);
  val= mesh.checkConnection();   
  }
  if (val)
  {
    //No message sent but RF Connectivity OK!
    MESH_CONNECT_OK();
    sec->state=RECOVERY;
  }
  else
  {
    //No mesh connection!
    sec->state=RF_MANAGER;
  }
  }
  else
  {
  //Message Sent Correctly  
    MESSAGE_SENT_OK();
    sec->counter=0;
    sec->Max_Fails_counter=0;
    sec->state=RECOVERY;
  }
}

void RF_Validation(disp_sec_t * sec)
{
  sec->Maintenance = TRUE;
#if defined(DEBUG)
  {
  CoopMutexLock serialLock(serialMutex);                
  Serial.println(F("RF Maintenance ON"));   
  }
#endif
  sec->state=SEND;
}

void WRITING_FSM(disp_sec_t * sec, int i){
  switch (sec->state){
    case SEND:
    //If there was an update, then we send the status
      taskUpdate.wait();
      Send_Validation(sec, RF_Send(sec,i));
      break;
    case RF_MANAGER:
      RF_Validation(sec);
      break;    
    case RECOVERY:
      delay(RECOVERY_TIME);
      sec->state=SEND;
      break;  
  }
    RF_Maintenance.post();
}

/* --- TASK 1 Update The System ---*/
void RF_Comm_Task () noexcept
{
  for (;;) // explicitly run forever without returning
  {
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
    if(Send_Maintenance_Validation(&sec))
    {
#if defined(DEBUG)
      {
      CoopMutexLock serialLock(serialMutex);                
      Serial.println(F("RF Maintenance being executed "));    
      }
#endif    
    continue;
    }
    WRITING_FSM(&sec,i);
    delay(RF_COMM_TIME);
    }
  }  
  }

}

/* --- Refresh the contact State ---*/
void Contact_Validation(disp_sec_t * sec)
{
  sec->Alarm_State = digitalRead(ALARM_GPIO);
  sec->Fail_State = analogRead(analogInPin);
  sec->Fail_State = map(sec->Fail_State, MIN_ANALOG_VAL, MAX_ANALOG_VAL, MIN_ANALOG_VAL, INTEGER_VOLT_VAL);
    
  if((sec->Alarm_State== HIGH)||(sec->Fail_State>=ANALOG_LIM))
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
    taskUpdate.post();
}

/* --- TASK 2 Update The System ---*/
void Update_Task() noexcept
{
   for(;;){
    // Send to the master node every 
    {
    CoopMutexLock Lock(spiMutex);
    mesh.update();
    }
  // Refresh The ALARM/FAIL Contact states
    Contact_Validation(&sec);
    delay(UPDATE_TIME);    
   }
}

void RF_Error_Handler (disp_sec_t * sec)
{
  sec->counter++;
  if(sec->counter>=COMM_TRIES)
  {
  sec->counter=0;
  sec->mstate=RENEWING;
  }  
} 

void RF_Refresh(disp_sec_t * sec){
  bool val=FALSE;
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
#if defined(DEBUG)
    {
    CoopMutexLock serialLock(serialMutex);                          
    Serial.print(F("Channel: "));
    Serial.println(channels1[sec->channelCounter]);
    }
#endif
    {
    CoopMutexLock Lock(spiMutex);
    mesh.begin(channels1[sec->channelCounter],RF24_1MBPS,COMM_WAITING_TIME);
    }
    sec->channelCounter = sec->channelCounter >= 2 ? 0 : ++sec->channelCounter; 
    sec->Max_Fails_counter++;
#if defined(DEBUG)
    {
    CoopMutexLock serialLock(serialMutex);                        
    Serial.print(F("ERRORES: "));
    Serial.println(sec->Max_Fails_counter);
    }
#endif
    if( sec->Max_Fails_counter >= COMM_ERROR_TOLERANCE)
    {
      sec->mstate=RESET;
    }
  }
  else{
    {
    CoopMutexLock serialLock(serialMutex);                        
    Serial.println(F("Addres Renewed Succesfully"));
  }
  }  
}

void FULL_SYSTEM_RESET(disp_sec_t * sec)
{
  digitalWrite(RESET_GPIO, HIGH);
  sec->Max_Fails_counter = 0;
  sec->counter=0;
  sec->mstate=TRYING;
}

void MAINTENANCE_FSM(disp_sec_t * sec){
  switch (sec->mstate){
    case TRYING:
    RF_Error_Handler(sec);
      break;
    case RENEWING:
    RF_Refresh(sec);
      break;    
    case RESET:
    FULL_SYSTEM_RESET(sec);
      break;  
  }
  sec->Maintenance = FALSE;
  RF_Maintenance.post();  
}

/* --- TASK 3 RF Maintenance ---*/
void RF_Maintenance_Task() noexcept
{
  bool val=FALSE;
  for(;;)
  {
    if(Maintenance_Validation(&sec))
    {
#if defined(DEBUG)
      {
      CoopMutexLock serialLock(serialMutex);                
      Serial.println(F("RF System OK!"));   
      }
#endif
      delay(10);        
      continue;
    }
  /* --- FSM --- */
  MAINTENANCE_FSM(&sec);
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
   printStackReport(task3);
}

// Task no.4: Report of available stack.
void Stack_Management() noexcept
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
  
  sec.Maintenance = FALSE;
  sec.state=SEND;
  sec.mstate= TRYING;
  sec.counter =0;
  sec.Max_Fails_counter=0;
  sec.channelCounter=0;
  sec.Alarm_State=0;
  sec.Fail_State=0;
  
  Serial.begin(BAUD_RATE);
  // put your setup code here, to run once:
  mesh.setNodeID(nodeID);
  // Connect to the mesh
  Serial.println(F("Connecting to the mesh..."));
  mesh.begin(CHANNEL_NO2,RF24_1MBPS,RF_START_UP_TIME);

  task1 = new CoopTask<void>(F("Task1"), RF_Comm_Task,0x410); // 136 FREE
  if (!*task1)
    Serial.println("CoopTask T1 out of stack");
    
  task2 = new CoopTask<void>(F("Task2"), Update_Task,0x370); // 168 FREE
  if (!*task2)
    Serial.println("CoopTask T2 out of stack");  
    
  task3 = new CoopTask<void>(F("Task3"), RF_Maintenance_Task,0x500); // 152 FREE 
  if (!*task3)
    Serial.println("CoopTask T3 out of stack");   
  
  taskReport = new CoopTask<void, CoopTaskStackAllocatorFromLoop<>>(F("TaskReport"), Stack_Management,0x400);
  if (!*taskReport)
    Serial.println("CoopTask T4 out of stack");
    
  CoopTaskBase::useBuiltinScheduler();
    
  if (!task1->scheduleTask())
    Serial.printf("Could not schedule task %s\n", task1->name().c_str());
    
  if (!task2->scheduleTask())
    Serial.printf("Could not schedule task %s\n", task2->name().c_str());  

  if (!task3->scheduleTask())
    Serial.printf("Could not schedule task %s\n", task2->name().c_str());  
  
  if (!taskReport->scheduleTask())
    Serial.printf("Could not schedule task %s\n", taskReport->name().c_str());
}

void loop()
{
  // put your main code here, to run repeatedly:
  runCoopTasks();
}
