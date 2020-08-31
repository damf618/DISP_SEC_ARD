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
#define RF_COMM_TIME 30
#define TRUE 1
#define FALSE 0

typedef enum RF_FSM_s { SEND, RF_MANAGER, RECOVERY}RF_FSM_t;
typedef enum RF_MAINT_FSM_s { TRYING, RENEWING, RESET}RF_MAINT_FSM_t;
typedef enum SEC_STATE_s { NORMAL_STATE, FAIL_STATE, ALARM_STATE, ALARM_FAIL_STATE}SEC_STATE_t;

typedef struct struct_s 
{
  SEC_STATE_t sec_state;  
  bool Maintenance;
  RF_FSM_t state;
  RF_MAINT_FSM_t mstate;
  int counter;
  int Max_Fails_counter;
  int channelCounter;
  uint8_t Alarm_State;
  uint8_t Fail_State;
}disp_sec_t;


RF24 radio(16,0);
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

int Code_Traslation (SEC_STATE_t state){
  
  int rtn;
  
  switch(state){
    case ALARM_FAIL_STATE:
    rtn=Code[3];  
      break;
    case ALARM_STATE:
      rtn=Code[0];
      break;  
    case FAIL_STATE:
      rtn=Code[1];    
      break;  
    case NORMAL_STATE:
      rtn=Code[2];
    break;      
  }
  return rtn;  
}

bool RF_Send(disp_sec_t * sec)
{
  int val;
  bool rtn = FALSE;
  val=Code_Traslation(sec->sec_state);
#if defined(DEBUG)
  {
  CoopMutexLock serialLock(serialMutex);                
  Serial.print(F("Code to Send: "));
  Serial.println(val);   
  Serial.print(F("State: "));
  Serial.println(sec->sec_state);   
  }
#endif  
  {
  CoopMutexLock Lock(spiMutex);
  rtn= mesh.write(&val, 'M', sizeof(int));   
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

void WRITING_FSM(disp_sec_t * sec)
{
  bool val;
  
  switch (sec->state){
    case SEND:
    //If there was an update, then we send the status
      taskUpdate.wait();
      val=RF_Send(sec);  
      Send_Validation(sec, val);
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
    Serial.println(F(" RF Send "));
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
    // Modifications of sec internals values will be made
    WRITING_FSM(&sec);
    delay(RF_COMM_TIME);
    }
}

SEC_STATE_t Digital_Validation(disp_sec_t * sec)
{
  
  int sensorValue;
  SEC_STATE_t rtn;
    
  sec->Alarm_State = digitalRead(ALARM_GPIO);
  sensorValue = analogRead(analogInPin);
  sec->Fail_State = map(sensorValue, MIN_ANALOG_VAL, MAX_ANALOG_VAL, MIN_ANALOG_VAL, INTEGER_VOLT_VAL); 

#if defined(DEBUG)
  {
  CoopMutexLock serialLock(serialMutex);                
  Serial.print(F("\n Analog Read: "));
  Serial.println(sensorValue);
  Serial.print(F("\n Conversion Read: "));      
  Serial.println(sec->Fail_State);
  }
#endif      
  
  
  if((sec->Alarm_State== HIGH)&&(sec->Fail_State>=ANALOG_LIM))
  {
    rtn = ALARM_FAIL_STATE;
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
  else if(sec->Alarm_State== HIGH)
  {
    rtn = ALARM_STATE;
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)    
  }
  else if(sec->Fail_State>=ANALOG_LIM)
  {
    rtn = FAIL_STATE;
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)  
  }
  else
  {
    rtn = NORMAL_STATE;
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  }
  return rtn;
}


/* --- Refresh the contact State ---*/
void Contact_Validation(disp_sec_t * sec)
{
  SEC_STATE_t state;
  state = Digital_Validation(sec);
  switch(state)
  {
    case(ALARM_FAIL_STATE):
      sec->sec_state= ALARM_FAIL_STATE;
#if defined(DEBUG)
      {
      CoopMutexLock serialLock(serialMutex);          
      Serial.println(F("ALARM AND FAIL EVENT!"));
      }
#endif    
      break;
    case(ALARM_STATE):
      sec->sec_state= ALARM_STATE;
#if defined(DEBUG)
      {
      CoopMutexLock serialLock(serialMutex);          
      Serial.println(F("ALARM EVENT!"));
      }
#endif      
      break;
    case(FAIL_STATE):
      sec->sec_state= FAIL_STATE;
#if defined(DEBUG)
      {
      CoopMutexLock serialLock(serialMutex);          
      Serial.println(F("FAIL EVENT!"));
      }
#endif  
      break; 
    case(NORMAL_STATE):
      sec->sec_state= NORMAL_STATE;
#if defined(DEBUG)
      {
      CoopMutexLock serialLock(serialMutex);          
      Serial.println(F("NORMAL STATE"));
      }
#endif
      break;
    default:
      sec->sec_state= FAIL_STATE;
      {
      CoopMutexLock serialLock(serialMutex);          
      Serial.println(F("UNDEFINED STATE!"));
      }
      break;   
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

void RF_Refresh(disp_sec_t * sec)
{
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

void MAINTENANCE_FSM(disp_sec_t * sec)
{
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

#if defined(DEBUG)
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
#endif

void setup()
{
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

  task1 = new CoopTask<void>(F("Task1"), RF_Comm_Task,0x410); // 184 FREE
  if (!*task1)
    Serial.println(F("CoopTask T1 out of stack"));
    
  task2 = new CoopTask<void>(F("Task2"), Update_Task,0x410); // 40 FREE
  if (!*task2)
    Serial.println(F("CoopTask T2 out of stack"));  
    
  task3 = new CoopTask<void>(F("Task3"), RF_Maintenance_Task,0x5C8); // 8 FREE 
  if (!*task3)
    Serial.println(F("CoopTask T3 out of stack"));   

  CoopTaskBase::useBuiltinScheduler();

  if (!task1->scheduleTask())
    //Serial.printf("Could not schedule task %s\n", task1->name().c_str());
    Serial.println(F("CoopTask T1 not scheduled"));  
          
  if (!task2->scheduleTask())
    Serial.println(F("CoopTask T2 not scheduled"));  

  if (!task3->scheduleTask())
    Serial.println(F("CoopTask T3 not scheduled"));    

#if defined(DEBUG)
  taskReport = new CoopTask<void, CoopTaskStackAllocatorFromLoop<>>(F("TaskReport"), Stack_Management,0x400);
  if (!*taskReport)
    Serial.println(F("CoopTask Print Report out of stack"));
  
  if (!taskReport->scheduleTask())
    Serial.println(F("Could not schedule the Print Report Task"));
#endif
}

void loop()
{
  // put your main code here, to run repeatedly:
  runCoopTasks();
}
