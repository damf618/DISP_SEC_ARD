/*=====[TP4_Testing]==========================================================
 * Copyright 2020 Author Marquez Daniel <damf618@gmail.com>
 * All rights reserved.
 * License: license text or at least name and link
         (example: BSD-3-Clause <https://opensource.org/licenses/BSD-3-Clause>)
 *
 * Version: 1.0.0
 * Creation Date: 2020/07/25
 */

/*=====[Inclusions of function dependencies]=================================*/

#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>

/**
   User Configuration: nodeID - A unique identifier for each radio. Allows addressing
   to change dynamically with physical changes to the mesh.
   In this example, configuration takes place below, prior to uploading the sketch to the device
   A unique value from 1-255 must be configured for each node.
   This will be stored in EEPROM on AVR devices, so remains persistent between further uploads, loss of power, etc.
 **/

#define nodeID 2
#define COMM_TRIES 3
#define ALARM_GPIO 5
#define ANALOG_LIM 30
#define RESET_GPIO 4
#define BAUD_RATE 115200
#define alarm_code 987
#define fail_code 654
#define normal_code 321
#define alarm_fail_code 258
#define CHANNEL_NO1 117 
#define CHANNEL_NO2 120
#define CHANNEL_NO3 122
#define N_STATES 4
#define PERIOD_TIME 500
#define COMM_WAITING_TIME 0 
#define COMM_ERROR_TOLERANCE 150
#define RF_START_UP_TIME 2000
#define MAX_ANALOG_VAL 1023
#define MIN_ANALOG_VAL 0
#define INTEGER_VOLT_VAL 33
#define N_TRIES_STATE 7

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

void CHIP_ON (){
  bool result = radio.isChipConnected();
  Serial.println("Is the Chip responding correctly?");
  while (!result){
    Serial.println("No response");
    result = radio.isChipConnected();
    delay(RF_START_UP_TIME);
  }
  Serial.println("Everything working properly!");
}

struct payload_t {
  unsigned long ms;
  unsigned long counter;
};

void setup() {
  pinMode(RESET_GPIO, OUTPUT);
  pinMode(ALARM_GPIO, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(RESET_GPIO, LOW);    // turn the LED off by making the voltage LOW
  Serial.begin(BAUD_RATE);
  //printf_begin();
  //  CHIP_ON();
  
  // Set the nodeID manually
  mesh.setNodeID(nodeID);
  // Connect to the mesh
  Serial.println(F("Connecting to the mesh..."));
  mesh.begin(CHANNEL_NO2,RF24_1MBPS,RF_START_UP_TIME);
}

void Contact_Validation(){

  Alarm_State = digitalRead(ALARM_GPIO);
    Fail_State = analogRead(analogInPin);
    Fail_State = map(Fail_State, MIN_ANALOG_VAL, MAX_ANALOG_VAL, MIN_ANALOG_VAL, INTEGER_VOLT_VAL);
    
    if((Alarm_State== HIGH)||(Fail_State>=ANALOG_LIM)){
        digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
        Serial.println(F("ALARM and FAIL BUTTON!! "));
    }
    else{
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      Serial.println(F("NORMAL_ALARM_BUTTON"));
    }
}

void loop() {
    // Send to the master node every second
  mesh.update();    
  
    Contact_Validation();
  
  // Send to the master node every second
    /* ----- TASK 1 ----- */
  if (millis() - displayTimer >= PERIOD_TIME) {
      displayTimer = millis();
    
    // Send an 'M' type message containing the current millis()
        Serial.println(F("New Cycle "));
      for (int i=0;i<N_STATES;i++){
          for (int j=0;j<N_TRIES_STATE;j++){
            Serial.print("Codigo --> ");
            Serial.println(TCode[i]);
            Serial.print(F(" Status: "));
          if (!mesh.write(&Code[i], 'M', sizeof(int))) {

   // If a write fails, check connectivity to the mesh network
              if ( ! mesh.checkConnection() ) {
                counter++;
                if(counter>=COMM_TRIES){
                    counter=0;
   // refresh the network address
                    Serial.println("Renewing Address");
                    if(!mesh.renewAddress(COMM_WAITING_TIME)){
   // If address renewal fails, reconfigure the radio and restart the mesh
   // This allows recovery from most if not all radio errors
                      if(hoopingchannel==true){
                          Serial.print("Channel: ");
                          Serial.println(channels1[channelCounter]);
                          mesh.begin(channels1[channelCounter],RF24_1MBPS,COMM_WAITING_TIME);
                          channelCounter = channelCounter >= 2 ? 0 : ++channelCounter; 
                          Max_Fails_counter++;
                          Serial.print("ERRORES: ");
                          Serial.println(Max_Fails_counter);
                          if( Max_Fails_counter == COMM_ERROR_TOLERANCE)
                  {
                            digitalWrite(RESET_GPIO, HIGH);
                          }
                      }
                      else
                {
                          mesh.begin(CHANNEL_NO2);
                      }
                    }else
              {
                      Serial.println("Send fail, Test OK");
                    }
                }
            }
          } else
        {
              Serial.println(" OK ");
              counter=0;
              delay(PERIOD_TIME);
          }
        }
      }
  }
}
