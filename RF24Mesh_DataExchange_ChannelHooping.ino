/** RF24Mesh_Example.ino by TMRh20

   This example sketch shows how to manually configure a node via RF24Mesh, and send data to the
   master node.
   The nodes will refresh their network address as soon as a single write fails. This allows the
   nodes to change position in relation to each other and the master node.
*/


#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>
//#include <printf.h>

uint8_t channels1[] = {117,120,122};
uint8_t channels2[] = {110,113,115};
uint8_t channelCounter = 0;
uint8_t Max_Fails_counter = 0;
uint8_t Alarm_State=0;
int Fail_State=0;
/**** Configure the nrf24l01 CE and CS pins ****/
RF24 radio(16,0);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

/**
   User Configuration: nodeID - A unique identifier for each radio. Allows addressing
   to change dynamically with physical changes to the mesh.

   In this example, configuration takes place below, prior to uploading the sketch to the device
   A unique value from 1-255 must be configured for each node.
   This will be stored in EEPROM on AVR devices, so remains persistent between further uploads, loss of power, etc.

 **/
 
#define nodeID 2
#define COMM_TRIES 3

const int analogInPin = A0;
uint32_t displayTimer = 0;
uint32_t ReadTimer = 0;
bool hoopingchannel=true;
int counter=0;

int Code[3]={987,654,321};
String TCode[3]={"Alarma","Falla","Normal"};

void CHIP_ON (){
  bool result = radio.isChipConnected();
  Serial.println("Is this thing on?");
  while (!result){
    Serial.println("Naah");
    result = radio.isChipConnected();
    delay(2000);
  }
  Serial.println("HELL YEAH!");
}

struct payload_t {
  unsigned long ms;
  unsigned long counter;
};

void setup() {
  pinMode(4, OUTPUT);
  pinMode(5, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(4, LOW);    // turn the LED off by making the voltage LOW
  Serial.begin(115200);
  //printf_begin();
  //  CHIP_ON();
  // Set the nodeID manually
  mesh.setNodeID(nodeID);
  // Connect to the mesh
  Serial.println(F("Connecting to the mesh..."));
  mesh.begin(120,RF24_1MBPS,2000);
}



void loop() {

    // Send to the master node every second

  mesh.update();

    Alarm_State = digitalRead(5);
    Fail_State = analogRead(analogInPin);
    Fail_State = map(Fail_State, 0, 1023, 0, 33);
    
    if((Alarm_State== HIGH)||(Fail_State>=3)){
        digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
        Serial.println(F("ALARM and FAIL BUTTON!! "));
    }
    else{
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      Serial.println(F("NORMAL_ALARM_BUTTON"));
    }
    
    
  // Send to the master node every second
  if (millis() - displayTimer >= 500) {
    displayTimer = millis();
    // Send an 'M' type message containing the current millis()
    
        Serial.println(F("New Cycle "));
    for (int i=0;i<=2;i++){
      for (int j=0;j<=6;j++){
        Serial.print("Codigo --> ");
        Serial.println(TCode[i]);
        Serial.print(F(" Status: "));
    if (!mesh.write(&Code[i], 'M', sizeof(int))) {

      // If a write fails, check connectivity to the mesh network
      if ( ! mesh.checkConnection() ) {
        counter++;
        if(counter>=COMM_TRIES){
          counter=0;
          //refresh the network address
          Serial.println("Renewing Address");
          //if(!mesh.renewAddress(1000)){
          if(!mesh.renewAddress(0)){
            //If address renewal fails, reconfigure the radio and restart the mesh
            //This allows recovery from most if not all radio errors
            if(hoopingchannel==true){
              Serial.print("Channel: ");
              Serial.println(channels1[channelCounter]);
              //mesh.begin(channels1[channelCounter],RF24_1MBPS,2000);
              mesh.begin(channels1[channelCounter],RF24_1MBPS,0);
              channelCounter = channelCounter >= 2 ? 0 : ++channelCounter;
              
              Max_Fails_counter++;
              Serial.print("ERRORES: ");
              Serial.println(Max_Fails_counter);
              //if( Max_Fails_counter == 250){
              if( Max_Fails_counter == 250){
                digitalWrite(4, HIGH);
              }
            }
            else{
              mesh.begin(120);
            }
          } else {
            Serial.println("Send fail, Test OK");
          }
      }
    }
    } else {
      Serial.println(" OK ");
      counter=0;
      //delay(2000);
      delay(500);
    }
  }
  }
  //delay(2000);
  }

/*
  while (network.available()) {
    RF24NetworkHeader header;
    payload_t payload;
    network.read(header, &payload, sizeof(payload));
    Serial.print("Received packet #");
    Serial.print(payload.counter);
    Serial.print(" at ");
    Serial.println(payload.ms);
  }
  */
  }
  
