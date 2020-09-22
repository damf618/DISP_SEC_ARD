#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>

#define WRITING_TIME 60

struct payload_t {
  unsigned long ms;
  unsigned long counter;
};

/**** Configure the nrf24l01 CE and CS pins ****/
//RF24 radio(16,0);
RF24 radio(7, 8);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

//int Code[3]={987,654,321};
//String TCode[3]={"Alarma","Falla","Normal"};

int Code[3]={987,987,987};
String TCode[3]={"Alarma","Alarma","Alarma"};

uint32_t displayTimer = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(F("Monitor Init"));
  delay(500);
  Serial.println(F("Connecting to the mesh..."));
  mesh.begin();
  mesh.setNodeID(12);
  radio.setChannel(120);
}

void loop() {
  // put your main code here, to run repeatedly:
    mesh.update();

    //if (millis() - displayTimer >= WRITING_TIME) {
    while(1){
    //displayTimer = millis();
   
    Serial.println(F("New Cycle "));
    for (int i=0;i<=2;i++){
      for (int j=0;j<=6;j++){
        Serial.print("Codigo --> ");
        Serial.println(TCode[i]);
        Serial.print(F(" Status: "));
       
        // Send an 'M' type message containing the current millis()
        if (!mesh.write(&Code[i], 'M', sizeof(int)))
        {
        // If a write fails, check connectivity to the mesh network
        if ( ! mesh.checkConnection() )
        {
          //refresh the network address
          Serial.println("Renewing Address");
          if(!mesh.renewAddress()){
            //If address renewal fails, reconfigure the radio and restart the mesh
            //This allows recovery from most if not all radio errors
            mesh.begin();
            radio.setChannel(120);
          }
        } else
        {
          Serial.println("Send fail, Test OK");
        }
        } else
        {
          Serial.print("Send OK: "); Serial.println(displayTimer);
        }
        delay(WRITING_TIME);  
      }
   }
  }
    while (network.available()) {
    RF24NetworkHeader header;
    payload_t payload;
    network.read(header, &payload, sizeof(payload));
    Serial.print("Received packet #");
    Serial.print(payload.counter);
    Serial.print(" at ");
    Serial.println(payload.ms);
  }
}
