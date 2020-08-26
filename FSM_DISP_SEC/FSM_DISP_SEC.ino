void setup() {
  // put your setup code here, to run once:

}

typedef enum RF_FSM{Update, Send, Val, Recovery}RF_FSM;

void Update_Validation() {
  
}

void Send_Validation() {
  
}

void RF_Validation() {
  
}

void WRITING_FSM(RF_FSM state){
  switch (state){
    case Update:
      Update_Validation();
      break;
    case Send:
      Send_Validation();
      break;
    case Val:
      RF_Validation();
      break;    
    case Recovery:
      delay(10);
      break;  
  }
}



void loop() {
  // put your main code here, to run repeatedly:

}
