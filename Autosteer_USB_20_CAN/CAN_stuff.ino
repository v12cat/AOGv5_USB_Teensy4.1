void CAN_setup (void) {

  byte navController[8] = {0x00, 0x00, 0xA0, 0x27, 0x00, 0x17, 0x02, 0x20} ;  //Fendt Set, 2C Navagation Controller
  byte claimISOBus[8] = {0x00, 0x00, 0xA0, 0x27, 0x00, 0x17, 0x02, 0x20} ; 

   
//++++++++++++++++++++++++++++  BUS Setup  +++++++++++++++++++++++++++++++++++++++++++++++
  
V_Bus.begin();
  V_Bus.setBaudRate(250000);
  V_Bus.setMaxMB(16);
  V_Bus.enableFIFO();
  V_Bus.enableFIFOInterrupt();
  V_Bus.onReceive(V_Bus_stuff);
  V_Bus.mailboxStatus();
  V_Bus.setFIFOFilter(REJECT_ALL);
  V_Bus.setFIFOFilter(0, 0x0CEF2CF0, EXT);
  
  CAN_message_t msgV;
  msgV.id = 0x18EEFF2C;
  msgV.flags.extended = true;
  msgV.len = 8;
  for ( uint8_t i = 0; i <= sizeof(navController); i++ ) msgV.buf[i] = navController[i];
   V_Bus.write(msgV);
  
ISO_Bus.begin();
  ISO_Bus.setBaudRate(250000);
  ISO_Bus.setMaxMB(16);
  ISO_Bus.enableFIFO();
  ISO_Bus.enableFIFOInterrupt();
  ISO_Bus.onReceive(ISO_Bus_stuff);
  ISO_Bus.mailboxStatus();
  ISO_Bus.setFIFOFilter(REJECT_ALL);
  ISO_Bus.setFIFOFilter(0,0x18EF2CF0, EXT);
  CAN_message_t msgI;
  msgI.id = 0x18EEFF2C;
  msgI.flags.extended = true;
  msgI.len = 8;
  for ( uint8_t i = 0; i <= sizeof(claimISOBus); i++ ) msgI.buf[i] = claimISOBus[i];
   ISO_Bus.write(msgI);

  K_Bus.begin();
  K_Bus.setBaudRate(250000);
  K_Bus.setMaxMB(16);
  K_Bus.enableFIFO();
  K_Bus.enableFIFOInterrupt();
  K_Bus.onReceive(K_Bus_stuff);
  K_Bus.mailboxStatus();
  K_Bus.setFIFOFilter(REJECT_ALL);
  K_Bus.setFIFOFilter(0, 0x613, STD);
  
  
  //+++++++++++++++++++++++++++++++  Steer Switching  +++++++++++++++++++++++++++++++++++++++
}

void V_Bus_stuff(const CAN_message_t &msg) {

     if (msg.len == 3 && (msg.buf[2] == 0)) steerSwitch = 1;      // steer disable via V bus

     if (msg.len == 8 && msg.buf[0] == 5 && msg.buf[1] == 10) 
          canSteerPosition = ((msg.buf[4] << 8) + msg.buf[5]);    // WAS reading
}

void ISO_Bus_stuff(const CAN_message_t &msg) {

   if ((msg.buf[0])== 0x0F && (msg.buf[1])== 0x60 && (msg.buf[2])== 0x01) steerSwitch = 0;                         //steer enable via ISO bus
 
}

void K_Bus_stuff(const CAN_message_t &msg) {

  if (msg.buf[0]==0x15 and msg.buf[2]==0x06 and msg.buf[3]==0xCA){

    if(msg.buf[1]==0x8A and msg.buf[4]==0x80) steerSwitch = 1;        // turn AOG steer off if tractor steer is cancelled completely
  }
                                                            
  }


//+++++++++++++++++++++++++++++++++++++ Send steer angle to CAN ++++++++++++++++++++++++++++++++++++++++

void sendSteer(void){

 // ----------------------- Alter steer value according to PWM-------------------------

    float shift = (sq(pwmDrive)/32);            //  trying the square divided down method again!!
    if (pwmDrive< 0) shift *= -1;               // reinstate negative cancelled by square
     steerValue += shift;                       // using a float steer value to prevent rollover etc.
     if (steerValue > 32767) steerValue = 32767;
    if (steerValue < -32768) steerValue = -32768;
    setCurve = (int16_t) steerValue;
    
  //----------------------------------------------------------------------------------
  
    CAN_message_t curveData;
    curveData.id = 0x0CEFF02C;
    curveData.flags.extended = true;
    curveData.len = 6;
    curveData.buf[0] = 5;
    curveData.buf[1] = 9;
    curveData.buf[2] = (!steerSwitch + 2);
    curveData.buf[3] = 10;
    if (steerSwitch == 0){                         //     send steer value to CAN when steer enabled
      curveData.buf[4] = highByte(setCurve);
      curveData.buf[5] = lowByte(setCurve);
    }
    else{
      curveData.buf[4] = 0;
      curveData.buf[5] = 0;
      steerValue = (float) canSteerPosition;       //    Keep steer value similar to steer position if not steering
    }                                              //     to prevent jumps when re-engaging   
V_Bus.write(curveData);
pwmDisplay = pwmDrive;

    CAN_message_t requestData;
    requestData.id = 0x0CEFF02C;
    requestData.flags.extended = true;
    requestData.len = 2;
    requestData.buf[0] = 0x5;
    requestData.buf[1] = 0x19;
    V_Bus.write(requestData);

    
}


//++++++++++++++++++++++++++  K_Bus Buttons ++++++++++++++++++++++++++++++++++++++++


void pressGo()
{                                    
 CAN_message_t buttonData;
 buttonData.id = 0X613;
 buttonData.len = 8;
for ( uint8_t i = 0; i <= sizeof(goPress); i++ ) buttonData.buf[i] = goPress[i];
   K_Bus.write(buttonData);
   goDown = true;
   
}


void liftGo()
{
  CAN_message_t buttonData;
 buttonData.id = 0X613;
 buttonData.len = 8;
for ( uint8_t i = 0; i <= sizeof(goLift); i++ ) buttonData.buf[i] = goLift[i];
   K_Bus.write(buttonData);
   goDown = false;

}

void pressEnd() {
  CAN_message_t buttonData;
 buttonData.id = 0X613;
 buttonData.len = 8;
for ( uint8_t i = 0; i <= sizeof(endPress); i++ ) buttonData.buf[i] = endPress[i];
   K_Bus.write(buttonData);
   endDown = true;
   
}

void liftEnd(){
CAN_message_t buttonData;
 buttonData.id = 0X613;
 buttonData.len = 8;
for ( uint8_t i = 0; i <= sizeof(endLift); i++ ) buttonData.buf[i] = endLift[i];
   K_Bus.write(buttonData);
   endDown = false;

  
}
