/*
  USB Autosteer code For AgOpenGPS
  4 Feb 2021, Brian Tischler
  Modified by Alan Webb for CAN bus usage on the Teensy4.1
  Like all Arduino code - copied from somewhere else :)
  So don't claim it as your own

*/

////////////////// User Settings /////////////////////////

//How many degrees before decreasing Max PWM
#define LOW_HIGH_DEGREES 5.0

/*  PWM Frequency ->
     490hz (default) = 0
     122hz = 1
     3921hz = 2
*/
#define PWM_Frequency 0

/////////////////////////////////////////////

// if not in eeprom, overwrite
#define EEP_Ident 4410

//version in AOG ex. v4.6.20
#define aogVersion 20

// Address of CMPS14 shifted right one bit for arduino wire library
#define CMPS14_ADDRESS 0x60

// BNO08x definitions
#define REPORT_INTERVAL 90 //Report interval in ms (same as the delay at the bottom)

//   ***********  Motor drive connections  **************888
//Connect ground only for cytron, Connect Ground and +5v for IBT2

//Dir1 for Cytron Dir, Both L and R enable for IBT2
#define DIR1_RL_ENABLE  4  //PD4

//PWM1 for Cytron PWM, Left PWM for IBT2
#define PWM1_LPWM  3  //PD3

//Not Connected for Cytron, Right PWM for IBT2
#define PWM2_RPWM  9 //D9

//--------------------------- Switch Input Pins ------------------------
//#define STEERSW_PIN 6 //PD6
#define WORKSW_PIN 7  //PD7
#define REMOTE_PIN 8  //PB0

#include <Wire.h>
#include <EEPROM.h>
//ads #include "zADS1115.h"
//ads ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);     // Use this for the 16-bit version ADS1115
#include "BNO08x_AOG.h"

//+++++++++++++++++++++++++++++++CAN+++++++++++++++++++++++++++++++++++++++

#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> K_Bus;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> ISO_Bus;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> V_Bus;

byte navController[8]  = {0x00, 0x00, 0xA0, 0x27, 0x00, 0x17, 0x02, 0x20} ;    //  Fendt Set, 2C Navagation Controller
byte claimISOBus[8]    = {0x00, 0x00, 0xA0, 0x27, 0x00, 0x17, 0x02, 0x20} ;    //  Fendt claim isobus
byte curveData[6]      = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0} ;                      //  Fendt Curve Command
byte request[2]        = {0x5, 0x19} ;                                         //  Valve request
byte goPress[8]        = {0x15, 0x22, 0x06, 0xCA, 0x80, 0x01, 0x00, 0x00} ;    //  press little go
byte goLift[8]         = {0x15, 0x22, 0x06, 0xCA, 0x00, 0x02, 0x00, 0x00} ;    //  lift little go
byte endPress[8]       = {0x15, 0x23, 0x06, 0xCA, 0x80, 0x03, 0x00, 0x00} ;    //  press little end
byte endLift[8]        = {0x15, 0x23, 0x06, 0xCA, 0x00, 0x04, 0x00, 0x00} ;    //  lift little end
int16_t canSteerPosition = 0;  //can steer variable
bool goDown = false, endDown = false , bitState = false, bitStateOld = false;

//-----------------------------CAN---------------------------------------

//loop time variables in microseconds
const unsigned int LOOP_TIME = 40;  //25Hz
unsigned long lastTime = LOOP_TIME;
unsigned long currentTime = LOOP_TIME;

const unsigned int WATCHDOG_THRESHOLD = 100;
const unsigned int WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
byte watchdogTimer = WATCHDOG_FORCE_VALUE;

//Parsing PGN
bool isPGNFound = false, isHeaderFound = false;
byte pgn = 0, dataLength = 0, idx = 0;
int tempHeader = 0;

//show life in AgIO
byte helloAgIO[] = {0x80, 0x81, 0x7f, 0xC7, 1, 0, 0x47 };
byte helloCounter = 0;

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, Heading-7,8,
//Roll-9,10, SwitchByte-11, pwmDisplay-12, CRC 13
byte AOG[] = {0x80, 0x81, 0x7f, 0xFD, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };
int AOGSize = sizeof(AOG);

// booleans to see if we are using CMPS or BNO08x
bool useCMPS = false;
bool useBNO08x = false;

// BNO08x address variables to check where it is
const byte bno08xAddresses[] = {0x4A, 0x4B};
const int nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
byte bno08xAddress;
BNO080 bno08x;

float bno08xHeading = 0;
double bno08xRoll = 0;
double bno08xPitch = 0;

int bno08xHeading10x = 0;
int bno08xRoll10x = 0;

//EEPROM
int EEread = 0;

//Relays
bool isRelayActiveHigh = true;
byte relay = 0, relayT_Hi = 0, uTurn = 0;
byte tram = 0;

//Switches
byte remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

//On Off
byte guidanceStatus = 0;

//speed sent as *10
float gpsSpeed = 0;

byte relayHi = 0, relayLo = 0,  tramline = 0, tree = 0,  hydLift = 0;

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0; //the desired angle from AgOpen
int steeringPosition = 0; //from steering sensor
float steerAngleError = 0; //setpoint - actual

//pwm variables
int pwmDrive = 0, pwmDisplay = 0;
float pValue = 0;
float errorAbs = 0;
float highLowPerDeg = 0;

//Steer switch button  ***********************************************************************************************************
byte currentState = 1, reading, previous = 0;
byte pulseCount = 0; // Steering Wheel Encoder
bool encEnable = false; //debounce flag
byte thisEnc = 0, lastEnc = 0;

//Variables for settings
struct Storage {
  byte Kp = 40;  //proportional gain
  byte lowPWM = 10;  //band of no action
  int wasOffset = 0;
  byte minPWM = 9;
  byte highPWM = 60;//max PWM value
  float steerSensorCounts = 30;
  float AckermanFix = 1;     //sent as percent
};  Storage steerSettings;  //14 bytes

//Variables for settings - 0 is false
struct Setup {
  byte InvertWAS = 0;
  byte isRelayActiveHigh = 0; //if zero, active low (default)
  byte MotorDriveDirection = 0;
  byte SingleInputWAS = 1;
  byte CytronDriver = 0;
  byte SteerSwitch = 1;  //0 button, 1 on-off switch
  byte ShaftEncoder = 0;
  byte PulseCountMax = 5;
  byte isDanfoss = 0;
};  Setup steerConfig;          //9 bytes

//reset function
void(* resetFunc) (void) = 0;

void setup()

{
  delay(3000);
  CAN_setup();
  delay(10);




  //keep pulled high and drag low to activate, noise free safe
  pinMode(WORKSW_PIN, INPUT_PULLUP);
  // pinMode(STEERSW_PIN, INPUT_PULLUP);
  pinMode(REMOTE_PIN, INPUT_PULLUP);
  pinMode(DIR1_RL_ENABLE, OUTPUT);

  if (steerConfig.CytronDriver) pinMode(PWM2_RPWM, OUTPUT);

  //set up communication
  Wire.begin();
  Serial.begin(115200);
  delay(1000);
  //test if CMPS working
  byte error;
  Wire.beginTransmission(CMPS14_ADDRESS);
  error = Wire.endTransmission();

  if (error == 0)
  {
    Serial.println("Error = 0");
    Serial.print("CMPS14 ADDRESs: 0x");
    Serial.println(CMPS14_ADDRESS, HEX);
    Serial.println("CMPS14 Ok.");
    useCMPS = true;
  }
  else
  {
    Serial.println("Error = 4");
    Serial.println("CMPS not Connected or Found");
    useCMPS = false;
  }

  // Check for BNO08x
  if (!useCMPS)
  {
    for (int i = 0; i < nrBNO08xAdresses; i++)
    {
      bno08xAddress = bno08xAddresses[i];

      Serial.print("\r\nChecking for BNO08X on ");
      Serial.println(bno08xAddress, HEX);
      Wire.beginTransmission(bno08xAddress);
      error = Wire.endTransmission();

      if (error == 0)
      {
        Serial.println("Error = 0");
        Serial.print("BNO08X ADDRESs: 0x");
        Serial.println(bno08xAddress, HEX);
        Serial.println("BNO08X Ok.");

        // Initialize BNO080 lib
        if (bno08x.begin(bno08xAddress))
        {
          Wire.setClock(400000); //Increase I2C data rate to 400kHz

          // Use gameRotationVector
          bno08x.enableGameRotationVector(REPORT_INTERVAL); //Send data update every REPORT_INTERVAL in ms for BNO085

          // Retrieve the getFeatureResponse report to check if Rotation vector report is corectly enable
          if (bno08x.getFeatureResponseAvailable() == true)
          {
            if (bno08x.checkReportEnable(SENSOR_REPORTID_GAME_ROTATION_VECTOR, REPORT_INTERVAL) == false) bno08x.printGetFeatureResponse();

            // Break out of loop
            useBNO08x = true;
            break;
          }
          else
          {
            Serial.println("BNO08x init fails!!");
          }
        }
        else
        {
          Serial.println("BNO080 not detected at given I2C address.");
        }
      }
      else
      {
        Serial.println("Error = 4");
        Serial.println("BNO08X not Connected or Found");
      }
    }
  }

  //50Khz I2C
  TWBR = 144;

  EEPROM.get(0, EEread);              // read identifier

  if (EEread != EEP_Ident)   // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(10, steerSettings);
    EEPROM.put(40, steerConfig);
  }
  else
  {
    EEPROM.get(10, steerSettings);     // read the Settings
    EEPROM.get(40, steerConfig);
  }

  // for PWM High to Low interpolator
  highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;



}// End of Setup

void loop()
{
  // Loop triggers every 100 msec and sends back steer angle etc
  currentTime = millis();

  if (currentTime - lastTime >= LOOP_TIME)
  {
    lastTime = currentTime;

    //reset debounce
    encEnable = true;

    //If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    if (watchdogTimer++ > 250) watchdogTimer = WATCHDOG_FORCE_VALUE;

    //read all the switches
    workSwitch = digitalRead(WORKSW_PIN);  // read work switch



    remoteSwitch = digitalRead(REMOTE_PIN); //read auto steer enable switch open = 0n closed = Off
    switchByte = 0;
    switchByte |= (remoteSwitch << 2); //put remote in bit 2
    switchByte |= (steerSwitch << 1);   //put steerswitch status in bit 1 position
    switchByte |= workSwitch;

    //get steering position



    steeringPosition = (canSteerPosition + 32768) / 5 ; //bit shift by 2  0 to 13610 is 0 to 5v

    //DETERMINE ACTUAL STEERING POSITION

    //convert position to steer angle. 32 counts per degree of steer pot position in my case
    //  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
    if (steerConfig.InvertWAS)
    {
      steeringPosition = (steeringPosition - 6805  - steerSettings.wasOffset);   // 1/2 of full scale
      steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
    }
    else
    {
      steeringPosition = (steeringPosition - 6805  + steerSettings.wasOffset);   // 1/2 of full scale
      steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
    }

    //Ackerman fix
    if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);

    if (watchdogTimer < WATCHDOG_THRESHOLD)
    {
     
     

      steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error
      //if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;

      calcSteeringPID();  //do the pid
      sendSteer();       //out to motors the pwm value
    }
    else
    {
      //we've lost the comm to AgOpenGPS, or just stop request
     

      pwmDrive = 0; //turn off steering motor
      sendSteer(); //out to motors the pwm value
      pulseCount = 0;
    }

    //send empty pgn to AgIO to show activity
    if (++helloCounter > 10)
    {
      Serial.write(helloAgIO, sizeof(helloAgIO));
      helloCounter = 0;
    }

    SetRelays();


  } //end of timed loop

  //This runs continuously, not timed //// Serial Receive Data/Settings /////////////////

  // Serial Receive
  //Do we have a match with 0x8081?
  if (Serial.available() > 1 && !isHeaderFound && !isPGNFound)
  {
    byte temp = Serial.read();
    if (tempHeader == 0x80 && temp == 0x81)
    {
      isHeaderFound = true;
      tempHeader = 0;
    }
    else
    {
      tempHeader = temp;     //save for next time
      return;
    }
  }

  //Find Source, PGN, and Length
  if (Serial.available() > 2 && isHeaderFound && !isPGNFound)
  {
    Serial.read(); //The 7F or less
    pgn = Serial.read();
    dataLength = Serial.read();
    isPGNFound = true;
    idx = 0;
  }

  //The data package
  if (Serial.available() > dataLength && isHeaderFound && isPGNFound)
  {


    //+++++++++++++++++++++++++++++++++++++++++++++++++++ Machine++++++++++++++++++++++++++++


    if (pgn == 239) // EF Machine Data
    {
      uTurn = Serial.read();
      tree = Serial.read();

      hydLift = Serial.read();
      tramline = Serial.read();

      //just get the rest of bytes
      Serial.read();   //high,low bytes
      Serial.read();

      relayLo = Serial.read();          // read relay control from AgOpenGPS
      relayHi = Serial.read();

      //Bit 13 CRC
      Serial.read();

      //reset watchdog
      watchdogTimer = 0;

      //Reset serial Watchdog
      // serialResetTimer = 0;

      //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn = dataLength = 0;
    }


    if (pgn == 254) //FE AutoSteerData
    {
      //bit 5,6
      gpsSpeed = ((float)(Serial.read() | Serial.read() << 8 )) * 0.1;

      //bit 7
      guidanceStatus = Serial.read();

      //Bit 8,9    set point steer angle * 100 is sent
      steerAngleSetPoint = ((float)(Serial.read() | Serial.read() << 8 )) * 0.01; //high low bytes

      if ((bitRead(guidanceStatus, 0) == 0) || (gpsSpeed < 0.5) || (steerSwitch == 1) )
      {
        watchdogTimer = WATCHDOG_FORCE_VALUE; //turn off steering motor
      }
      else          //valid conditions to turn on autosteer
      {
        watchdogTimer = 0;  //reset watchdog
      }

      //Bit 10 Tram
      tram = Serial.read();

      //Bit 11 section 1 to 8
      relay = Serial.read();

      //Bit 12 section 9 to 16
      relayT_Hi = Serial.read();


      //Bit 13 CRC
      Serial.read();

      //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn = dataLength = 0;

      //----------------------------------------------------------------------------
      //Serial Send to agopenGPS

      int sa = (int)(steerAngleActual * 100);
      AOG[5] = (byte)sa;
      AOG[6] = sa >> 8;

      if (useCMPS)
      {
        Wire.beginTransmission(CMPS14_ADDRESS);
        Wire.write(0x02);
        Wire.endTransmission();

        Wire.requestFrom(CMPS14_ADDRESS, 2);
        while (Wire.available() < 2);

        //the heading x10
        AOG[8] = Wire.read();
        AOG[7] = Wire.read();

        Wire.beginTransmission(CMPS14_ADDRESS);
        Wire.write(0x1C);
        Wire.endTransmission();

        Wire.requestFrom(CMPS14_ADDRESS, 2);
        while (Wire.available() < 2);

        //the roll x10
        AOG[10] = Wire.read();
        AOG[9] = Wire.read();
      }
      else if (useBNO08x)
      {
        if (bno08x.dataAvailable() == true)
        {
          bno08xHeading = (bno08x.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
          bno08xHeading = -bno08xHeading; //BNO085 counter clockwise data to clockwise data

          if (bno08xHeading < 0 && bno08xHeading >= -180) //Scale BNO085 yaw from [-180°;180°] to [0;360°]
          {
            bno08xHeading = bno08xHeading + 360;
          }

          bno08xRoll = (bno08x.getRoll()) * 180.0 / PI; //Convert roll to degrees
          bno08xPitch = (bno08x.getPitch()) * 180.0 / PI; // Convert pitch to degrees

          bno08xHeading10x = (int)(bno08xHeading * 10);
          bno08xRoll10x = (int)(bno08xRoll * 10);

          Serial.print(bno08xHeading10x);
          Serial.print(",");
          Serial.println(bno08xRoll10x);

          //the heading x10
          AOG[7] = (byte)bno08xHeading10x;
          AOG[8] = bno08xHeading10x >> 8;


          //the roll x10
          AOG[9] = (byte)bno08xRoll10x;
          AOG[10] = bno08xRoll10x >> 8;
        }
      }
      else
      {
        //heading
        AOG[7] = (byte)9999;
        AOG[8] = 9999 >> 8;

        //roll
        AOG[9] = (byte)8888;
        AOG[10] = 8888 >> 8;
      }

      AOG[11] = switchByte;
      AOG[12] = (byte)pwmDisplay;

      //add the checksum
      int CK_A = 0;
      for (byte i = 2; i < AOGSize - 1; i++)
      {
        CK_A = (CK_A + AOG[i]);
      }

      AOG[AOGSize - 1] = CK_A;

      Serial.write(AOG, AOGSize);

      // Stop sending the helloAgIO message
      helloCounter = 0;
      //--------------------------------------------------------------------------
    }

    else if (pgn == 252) //FC AutoSteerSettings
    {
      //PID values
      steerSettings.Kp = ((float)Serial.read());   // read Kp from AgOpenGPS
      steerSettings.Kp *= 0.5;

      steerSettings.highPWM = Serial.read();

      steerSettings.lowPWM = (float)Serial.read();   // read lowPWM from AgOpenGPS

      steerSettings.minPWM = Serial.read(); //read the minimum amount of PWM for instant on

      steerSettings.steerSensorCounts = Serial.read(); //sent as setting displayed in AOG

      steerSettings.wasOffset = (Serial.read());  //read was zero offset Hi

      steerSettings.wasOffset |= (Serial.read() << 8);  //read was zero offset Lo

      steerSettings.AckermanFix = (float)Serial.read() * 0.01;

      //crc
      //udpData[13];        //crc
      Serial.read();

      //store in EEPROM
      EEPROM.put(10, steerSettings);

      // for PWM High to Low interpolator
      highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;

      //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn = dataLength = 0;
    }

    else if (pgn == 251) //FB - steerConfig
    {
      byte sett = Serial.read();

      if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
      if (bitRead(sett, 1)) steerConfig.isRelayActiveHigh = 1; else steerConfig.isRelayActiveHigh = 0;
      if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
      if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
      if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
      if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
      if (bitRead(sett, 6)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

      steerConfig.PulseCountMax = Serial.read();

      //was speed
      Serial.read();

      //Danfoss type hydraulics
      steerConfig.isDanfoss = Serial.read(); //byte 8

      Serial.read(); //byte 9
      Serial.read(); //byte 10

      Serial.read(); //byte 11
      Serial.read(); //byte 12

      //crc byte 13
      Serial.read();

      EEPROM.put(40, steerConfig);

      //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn = dataLength = 0;

      //reset the arduino
      //resetFunc();
    }

    //clean up strange pgns
    else
    {
      //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn = dataLength = 0;
    }

  } //end if (Serial.available() > dataLength && isHeaderFound && isPGNFound)

  if (encEnable)
  {
    thisEnc = digitalRead(REMOTE_PIN);
    if (thisEnc != lastEnc)
    {
      lastEnc = thisEnc;
      if ( lastEnc) EncoderFunc();
    }
  }

} // end of main loop

//ISR Steering Wheel Encoder
void EncoderFunc()
{
  if (encEnable)
  {
    pulseCount++;
    encEnable = false;
  }
}

void SetRelays(void)
{
  if (goDown)   liftGo();
  if (endDown)   liftEnd();

  bitState = (bitRead(relayLo, 0));
  if (bitState  && !bitStateOld) pressGo();
  if (!bitState && bitStateOld) pressEnd();

  bitStateOld = bitState;

}
