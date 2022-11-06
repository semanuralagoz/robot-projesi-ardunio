/* meArm analog joysticks version 1.5.4 - UtilStudio.com Dec 2018
   Uses two analogue joysticks and four servos.

   In version 1.5.4 was replaced an external resistors by internal pull up resistors.
   External LED diode was replaced with using internal "L" diode on Arduino UNO board.
   Some bugs was removed.

   First joystick moves gripper forwards, backwards, left and right,
   button start/stop recording positions.

   Second joystick moves gripper up, down, and closes and opens,
   button start/stop playing recorded positions.
   Press button for 2 seconds to autoplay.

   Pins:
   Arduino    Stick1    Stick2    Base   Shoulder  Elbow    Gripper   Record/
      GND       GND       GND    Brown     Brown   Brown     Brown    Auto play
       5V       VCC       VCC      Red       Red     Red       Red    LED
       A0       HOR
       A1       VER
       PD2      BUTT
       A2                 HOR
       A3                 VER
       PD3                BUTT
       11                       Yellow
       10                                 Yellow
        9                                         Yellow
        6                                                   Yellow
       13                                                             X
*/
#include <Servo.h>

bool repeatePlaying = false; /* Repeatedly is running recorded cycle */
int delayBetweenCycles = 2000; /* Delay between cycles */

int basePin = 11;       /* Base servo */
int shoulderPin = 10;   /* Shoulder servo */
int elbowPin = 9;       /* Elbow servo */
int gripperPin = 6;     /* Gripper servo */
int realGripperPin = 7  ;
int xdirPin = 0;        /* Base - joystick1*/
int ydirPin = 1;        /* Shoulder - joystick1 */
int zdirPin = 3;        /* Elbow - joystick2 */
int gdirPin = 2;        /* Gripper - joystick2 */

//int pinRecord = A4;     /* Button record - backward compatibility */
//int pinPlay = A5;       /* Button play  - backward compatibility */
int pinRecord = PD2;     /* Button record - recommended (A4 is deprecated, will by used for additional joystick) */
int pinPlay = PD3;       /* Button play  - recommended (A5 is deprecated, will by used for additional joystick) */
int pinLedRecord = 13;   /* LED - indicates recording (light) or auto play mode (blink ones) */

const int buffSize = 512; /* Size of recording buffer */

int startBase = 90;
int startShoulder = 90;
int startElbow = 90;
int startGripper = 90;

int posBase = 90;
int posShoulder = 90;
int posElbow = 90;
int posGripper = 90;

int lastBase = 90;
int lastShoulder = 90;
int lastElbow = 90;
int lastGripper = 90;

int minBase = 0;
int maxBase = 150;
int minShoulder = 0;
int maxShoulder = 150;
int minElbow = 0;
int maxElbow = 150;
int minGripper = 0;
int maxGripper = 150;

int angle =90;    // initial angle  for servo (beteen 1 and 179)
int angleStep =10;
const int minAngle = 0;
const int maxAngle = 150;

int buttonPushed =0;
int buttonPushed2 =0;
const int type =2;
const int countServo = 5;
int buff[buffSize];
int buffAdd[countServo];
int recPos = 0;
int playPos = 0;

int buttonRecord = HIGH;
int buttonPlay = HIGH;

int buttonRecordLast = LOW;
int buttonPlayLast = LOW;

bool record = false;
bool play = false;
bool debug = false;

String command = "Manual";
int printPos = 0;

int buttonPlayDelay = 20;
int buttonPlayCount = 0;

bool ledLight = false;

Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoGripper;
Servo myservo;

void setup() {
  Serial.begin(9600);

  //
          //  setup serial
  myservo.attach(realGripperPin);  // attaches the servo on pin 3 to the servo object
  pinMode(gdirPin,INPUT_PULLUP);
  Serial.println("Robojax Servo Button ");
  myservo.write(angle);//initial position

  //
  
  pinMode(xdirPin, INPUT);
  pinMode(ydirPin, INPUT);
  pinMode(zdirPin, INPUT);
  pinMode(gdirPin, INPUT);

  pinMode(pinRecord, INPUT_PULLUP);
  pinMode(pinPlay, INPUT_PULLUP);

  pinMode(pinLedRecord, OUTPUT);

  servoBase.attach(basePin);
  servoShoulder.attach(shoulderPin);
  servoElbow.attach(elbowPin);
  servoGripper.attach(gripperPin);

  StartPosition();

  digitalWrite(pinLedRecord, HIGH);
  delay(1000);
  digitalWrite(pinLedRecord, LOW);
}

void loop() {

  buttonRecord = digitalRead(pinRecord);
  buttonPlay = digitalRead(pinPlay);


  //////
  if(digitalRead(3) == LOW){
    buttonPushed = 1;
  }
  if(digitalRead(2) == LOW){
    buttonPushed2 = 1;
  }
   if( buttonPushed && angle < maxAngle ){
  // change the angle for next time through the loop:
  angle = angle + angleStep;


    
        if(type ==1)
        {
            buttonPushed =0;                   
        }
    
    
  
       if(type ==2)
        {
            buttonPushed =0;       
        }
    
    
    myservo.write(angle); // move the servo to desired angle
      Serial.print("Moved to: ");
      Serial.print(angle);   // print the angle
      Serial.println(" degree");    
  delay(100); // waits for the servo to get there
   }


 
  //////


/////////////////////7
 if( buttonPushed2 && angle > minAngle ){
  // change the angle for next time through the loop:
  angle = angle - angleStep;


    
        if(type ==1)
        {
            buttonPushed2 =0;                   
        }
    
    
  
       if(type ==2)
        {
            buttonPushed2 =0;       
        }
    
    
    myservo.write(angle); // move the servo to desired angle
      Serial.print("Moved to: ");
      Serial.print(angle);   // print the angle
      Serial.println(" degree");    
  delay(100); // waits for the servo to get there
   }


//////////////////



    

  //  Serial.print(buttonRecord);
  //  Serial.print("\t");
  //  Serial.println(buttonPlay);
  //  for testing purposes

//  if (buttonPlay == LOW)
//  {
//    buttonPlayCount++;
//
//    if (buttonPlayCount >= buttonPlayDelay)
//    {
//      repeatePlaying = true;
//    }
//  }
//  else buttonPlayCount = 0;

//  if (buttonPlay != buttonPlayLast)
//  {
//    if (record)
//    {
//      record = false;
//    }
//
//    if (buttonPlay == LOW)
//    {
//      play = !play;
//      repeatePlaying = false;
//
//     // if (play)
//    //  {
//   //     StartPosition();
//   //   }
//    }
//  }
//
//  if (buttonRecord != buttonRecordLast)
//  {
//    if (buttonRecord == LOW)
//    {
//      record = !record;
//
//      if (record)
//      {
//        play = false;
//        repeatePlaying = false;
//        recPos = 0;
//      }
//      else
//      {
//        if (debug) PrintBuffer();
//      }
//    }
//  }
//
//  buttonPlayLast = buttonPlay;
//  buttonRecordLast = buttonRecord;

  float dx = map(analogRead(xdirPin), 0, 1023, -5.0, 5.0);
  float dy = map(analogRead(ydirPin), 0, 1023, 5.0, -5.0);
  float dz = map(analogRead(zdirPin), 0, 1023, 5.0, -5.0);
  float dg = map(analogRead(gdirPin), 0, 1023, 5.0, -5.0);

  if (abs(dx) < 1.5) dx = 0;
  if (abs(dy) < 1.5) dy = 0;
  if (abs(dz) < 1.5) dz = 0;
  if (abs(dg) < 1.5) dg = 0;

  posBase += dx;
  posShoulder += dy;
  posElbow += dz;
  posGripper += dg;

  if (play)
  {
    if (playPos >= recPos) {
      playPos = 0;

      if (repeatePlaying)
      {
        delay(delayBetweenCycles);
        StartPosition();
      }
      else
      {
        play = false;
      }
    }

    bool endOfData = false;

    while (!endOfData)
    {
      if (playPos >= buffSize - 1) break;
      if (playPos >= recPos) break;

      int data = buff[playPos];
      int angle = data & 0xFFF;
      int servoNumber = data & 0x3000;
      endOfData = data & 0x4000;

      switch (servoNumber)
      {
        case 0x0000:
          posBase = angle;
          break;

        case 0x1000:
          posShoulder = angle;
          break;

        case 0x2000:
          posElbow = angle;
          break;

        case 0x3000:
          posGripper = angle;
          break;
      }

      playPos++;
    }
  }

  if (posBase > maxBase) posBase = maxBase;
  if (posShoulder > maxShoulder) posShoulder = maxShoulder;
  if (posElbow > maxElbow) posElbow = maxElbow;
  if (posGripper > maxGripper) posGripper = maxGripper;

  if (posBase < minBase) posBase = minBase;
  if (posShoulder < minShoulder) posShoulder = minShoulder;
  if (posElbow < minElbow) posElbow = minElbow;
  if (posGripper < minGripper) posGripper = minGripper;

  servoBase.write(posBase);
  servoShoulder.write(posShoulder);
  servoElbow.write(posElbow);

  //  if (dg < -3.0) {
  //    posGripper = minGripper;
  //    servoGripper.write(posGripper);
  //    Serial.println(posGripper);
  //  }
  //  else if (dg > 3.0) {
  //    posGripper = maxGripper;
  //    servoGripper.write(posGripper);
  //    Serial.println(posGripper);
  //  }


  servoGripper.write(posGripper);
 

  //Serial.println(posGripper);

  if ((lastBase != posBase) | (lastShoulder != posShoulder) | (lastElbow != posElbow) | (lastGripper != posGripper))
  {
    if (record)
    {
      if (recPos < buffSize - countServo)
      {
        int buffPos = 0;

        if (lastBase != posBase)
        {
          buffAdd[buffPos] = posBase;
          buffPos++;
        }

        if (lastShoulder != posShoulder)
        {
          buffAdd[buffPos] = posShoulder | 0x1000;
          buffPos++;
        }

        if (lastElbow != posElbow)
        {
          buffAdd[buffPos] = posElbow | 0x2000;
          buffPos++;
        }

        if (lastGripper != posGripper)
        {
          buffAdd[buffPos] = posGripper | 0x3000;
          buffPos++;
        }

        buffAdd[buffPos - 1] = buffAdd[buffPos - 1] | 0x4000;

        for (int i = 0; i < buffPos; i++)
        {
          buff[recPos + i] = buffAdd[i];
        }

        recPos += buffPos;
      }
    }

    command = "Manual";
    printPos = 0;

    if (play)
    {
      command = "Play";
      printPos = playPos;
    }
    else if (record)
    {
      command = "Record";
      printPos = recPos;
    }

    Serial.print(command);
    Serial.print("\t");
    Serial.print(printPos);
    Serial.print("\t");
    Serial.print(posBase);
    Serial.print("\t");
    Serial.print(posShoulder);
    Serial.print("\t");
    Serial.print(posElbow);
    Serial.print("\t");
    Serial.print(posGripper);
    Serial.print("\t");
    Serial.print(record);
    Serial.print("\t");
    Serial.print(play);
    Serial.println();
  }

  lastBase = posBase;
  lastShoulder = posShoulder;
  lastElbow = posElbow;
  lastGripper = posGripper;

  if ( repeatePlaying)
  {
    ledLight = !ledLight;
  }
  else
  {
    if (ledLight)
    {
      ledLight = false;
    }

    if (record)
    {
      ledLight = true;
    }
  };

  digitalWrite(pinLedRecord, ledLight);
  delay(50);
}

void PrintBuffer()
{
  for (int i = 0; i < recPos; i++)
  {
    int data = buff[i];
    int angle = data & 0xFFF;
    int servoNumber = data & 0x3000;
    bool endOfData = data & 0x4000;

    Serial.print("Servo=");
    Serial.print(servoNumber);
    Serial.print("\tAngle=");
    Serial.print(angle);
    Serial.print("\tEnd=");
    Serial.print(endOfData);
    Serial.print("\tData=");
    Serial.print(data, BIN);
    Serial.println();
  }
}

void StartPosition()
{
  int angleBase = servoBase.read();
  int angleShoulder = servoShoulder.read();
  int angleElbow = servoElbow.read();
  int angleGripper = servoGripper.read();

  Serial.print(angleBase);
  Serial.print("\t");
  Serial.print(angleShoulder);
  Serial.print("\t");
  Serial.print(angleElbow);
  Serial.print("\t");
  Serial.print(angleGripper);
  Serial.println("\t");

  posBase = startBase;
  posShoulder = startShoulder;
  posElbow = startElbow;
  posGripper = startGripper;

  servoBase.write(posBase);
  servoShoulder.write(posShoulder);
  servoElbow.write(posElbow);
  servoGripper.write(posGripper);
}
