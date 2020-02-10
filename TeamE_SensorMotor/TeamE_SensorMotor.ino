#include <Servo.h>
#include <PID_v1.h>
Servo myservo;  // create servo object to control a servo

// Pin Definitions
const int IRpin = 1;                 // IR goes to Board A1
const int IRbuttonPin = 2;          // IR button goes to Digital 2
const int servoPin = 10;             // Servo pin goes to Digital 10
const int stepperPinA = 8;
const int stepperPinB = 9;
const int ultraPin = 0;

#define POT A1
#define encoder0PinA  3
#define encoder0PinB  4
#define CW 5
#define CCW 6

// State Changing Variables
int IRbuttonVal = 0;
const int numStates = 4;
int state = 0;
bool IRcanChange = true;
int angle;
int control = 0;
int prev_motor = 0;
int motor = 0;
int sensor_reading;
bool newAngle = false;

// IR Sensor Variables
int val = 0;                 // variable to store the values from sensor(initially zero)
int pos = 0;                 // variable to store the servo position
float IRhigh = 4;
float IRlow = .5;

// Stepper Motor Variables
String readString;
int StepCounter = 0;
int Stepping = false;

// Encoder variables
int encoder0PinALast = LOW;
int n = LOW;

int resolution = 322;
int count = 0;

// Pot variables
float pot_gain = 100;
float grad;

// PID variables
long int timestamp = 0;
double Timer3_HZ = 100.0;
int PWMmax;

// PID Settings
double Feed = 0;
double Set = 0;
double PWMvalue = 0;  //Computed PWM requirement
double Kp, Ki, Kd;
PID Motor_PID(&Feed, &PWMvalue, &Set, 0, 0, 0, DIRECT);


void setup()
{
  Serial.begin(9600);               // starts the serial monitor
  myservo.attach(servoPin);  // attaches the servo on pin 10 to the servo object
  pinMode(IRbuttonPin, INPUT);
  pinMode (POT, INPUT);
  pinMode(CCW, OUTPUT);
  pinMode(CW, OUTPUT);

  pinMode(stepperPinA, OUTPUT);
  pinMode(stepperPinB, OUTPUT);
  digitalWrite(stepperPinB, LOW);
  digitalWrite(stepperPinA, LOW);

  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);
  attachInterrupt(0, doEncoder, CHANGE);

  PID_init();
  timestamp = micros();

  analogWrite(CCW, 0); // 0 is stop, 255 is max speed
  analogWrite(CW, 0);
}

void loop()
{
//  Serial.println("control:");/
//  Serial.print("motor:"); 
//  Serial.println(motor);
//  Serial.print("control:");
//  Serial.println(control);
//  Serial.println("angle:");
//  Serial.print(angle);


  parseInput();

  if(motor == 1) // Servo Motor Functions
  {
    if (control == 0){
      report_state();
      servoIR();   //Servo Sensor Control
      } 
    else if (control == 1 && newAngle) // Servo Serial Control
    {
//      Serial.println(newAngle);/
      servoSerial();
      newAngle = false;
    }
  }
  
  else if(motor == 2) // Stepper Motor Functions
  {
    if (control == 0) {
      servoIR();   
      ultrastepper();
      } // Stepper Sensor Control
    else if (control == 1 && newAngle)  // Stepper Serial Control
    {
      serialStepper();
      newAngle = false;
    }
  }

  else if(motor == 3) // DC Brushless Motor Functions
  {
    if (control == 0) {
      Serial.println("Brushless Sensor Func"); // Brushless Sensor Control
      DC_vel_control();
    
    }
    else if (control == 1 && newAngle) // Brushless Serial Control
    {
      Serial.println("Brushless Sensor Func");
      newAngle = false;
      DC_pos_control();
    }

  }
}

  ////////////////////////////////////////////////////////////////
  /////////////////// Ultramotor Functions ///////////////////////
  void ultrastepper()
  {
    int Ultrasensor, Ultrainches;

    // read the analog output of the EZ1 from analog input 0
    Ultrasensor = analogRead(ultraPin);
    Ultrainches = Ultrasensor / 2;
    sensor_reading = Ultrainches; 
    //Serial.println(inches,DEC);
    delay(1);
    digitalWrite(stepperPinB, HIGH);
    delay(Ultrainches / 10);
    digitalWrite(stepperPinB, LOW);
    delay(Ultrainches / 10);
  }

void serialStepper()
{
  int n = angle;
  int m;
  if (n >= -360 && n <= 360)
  {
        //Serial.println(n);
    if (n >= 0 && n <= 360)
    { //forward
      m = n * 4.444;
      Serial.println(m);
      digitalWrite(stepperPinA, LOW);
      Stepping = true;
    }
    else
    { //reverse
      m = -n * 4.444;
      Serial.println(m);
      digitalWrite(stepperPinA, HIGH);
      Stepping = true;
    }
  }
  while (Stepping == true)
  {
    digitalWrite(stepperPinB, HIGH);
    delay(1);
    digitalWrite(stepperPinB, LOW);
    delay(1);
    StepCounter = StepCounter + 1;
  
    if (StepCounter == m)
    {
      StepCounter = 0;
      Stepping = false;
    }
  }
}

  ////////////////////////////////////////////////////////////////
  /////////////////// Get IR/Servo Functions//////////////////////
  void servoSerial()
  { 
    myservo.write(angle);
    newAngle = false;
  }

  void servoIR()
  {
    int IRdist =  averageFilter();
    int new_pos = setServo(IRdist);
    sensor_reading = IRdist;
    myservo.write(new_pos);
  }


  float getIRdist()
  {
    int IRdist = analogRead(IRpin);       // reads the value of the sharp sensor
    //  Serial.println(val);
    IRdist = (IRdist - 239.46) / 65.961;
    if (IRdist < -1) IRdist = 5;
    if (IRdist > IRhigh) IRdist = IRhigh;
    if (IRdist < IRlow) IRdist = IRlow;

    return IRdist;
  }


  float averageFilter()
  {
    float distance_filtered = 0;
    int num_points = 200;
    for (int i = 0; i < num_points; ++i)
    {
      distance_filtered += getIRdist();
    }
    delay(50);
    distance_filtered /= (num_points);

    return distance_filtered;
  }

  int setServo(int IRdist)
  {
    float servLow = 0;
    float servHigh = 180;
    float pos2 = servLow + (float(IRdist) - IRlow) * (servHigh - servLow) / (IRhigh - IRlow);
    return  int(pos2);
  }

  ////////////////////////////////////////////////////////////////
  /////////////////// State  Related functions ///////////////////
  void getIRbuttonInfo()
  {
    IRbuttonVal = digitalRead(IRbuttonPin);
    if (IRbuttonVal == HIGH && IRcanChange) {
      state = buttonPress(state);
      IRcanChange = false;
      Serial.print("Entering State: ");
      Serial.println(state % numStates);
    }
    if (IRbuttonVal == LOW) IRcanChange = true;
    state = state % numStates;
  }

  int buttonPress(int val)
  {
    delay(100);
    return val + 1;
  }


void parseInput()
{
  String serialIn = "";
  // Get Input From Keyboard
  if (Serial.available() > 0) serialIn = Serial.readString();
  
  // Check if full input is received
  if (serialIn.length() == 4)
  {
    // Extract first char from input
    //Serial.println(serialIn);
  char inChar = serialIn.charAt(0);
  serialIn.remove(0,1);
    
    // Check if remaining is a number
    if(serialIn.toInt()|| serialIn=="000")
    {
      // Convert String
      int newVal = serialIn.toInt();
      
      // Update State
      if(inChar == 'm'){
        prev_motor = motor;
        motor = newVal;
        if (newVal !=  prev_motor){
          control = 0;
        }
       }
       
      else if(inChar == 'c') control = newVal;
      else if(inChar == 'a') 
      {
        newAngle = true;
        angle = newVal;
      }
   }
  }
}




void report_state(){
  Serial.print('s');
  Serial.println(sensor_reading);
}
