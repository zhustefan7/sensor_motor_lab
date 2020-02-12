#include <Servo.h>
#include <PID_v1.h>
Servo myservo;  // create servo object to control a servo

// Pin Definitions
const int IRpin = 1;                 // IR goes to Board A1
const int IRbuttonPin = 11;          // IR button goes to Digital 2
const int servoPin = 10;             // Servo pin goes to Digital 10
const int stepperPinA = 8;
const int stepperPinB = 9;
const int ultraPin = 0;

#define POT A2
#define encoder0PinA  2
#define encoder0PinB  3
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
int motor_reading;
bool newAngle = false;
int last_control = 0;

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

int resolution = 360; //322;
int count = 0;

// Pot variables
float pot_gain = 25;
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

// Timer Stuff
int motor_timer = 0;
int sensor_timer = 0;
int timer_threshold = 500;


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
//  Serial.print("angle:");
//  Serial.println(angle);

motor_timer += 1;
sensor_timer += 1;


  parseInput();

  if(motor == 1) // Servo Motor Functions
  {
    if (control == 0){
      servoIR();   //Servo Sensor Control
      report_state();
      } 
    else if (control == 1 && newAngle) // Servo Serial Control
    {
//      Serial.println(newAngle);/
      servoSerial();
      newAngle = false;
    }
    report_motor();
  }
  
  else if(motor == 2) // Stepper Motor Functions
  {
    if (control == 0) { 
      ultrastepper();
      report_state();
      } // Stepper Sensor Control
    else if (control == 1 && newAngle)  // Stepper Serial Control
    {
      serialStepper();
      newAngle = false;
    }
    report_motor();
  }

  else if(motor == 3) // DC Brushless Motor Functions
  {
    if (control != last_control){ // control mode transition
      for (int i = 0; i < 10; i++){
        PWMdrive(0, 0);
        //Serial.println("DC Reset");
      }
      Set = 0;
      count = 0;
      motor_reading = 0;
    }
    
    if (control == 0) {
      //Serial.println("Brushless Sensor Func"); // Brushless Sensor Control
      DC_vel_control();
      report_state();
    
    }
    else if (control == 1 && newAngle) // Brushless Serial Control
    {
      //Serial.println("Brushless Sensor Func");
      newAngle = false;
      Set = angle * (360.0 / resolution);
      while(count <= Set){
        DC_pos_control();
      }
      
      PWMdrive(0, 0); //stop motor after reaching target position
      delay(10);     //wait until motor stops
      Set = 0;    
      count = 0;      //reset target and encoder count
    }
    last_control = control;
    
  report_motor();
  }
}

  ////////////////////////////////////////////////////////////////
  /////////////////// Ultramotor Functions ///////////////////////
  void ultrastepper()
  {
    int Ultrasensor, Ultrainches;
    float scale = 2;
    float bias = 0;

    // read the analog output of the EZ1 from analog input 0
    Ultrasensor = analogRead(ultraPin);
//    Serial.println(Ultrasensor,DEC);
    Ultrainches = (Ultrasensor + bias) / scale;
    sensor_reading = Ultrainches; 
    //Serial.println(inches,DEC);
    int delayval = Ultrainches / 10;
    motor_reading = delayval;
    delay(1);
    digitalWrite(stepperPinB, HIGH);
    delay(delayval);
    digitalWrite(stepperPinB, LOW);
    delay(delayval);
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
//      Serial.println(m);
      digitalWrite(stepperPinA, LOW);
      Stepping = true;
    }
    else
    { //reverse
      m = -n * 4.444;
//      Serial.println(m);
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
    motor_reading = StepCounter;
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
    motor_reading = angle;
  }

  void servoIR()
  {
    float IRdist =  averageFilter();
    sensor_reading = IRdist;
    int new_pos = setServo(int(IRdist));
    motor_reading = new_pos;
    myservo.write(new_pos);
  }


  float getIRdist()
  {
    int IRdist = analogRead(IRpin);       // reads the value of the sharp sensor
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
    distance_filtered /= float(num_points);
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
  if (serialIn.length() == 6)
  {
    // Extract first char from input
  char inChar = serialIn.charAt(0);
  serialIn.remove(0,1);

  char inChar1 = serialIn.charAt(0);
  serialIn.remove(0,1);
  
    // Check if remaining is a number
    if(serialIn.toInt() || (serialIn[0]=='0'&& serialIn[2]=='0'&& serialIn[1]=='0'))
    {
      // Convert String
      int newVal = serialIn.toInt();
      
      // Update State
      if(inChar == 'm' && inChar1 =='m'){
        prev_motor = motor;
        motor = newVal;
        if (newVal !=  prev_motor){
          control = 0;
        }
       }
       
      else if(inChar == 'c'&& inChar1 =='c') control = newVal;
      else if(inChar == 'a'&& inChar1 =='a') 
      {
        newAngle = true;
        angle = newVal;
      }
      else if(inChar == 'a'&& inChar1 =='v') 
      {
        control = 2;
        angle = newVal;
      }
   }
  }
}

void report_motor(){
  if(motor_timer > timer_threshold)
  {
    Serial.print('z');
    Serial.println(motor_reading);
    motor_timer = 0;
  }
}


void report_state(){
  if(sensor_timer > timer_threshold)
  {
    Serial.print('s');
    Serial.println(sensor_reading);
    sensor_timer = 0;
  }
}
