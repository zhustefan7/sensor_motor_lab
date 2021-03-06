void DC_vel_control(bool vcmd){
  // sensor = 0, Sensor controls velocity
  // 1. Get encoder readings: interrupt

  // 2. Stopwatch 
  double dt = (double)(micros() - timestamp) / 1000000;
  timestamp = micros();

  // 3. PID settings
  if (vcmd == false){
    sensor_reading = analogRead(POT); // 8~400
    grad = sensor_reading / pot_gain;
  }
  
  //Serial.println(grad);
  Set += grad;
  Feed = count;

  // 4. Compute PID
  Kp = 0.2, Ki = 0, Kd = 0.01; //Kp 0.12
  PWMmax = 150;
  Motor_PID.SetOutputLimits(-PWMmax, PWMmax);
  Motor_PID.SetTunings(Kp, Ki, Kd);
  Motor_PID.Compute();
  PWMdrive(PWMvalue, 20); //20

  if (vcmd == true){
    motor_reading = int(angle);
  }
  else{
    motor_reading = int(PWMvalue);
  }
  

  //Serial.print("Velocity: \t");
  //Serial.println(motor_reading);
}

void DC_pos_control(){
  // sensor = 1, give position target manually
  // 1. Get encoder readings: interrupt

  // 2. Stopwatch 
  double dt = (double)(micros() - timestamp) / 1000000;
  timestamp = micros();

  // 3. PID settings
  Feed = count;

  // 4. Compute PID
  Kp = 0.15, Ki = 0.01, Kd = 0;
  PWMmax = 10;
  Motor_PID.SetOutputLimits(-PWMmax, PWMmax);
  Motor_PID.SetTunings(Kp, Ki, Kd);
  Motor_PID.Compute();
  PWMdrive(PWMvalue, 50);
  motor_reading = count * (resolution / 360.0);
  //Serial.print("Position (degrees): \t");
  //Serial.println(motor_reading); //grad
}


void doEncoder() {
 n = digitalRead(encoder0PinA);
  if ((encoder0PinALast == LOW) && (n == HIGH)) {
    if (digitalRead(encoder0PinB) == LOW) {
      count--;
    } else {
      count++;
    }
  }
  encoder0PinALast = n;
}

void PID_init(){
  Motor_PID.SetSampleTime(1000 / Timer3_HZ);
  Motor_PID.SetMode(MANUAL);
  Motor_PID.SetMode(AUTOMATIC);
}

void PWMdrive(double val, int shift){
  if (val >= 0){
    val += shift;
    analogWrite(CW, val);
    analogWrite(CCW, 0);
  }
  else{
    val -= shift;
    analogWrite(CCW, -val);
    analogWrite(CW, 0);
  }
}

void PWMtest(){
    for (int i = 0; i <= 255; i++){
    analogWrite(CW, i);
    analogWrite(CCW, 0);
    delay(10);
  }
  for (int i = 255; i >= 0; i--){
    analogWrite(CW, i);
    analogWrite(CCW, 0);
    delay(10);
  }
 for (int i = 0; i <= 255; i++){
    analogWrite(CCW, i);
    analogWrite(CW, 0);
    delay(10);
  }
  for (int i = 255; i >= 0; i--){
    analogWrite(CCW, i);
    analogWrite(CW, 0);
    delay(10);
  }
}

