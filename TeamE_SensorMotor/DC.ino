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

void PID_init(int control){
  if (control == 0){ //sensor
    Kp = 0.05, Ki = 0, Kd = 0.01;
  }
  else if (control == 1){
    Kp = 0.15, Ki = 0.01, Kd = 0;
  }

  Motor_PID.SetTunings(Kp, Ki, Kd);
  Motor_PID.SetSampleTime(1000 / Timer3_HZ);
  Motor_PID.SetOutputLimits(-PWMmax, PWMmax);
  Motor_PID.SetMode(MANUAL);
  Motor_PID.SetMode(AUTOMATIC);
}

void PWMdrive(double val){
  if (val >= 0){
    val += 40;
    analogWrite(CW, val);
    analogWrite(CCW, 0);
  }
  else{
    val -= 40;
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

