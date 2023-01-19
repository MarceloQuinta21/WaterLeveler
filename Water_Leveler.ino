#include <Servo.h>

Servo myservo;    // create servo object to control a servo named 'myservo'

char printbuffer[128];
double DC = 0;

/* PID */
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double percentError;
double lastError;
double input, output, setPoint;
double cumError, rateError;

// PID Constants
double kp = 1;
double ki = 1;

void setup(){
  setPoint = 700;   // Desired ADC Value
  
  Serial.begin(9600);   // Setup Serial

  // Pump Setup
  pinMode(A0, OUTPUT);    // R_EN
  pinMode(A1, OUTPUT);    // L_EN
  pinMode(A2, OUTPUT);    // RPWM
  pinMode(A3, OUTPUT);    // LPWM
  digitalWrite(A0, HIGH);
  digitalWrite(A1, HIGH);
  digitalWrite(A3, LOW);

  // Servo and Button Setup
  myservo.attach(4);  // attaches the servo on pin D4 to the servo object
  myservo.write(90);  //  90 tells the CRS to stop

  pinMode(2,INPUT);   // attaches button to pin D2
}    

void loop(){
  /* Main Code */
  double value = analogRead(4);  // Read ADC Value from Water Sensor
  DC = computePID(value);   // Grab PID Value

  // Disturbance
  if(digitalRead(2)==HIGH){
    myservo.write(180);   // 180 tells the continuous rotation servo (CRS) to move forward
    delay(150);    // wait for the servo to rech original position
    myservo.write(90);    // 90 tells the CRS to stop

    delay(1000); // wait before moving servo back

    myservo.write(80);     // 0 tells the continuous rotation servo (CRS) to move backward
    delay(400);    // wait for the servo to reach desired position
    myservo.write(90);    // 90 tells the CRS to stop  
  }
  else{
  }

  // Correct Duty Cycle Value
  if (DC > 100){
    DC = 100;
  }
  else if (DC < 0)
    DC = 0;
  else{
    // Do Nothing
  }
  pump(DC);

  // /*Print Values*/
  // //Print Water Sensor ADC Value
  // Serial.print("ADC: ");
  // Serial.print(value);
  // Serial.print(" - ");

  // // Print Duty Cycle
  // Serial.print("Duty Cycle: ");
  // Serial.print(DC);
  // Serial.print("%");
  
  // Serial.println();   // Print Values in One Line
  }

void pump(double duty_cycle){
  digitalWrite(A3, HIGH);
  delayMicroseconds(duty_cycle*10);
  digitalWrite(A3, LOW);
  delayMicroseconds(1000 - duty_cycle*10);
}

double computePID(double input){     
  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
  error = setPoint - input;                                // determine error
  double out = kp*error;                //PID output               
 
  lastError = error;                                //remember current error
  previousTime = currentTime;                        //remember current time
 
  return out;                                        //have function return the PID output
}