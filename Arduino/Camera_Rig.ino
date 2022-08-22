#include <AFMotor.h>
String command;
int inputval = 0;
int intervals = 0;
AF_Stepper motor(200, 2);
void setup() {
  Serial.begin(9600);
  motor.setSpeed(20);
  delay(1000);
}
void loop() {
  //Checks for Serial input
  if(Serial.available()){
    command = Serial.readStringUntil("\n");
    inputval = command.toInt();
    if(0 == inputval){
      //0 makes it run bcak to home
      motor.step(1100, BACKWARD, DOUBLE);
      motor.release();
    }
    else if(command.toInt() > 20) {
      //Prevent Python Bug
      Serial.println("Try Something Smaller");
    }
    else {
    // Runs motor for the inputted number of intervals
    intervals = 1100/abs(command.toInt());
      if(intervals < 1101 && intervals > 0){
     for(int i = 1; i <= command.toInt(); i++){
      motor.step(intervals, FORWARD, DOUBLE);
      delay(500);
      Serial.println("Photo");
      delay(3000);
    }
    // Returns back to zero
    motor.step(1100, BACKWARD, DOUBLE); 
    }
    else {
      Serial.println("Try Something in the range of 0-20");
    }
    motor.release();
    }
  }

}
