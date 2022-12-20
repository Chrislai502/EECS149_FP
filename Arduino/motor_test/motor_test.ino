#include <Arduino.h>
#include <Servo.h>

//#define servoPin1 9
#define servoPin2 7

//Servo s1;
Servo s2;
int pos = 0; 
int a = 120;

void setup() {
  // put your setup code here, to run once:
//  s1.attach(servoPin1);
  s2.attach(servoPin2);

  // Initializing both motor positions
  for (pos = 0; pos <= a; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    s2.write(a-pos); //+ 90); 
//    s1.write(pos); //+ 90);              // tell servo to go to position in variable 'pos'
    delay(30);                       // waits 15ms for the servo to reach the position
  }
//  delay(1000);                       // waits 15ms for the servo to reach the position
//  for (pos = a; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    s2.write(a-pos + 90); 
//    s1.write(pos + 90);              // tell servo to go to position in variable 'pos'
//    delay(30);                       // waits 15ms for the servo to reach the position
//  }
}

void loop() {
//   put your main  code here, to run repeatedly:
//  int a= 90;
  for (pos = 0; pos <= a; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    s2.write(a-pos); 
//    s1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  delay(1000);
  for (pos = a; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    s2.write(a-pos); 
//    s1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
