
#include <Arduino.h>
#include <Servo.h>

#define servoPin2 7
Servo s2;
int pos = 0; 
int a = 120;

#define echoPin 2 //pin D2 
#define trigPin 3 //pin D3

#define echoPin2 7 //pin D2 
#define trigPin2 6 //pin D3

float duration_us, distance, distance2;
bool closed = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  s2.attach(servoPin2);
  // Initializing both motor positions
  for (pos = 0; pos <= a; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    s2.write(a-pos); //+ 90); 
    delay(30);                       // waits 15ms for the servo to reach the position
  }
  delay(1000); 
//  for (pos = a; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    s2.write(a-pos); 
//    delay(30);                       // waits 15ms for the servo to reach the position
//  }
}

void closeGate(){
  for (pos = a; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    s2.write(a-pos); 
    delay(15);                       // waits 15ms for the servo to reach the position
  }
///
}

int readDistance() {
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(echoPin, HIGH);

  // calculate the distance0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  return 0.017 * duration_us;
}

void loop() {
  // put your main code here, to run repeatedly:
  distance = readDistance(); // read distance from sensor
  Serial.println(distance);

  // If it is close enough, send a 1
  if (closed == false){
    if (distance > 10 and distance <= 20){
      Serial.println(2);
    }
    else if (distance <= 10){ 
      // Before sending the message, close the cage. 
      // Closing the cage
      closeGate();
      closed = true;
      Serial.println(1);
    }
  }
  delay(50);
}
