#include <Arduino.h>
#include <Servo.h>

#define echoPin 2 //pin D2 
#define trigPin 3 //pin D3
#define servoPin 9

Servo s1;
long duration;
int distance;
int closed;
int pos;

void setup() {
  // put your setup code here, to run once:
  s1.attach(servoPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  closed = 0;
  Serial.begin(9600);
  Serial.println("Setting up!");
  openGate();
}

void loop() {
  distance = readDistance(); // read distance from sensor
  if (!closed) {
    printDistance(distance);
  }

  if (!closed && distance < 20 && distance > 0) {
    Serial.print("Object distance is ");
    Serial.print(distance);
    Serial.println(". Closing gate!");
    closeGate();
    closed = 1;
    Serial.println("Gate is closed!");
  }
  
  delay(100);
}

// HELPER FUNCTIONS
void openGate() {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    s1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void closeGate() {
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    s1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

int readDistance() {
  int d;
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(200);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(1000);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  d = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  return d;
}
void printDistance(int d) {
  Serial.print("Distance: ");
  Serial.print(d);
  Serial.println(" cm");
}
