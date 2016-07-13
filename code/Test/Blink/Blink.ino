/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://www.arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
 */


// the setup function runs once when you press reset or power the board
void setup() {
  // Setup the LED Enable
  pinMode(A4, OUTPUT);
  digitalWrite(A4, HIGH);
  
  // initialize pins for output
  pinMode(A0, OUTPUT); // Source 2 Enable
  pinMode(A1, OUTPUT); // Source 1 Enable
  pinMode(A2, OUTPUT); // Notification LED 2
  pinMode(A3, OUTPUT); // Notification LED 1
  pinMode(A5, OUTPUT); // PA Enable
  pinMode(0, OUTPUT);  // RX In
  pinMode(1, OUTPUT);  // TX Out
  pinMode(5, OUTPUT);  // CS
  pinMode(6, OUTPUT);  // Pi Power
  pinMode(7, OUTPUT);  // Onboard Audio Power
  pinMode(11, OUTPUT); // MOSI
  pinMode(12, OUTPUT); // MISO
  
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(A0, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(A1, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(A2, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(A3, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(A5, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(0, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(1, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(5, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(6, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(7, HIGH);
  digitalWrite(11, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(12, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(A0, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(A1, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(A2, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(A3, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(A5, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(0, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(1, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(5, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(6, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(7, LOW);
  digitalWrite(11, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(12, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second
}
