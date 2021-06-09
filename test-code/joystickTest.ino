/*
 * Cassidy Correll
 * JoystickTest
 * May 5, 2021
 * 
 * Tests the joystick
 * prints the cordinates of the joystick to serial monitor
 * When at center values will be around 780
 * 
 * X direction: All the way left is 0, all the way right is 1023
 * Y direction: All the way down is 0, all the way up is 1023
 */

 // Joystick Setup
#define joyX A0
#define joyY A1
#define joySelect 4

void setup() {
  Serial.begin(115200);

}

void loop() {
  //x axis of joystick
  Serial.print("\nX: ");
  Serial.print(analogRead(joyX));

  //y axis of joystick
  Serial.print("\nY: ");
  Serial.print(analogRead(joyY));


  Serial.print("\n\n");

  delay(500);

}
