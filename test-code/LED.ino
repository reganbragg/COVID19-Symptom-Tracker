/*LED Test
 * Regan Bragg, Cassidy Correll, Charlotte Rogerson
 * May 5, 2021
 * 
 * Tests the LED by changing the light from blue to green to red
 */
 
int blue = 16;
int green = 15;
int red = 17;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // The LED will change between blue, green, and red
  analogWrite(blue, 255);
  analogWrite(green, 0);
  analogWrite(red, 0);

  delay(1000);

  analogWrite(blue, 0);
  analogWrite(green, 255);
  analogWrite(red, 0);

  delay(1000);

  analogWrite(blue, 0);
  analogWrite(green, 0);
  analogWrite(red, 255);

  delay(1000);

}
