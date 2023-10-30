const int buttonPin = 2;

void setup() {
  pinMode(buttonPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // Read the state of the button (LOW when pressed, HIGH when not pressed)
  int buttonState = digitalRead(buttonPin);

  // Print the button state to the Serial Monitor
  Serial.print(buttonState);
  Serial.write(13);
  Serial.write(10);

  delay(100);  // Add a small delay to avoid rapid serial output
}