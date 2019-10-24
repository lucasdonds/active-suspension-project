int x = 2;


void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(x);
}

void loop() {
  if (Serial.available() > 0) {
    int y = Serial.read();
    y *= 2;
    Serial.println(y);
  }
    
}
