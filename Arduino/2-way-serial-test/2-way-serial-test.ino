int y = 1;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Start Serial Communication");
}

void loop() {
  for (int index = 0; index<25; index++) {
  Serial.println(y);
  y++;
  }
  int x = Serial.read();
  x++;
  Serial.println(x);
}
