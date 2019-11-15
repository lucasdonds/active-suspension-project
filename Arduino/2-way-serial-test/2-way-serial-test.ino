
// to interface with Matlab: 'two_way_connection_test.m'


int y = 1;
int x = 1; 

void setup() {
  Serial.begin(115200);
  Serial.println('a');
  char a = 'b';
  while (a != 'a')
  {
    a = Serial.read();
  }
}

void loop() {
  Serial.println(1);
  Serial.println(2);
  Serial.println(3);
  Serial.println(4);
  Serial.println(5);
  Serial.println(6);
  Serial.println(7);
  Serial.println(8);
  Serial.println(9);
  Serial.println(10);
  Serial.println(11);
  Serial.println(12);
  Serial.println(13);
  Serial.println(14);
  Serial.println(15);
  Serial.println(16);
  Serial.println(17);
  Serial.println(18);
  Serial.println(19);
  Serial.println(20);
  Serial.println(21);
  Serial.println(22);
  Serial.println(23);
  Serial.println(24);
  Serial.println(25);

  while (!Serial.available()) {}
  x = Serial.read();
  x++;
  Serial.println(x);
  
}
