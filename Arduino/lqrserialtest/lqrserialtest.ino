//Interface with matlab: lqrrecievetest.m
// Lucas Dondertman Nov 7th 2019
int LEDPin = 8;

double ledPWM = 0;
double otherCost = 0;
int readInt = 0;
double gainMatrix [2][2];


void serialHandshake()
{
  //handshake with matlab to start serial connection
  Serial.println('a');
  char a = 'b';
  while (a != 'a')
  {
    a = Serial.read();
  }
}

void setup() {
  
  //if using serial plotter/monitor ensure that baud rate is the same as this one
  Serial.begin(115200);

  serialHandshake();

  pinMode(LEDPin, OUTPUT);
  digitalWrite(LEDPin, HIGH);
}

void loop() {
  
  int gainRows = 2;
  int gainCols = 2; 

  for (int i = 0; i < gainRows; i++) {
    for (int j = 0; j < gainCols; j++) {
        while (!Serial.available()) {}
        gainMatrix[i][j] = Serial.parseInt();
    }
  }

  for (int i = 0; i < gainRows; i++) {
    for (int j = 0; j < gainCols; j++) {
      Serial.println(gainMatrix[i][j]);
      delay(10);
    }
  }
  
  ledPWM = gainMatrix[0][0];
  otherCost = gainMatrix[1][1];
  
  analogWrite(LEDPin, ledPWM);
  delay(3000);
}
