/*
  Digital Output
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
  // initialize digital pin 8 as an output.
  pinMode(13, OUTPUT);
  Serial.begin(9600);
}


const int NUMBER_NONE = -9999999;
void WriteSerial(char* p, int num1=NUMBER_NONE, int num2= NUMBER_NONE, int num3=NUMBER_NONE, int num4 = NUMBER_NONE)
{
  String stringOne = p;
  stringOne += "( ";
  stringOne += millis();
  stringOne += ", ";
  if (num1 != NUMBER_NONE){
    stringOne += num1;
    stringOne += ", ";
  }
  if (num2 != NUMBER_NONE){
    stringOne += num2;
    stringOne += ", ";
  }
  if (num3 != NUMBER_NONE){
    stringOne += num3;
    stringOne += ", ";
  }
  if (num4 != NUMBER_NONE){
    stringOne += num4;
    stringOne += ", ";
  }
  stringOne += ")\r\n";
  char charBuf[500];
  stringOne.toCharArray(charBuf, 499);
  Serial.write(charBuf);
}

void write_check(int out_pin, int in_pin){
     pinMode(out_pin, OUTPUT);
     pinMode(in_pin, INPUT_PULLUP);

     digitalWrite(out_pint, HIGH);

     WriteSerial("start sleep");
     for (int i=0;i<999999; i++){
     i=i;
     }
     WriteSerial("end sleep");
     
     float t_start = millis();
     digitalWrite(out_pint, LOW);
     while(digitalRead(in_pin) == HIGH){
     }
     WriteSerial("got low:", t_start, millis());
}

int nTemp = 0;
// the loop function runs over and over again forever
void loop() {
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second
//  Serial.println(nTemp++);
  Serial.println("Hello");
  if (Serial.available()>0)
  {
	Serial.write(Serial.read());
  }

  write_check(33, 39);

}
