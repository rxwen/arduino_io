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
void WriteSerial(char* p)
{
  String stringOne = p;
  stringOne += "( ";
  stringOne += micros();
  stringOne += ", ";
  stringOne += ")\r\n";
  char charBuf[5000];
  stringOne.toCharArray(charBuf, 499);
  Serial.write(charBuf);
}

void WriteSerial2(char* p, unsigned long num1=NUMBER_NONE, unsigned long num2= NUMBER_NONE)
{
  String stringOne = p;
  stringOne += "( ";
  stringOne += micros();
  stringOne += ", ";
    stringOne += num1;
    stringOne += ", ";
    stringOne += num2;
    stringOne += ", ";
  stringOne += ")\r\n";
  char charBuf[500];
  stringOne.toCharArray(charBuf, 499);
  Serial.write(charBuf);
}

void WriteSerial3(char* p, long num1=NUMBER_NONE, long num2= NUMBER_NONE, long num3=NUMBER_NONE)
{
  String stringOne = p;
  stringOne += "( ";
  stringOne += micros();
  stringOne += ", ";
    stringOne += num1;
    stringOne += ", ";
    stringOne += num2;
    stringOne += ", ";
    stringOne += num3;  
    stringOne += ", ";
  stringOne += ")\r\n";
  char charBuf[500];
  stringOne.toCharArray(charBuf, 499);
  Serial.write(charBuf);
}


void ShowDiffs(long* diffs, int count){
       String str = "All Diffs:";
       for (int i=0;i<count;i++){
       	   str +=diffs[i];
	   str += ", ";
       }
       char charBuf[5000];
       str.toCharArray(charBuf, 4999);
       Serial.write(charBuf);
}
	
long write_check(int out_pin, int in_pin, int n){
     pinMode(out_pin, OUTPUT);
     pinMode(in_pin, INPUT);

     digitalWrite(out_pin, LOW);

     WriteSerial("start sleep");
     unsigned long start_one = micros();
     delay(1000);
     unsigned long end_one = micros();
     WriteSerial("end sleep");
     WriteSerial3("sleep one:", start_one, end_one, end_one-start_one);
     
     unsigned long t_start = micros();
     digitalWrite(out_pin, HIGH);
     while(digitalRead(in_pin) == LOW){
     }
     unsigned long t_end = micros();
     WriteSerial3("got low:", n, t_end, t_end-t_start);
     return t_end-t_start;
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

  const long N_COUNT = 100;
  long total_interval = 0;
  for (int i =0; i< N_COUNT;i++){
      total_interval += write_check(33, 39, i);
  }
  long avg_interval = total_interval/N_COUNT;
  long diffs[N_COUNT];

  for (int i=0; i<N_COUNT; i++){
      long t =  write_check(33, 39, N_COUNT+i);
      diffs[i] = t - avg_interval;
      WriteSerial3("diff: ", diffs[i], t, avg_interval);      
  }
  ShowDiffs(diffs, N_COUNT);
}
