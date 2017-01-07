#include <ArduinoJson.h>
#include "pt.h"
#include <QueueList.h>
#include "Timer.h"

#define SerialCom Serial1
#define SerialDbg Serial

const char VERSION[]="1.0.0";

const int NUM_THREAD = 3;
const int TIMEOUT_ACK = 300;

volatile static int flag_thread = 0;
static struct pt pt0, pt1, pt2;
volatile static boolean need_init0, need_init1, need_init2;
static Timer timer;

const int pin_led = 13;
volatile static boolean twinkling = true;

const int pin_online = 44;
const int pin_control = 28;
const int adpin_check = 1;
volatile static int status = 0; //0: no device, 1: turned on, 2: turned off
const int button_interval = 500;  //millisecond
volatile static float time_status_start;

const int NUMBER_NONE = -9999999;
void WriteSerialDebug(char* p, int num1=NUMBER_NONE, int num2= NUMBER_NONE, int num3=NUMBER_NONE, int num4 = NUMBER_NONE)
{
  String stringOne = p;
  stringOne += "( ";
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
  SerialDbg.write(charBuf);
}

void setup()
{
  SerialCom.begin(9600,SERIAL_8N1 );
  SerialDbg.begin(9600);

  pinMode(pin_online, INPUT_PULLUP);
  pinMode(pin_control, OUTPUT);
  digitalWrite(pin_control, HIGH);
  pinMode(pin_led, OUTPUT);
  timer.every(500, Twinkle);
  PT_INIT(&pt0);
  PT_INIT(&pt1);
  PT_INIT(&pt2);

  initiate();
  WriteSerialDebug("setup() is called .......");
}

static void initiate()
{
  status = 0;
  time_status_start = millis();
  need_init0 = true;
  need_init1 = true;
  need_init2 = true;
}

static int thread2_Reserved(struct pt *pt)
{
  PT_BEGIN(pt);

  while(true) {
    need_init2 = false;

    PT_WAIT_UNTIL(pt, flag_thread == 2);


    //do something here

    flag_thread = (flag_thread+1)%NUM_THREAD;
  }
  PT_END(pt);
}

static void click_control()
{
    digitalWrite(pin_control, LOW);
    delay(button_interval);
    digitalWrite(pin_control, HIGH);
}

static void set_status(int new_status)
{
    status = new_status;
    time_status_start = millis();
}

static int read_online()
{
    float time_start = millis();
    while((unsigned long)(millis() - time_start ) < 700)
    {
        if(digitalRead(pin_online)==HIGH){
            return HIGH;
        }
    }
    return LOW;
}

static int thread0_Worker(struct pt *pt)
{
  PT_BEGIN(pt);
  while(true) {
    need_init0 = false;
    PT_WAIT_UNTIL(pt, flag_thread == 0);


    if ( read_online() == HIGH && status!=0){
       digitalWrite(pin_control, HIGH);
       set_status(0);
    }

    int v_online, v_check;
    float time_now;
    switch(status){
    case 0:
         v_online = read_online();
         if ( v_online == LOW){
            click_control();
            set_status(1);
         }
         else
         {
            if (analogRead(adpin_check) > 200)
            {
                click_control(); //turn off the light if there is no device 
            }
         }
         break;
    case 1:
         time_now = millis();
         if ((unsigned long)(time_now - time_status_start) > 4000)
         {
             v_check = analogRead(adpin_check);

             //the avarage value of 3.3v is 675
             if (v_check >= 605 && v_check <= 745){
                click_control();
                set_status(2);
                delay(500);   //wait the device to take action
             } 
         }
         break;
    case 2:
         time_now = millis();
         if ((unsigned long)(time_now - time_status_start) > 4000)
         {
             v_check = analogRead(adpin_check);

             if (v_check < 200){
                click_control();
                set_status(1);
                delay(500);   //wait the device to take action
             }
         }
         break;
    }    

    flag_thread = (flag_thread+1)%NUM_THREAD;
  }
  PT_END(pt);
}

static int thread1_Updater(struct pt *pt)
{
  PT_BEGIN(pt);
  while(true) {
    need_init1 = false;
    PT_WAIT_UNTIL(pt, flag_thread == 1);
    timer.update();
    flag_thread = (flag_thread+1)%NUM_THREAD;
  }
  PT_END(pt);
}

int digital_led;
void Twinkle()
{
  if (!twinkling)	return;
  digital_led = (digital_led == LOW? HIGH:LOW);
  digitalWrite(pin_led, digital_led);

  WriteSerialDebug("current status: ", status);

}

void loop()
{
  thread0_Worker(&pt0);
  thread1_Updater(&pt1);
  thread2_Reserved(&pt2);
}
