#include <ArduinoJson.h>
#include "pt.h"
#include <QueueList.h>
#include "Timer.h"

const int NUM_THREAD = 2;
static int flag_thread = 0;
static struct pt pt0, pt1;
static Timer timer;

int pin_led = 13;

void setup()
{
Serial.begin(9600);
pinMode(pin_led, OUTPUT);
timer.every(500, Twinkle);
PT_INIT(&pt0);
PT_INIT(&pt1);
}

void loop()
{
	thread0_StandBySerial(&pt0);
	thread1_Lighter(&pt1);
}

//read Serial for a PDU and feedback ACK to the sender
static JsonObject& ReadSPDU()
{
}

static void ProcessSPDU(JsonObject& pdu)
{
}

//Write a PDU to the Serial in a loop, until got feedback from the receiver
static void WriteSPDU(JsonObject& pdu)
{
}

static int thread0_StandBySerial(struct pt *pt)
{
	PT_BEGIN(pt);
	while(true) {
		    PT_WAIT_UNTIL(pt, flag_thread == 0);
		    JsonObject& pdu = ReadSPDU();
		    if (pdu != JsonObject::invalid())
		    {
			ProcessSPDU(pdu);
		    }
		    flag_thread = (flag_thread+1)%NUM_THREAD;		    
	}
	PT_END(pt);
}

static int thread1_Lighter(struct pt *pt)
{
	PT_BEGIN(pt);
	while(true) {
		    PT_WAIT_UNTIL(pt, flag_thread == 1);
		    timer.update();
		    flag_thread = (flag_thread+1)%NUM_THREAD;
	}
	PT_END(pt);
}

int digital_led;
void Twinkle()
{
	digital_led = (digital_led == LOW? HIGH:LOW);
	digitalWrite(pin_led, digital_led);
}