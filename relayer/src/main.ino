#include <ArduinoJson.h>
#include "pt.h"
#include <QueueList.h>
#include "Timer.h"
#include <avr/wdt.h>

#define SerialCom Serial1
#define SerialDbg Serial

const char VERSION[]="1.0.3";

const int NUM_THREAD = 3;
const int TIMEOUT_ACK = 300;
const char KEY_COMMAND_ID[] ="c";
const char KEY_SEQUENCE[] = "s";
const char KEY_PIN[] = "p";
const char KEY_MODE[] = "m";
const char KEY_VALUE[] = "v";
const char KEY_PINS[] = "ps";
const char KEY_VALUES[] = "vs";
const char KEY_VERSION[] = "version";
const char KEY_INPUT_PIN[] = "i_p";
const char KEY_OUTPUT_PIN[] = "o_p";


const int CMD_INIT = 0x01;
const int CMD_HEART = 0x02;
const int CMD_SET_MODE = 0x11;
const int CMD_SET_VALUE = 0x12;
const int CMD_QUERY_VALUE = 0x22;
const int CMD_QUERY_ADC = 0x23;
const int CMD_QUERY_VALUES = 0x24;
const int CMD_QUERY_VERSION = 0x25;
const int CMD_RELAY_VERIFY = 0x26;
const int CMD_EVENT_VALUE = 0x32;
const int CMD_EVENT_ADC = 0x33;
const int CMD_EVENT_VALUES = 0x34;
const int CMD_EVENT_VERSION = 0x35;
const int CMD_EVENT_RELAY_VERIFY = 0x36;

volatile static int flag_thread = 0;
static struct pt pt0, pt1, pt2;
volatile static boolean need_init0, need_init1, need_init2;
static Timer timer;

volatile static boolean waiting_ack= false;
volatile static float time_sent;
volatile static int sequence_mine, sequence_ack;
volatile static int resend= 0;

const int LEN_BUFFER_SEND = 1000;
const int LEN_BUFFER_RCV = 1000;
static byte buffer_rcv[LEN_BUFFER_RCV];
volatile static int pos_buffer_rcv;
static QueueList<String> queue_send;
const int pin_led = 13;
volatile static boolean twinkling = true;
volatile static int pin_query = 13, pin_query_adc = 1;

const int NUMBER_NONE = -9999999;
void WriteSerialDebug(char* p, long num1=NUMBER_NONE, long num2= NUMBER_NONE, long num3=NUMBER_NONE, long num4 = NUMBER_NONE)
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
  pinMode(pin_led, OUTPUT);
  timer.every(500, Twinkle);
  PT_INIT(&pt0);
  PT_INIT(&pt1);
  PT_INIT(&pt2);

  wdt_enable(WDTO_1S);
  initiate();
  WriteSerialDebug("setup() is called .......");
}

static void initiate()
{
  sequence_mine = 0;
  sequence_ack = 0;
  pos_buffer_rcv = 0;
  need_init0 = true;
  need_init1 = true;
  need_init2 = true;
  while (queue_send.count() > 0)
    queue_send.pop();
}

static void move_forward(int num)
{
  pos_buffer_rcv -= num;
  if (pos_buffer_rcv < 0)
    pos_buffer_rcv =0 ;

  for (int i = 0; i< pos_buffer_rcv; i++){
    buffer_rcv[i] = buffer_rcv[i+num];
  }
}

static byte check_sum(byte* p, int len){
  byte ret = 0;
  for(int i = 0; i< len; i++){
    ret += p[i];
  }
  return ret;
}


long write_check(int out_pin, int in_pin){
     WriteSerialDebug("enter write_check()!");
     digitalWrite(out_pin, LOW);
     unsigned long t_start = micros();
     unsigned long t_end;
     while(digitalRead(in_pin) == HIGH){
         t_end = micros();
         if (t_end-t_start > 1000000){
           WriteSerialDebug("the in_pin keep HIGH!");
            return -1;
         }
     }

     WriteSerialDebug("ready to verify output");
     t_start = micros();
     digitalWrite(out_pin, HIGH);
     while(digitalRead(in_pin) == LOW){
         wdt_reset();
         t_end = micros();
         if (t_end-t_start > 1000000){
           WriteSerialDebug("get relay response timeout!");
            return -1;
         }
     }     
     t_end = micros();
     WriteSerialDebug("response duration", t_end-t_start);
     return t_end-t_start;
}

long STABLE_FLUC = 500; # in micros
bool is_stable(long* values, int num){
     long max_value = values[0];
     long min_value = values[1];
     for (int i=1;i<num;i++){
         if (max_value < values[i]){
            max_value = values[i];
         }
         if (min_value > values[i]){
            min_value = values[i];
         }
     }
     return max_value - min_value < STABLE_FLUC*2;
}

long relay_verify(int out_pin, int in_pin){
     WriteSerialDebug("start relay_verify()!");
     long t_start = micros(), t_now;
     pinMode(out_pin, OUTPUT);
     pinMode(in_pin, INPUT);
     digitalWrite(out_pin, LOW);
     delay(300);
     wdt_reset();

     WriteSerialDebug("relay_verify pins' mode has been set!");
     int state =0 ; //0: finding stable value, 1: finding the follow stable values
     long stable_value = 0, current_value;
     int count=0;
     long values[50];
     while ((current_value = write_check(out_pin, in_pin))>=0) {
     	 wdt_reset();
         t_now = micros();
         if (t_now<t_start)
         {
            t_start = t_now;
         }
         if (t_now-t_start > 1000000*10){
           WriteSerialDebug("relay_verify timeout!");
            return -1;
         }

         values[++count] = current_value;
         if (count<4){
            continue;
         }
         if (state==0){
            if (is_stable(values, count)){
               long total = 0;
               for (int i=0;i<count;i++) {
                   total += values[i];
               }
               stable_value = total/count;
               state = 1;
            }
            else{
                for (int i=1;i<count;i++) {
                    values[i-1] = values[i];
                }
                count--;
            }
            continue;
         }
         if (current_value > stable_value+STABLE_FLUC ||
             current_value < stable_value-STABLE_FLUC){
            count--;
         }
         else{
            if (count>=10)
            {
               long total = 0;
               for (int i=0;i<count;i++) {
                   total += values[i];
               }
	       wdt_reset();
               return total/count;
            }
         }
     }
     WriteSerialDebug("relay verify failed!");
     return -1;
}


//read SerialCom for a PDU and feedback ACK to the sender
static JsonObject& ReadSPDU(StaticJsonBuffer<LEN_BUFFER_RCV>& _jsonBuffer)
{
  //read SerialCom into buffer_rcv
  while(pos_buffer_rcv< LEN_BUFFER_RCV){
    if (SerialCom.available()>0){
      char b = SerialCom.read();
      buffer_rcv[pos_buffer_rcv++] = b;
      SerialDbg.write(b);
    }
    else{
      break;
    }
  }

  //delete prefixed non-json bytes;
  int temp = 0;
  while (temp < pos_buffer_rcv && buffer_rcv[temp]!='{'){
    temp++;
  }
  move_forward(temp);

  //find the end-tag of json

  int pos_end = 1;
  while (pos_end < pos_buffer_rcv && buffer_rcv[pos_end]!='}'){
    pos_end++;
  }

  if ( pos_end+1 >= pos_buffer_rcv){
    //the end_tag has not been found, or the checksum has not been read
    return JsonObject::invalid();
  }
  //twinkling = !twinkling;
  //check check_sum
  byte checksum = check_sum(buffer_rcv, pos_end+1);
  byte checksum2 = buffer_rcv[pos_end+1];
  if (checksum!= checksum2){
    move_forward(pos_end+2);
    WriteSerialDebug("error check sum", checksum, checksum2);
    return JsonObject::invalid();
  }

  buffer_rcv[pos_end+1]='\0';
  JsonObject& pdu = _jsonBuffer.parseObject((char*)buffer_rcv);
  move_forward(pos_end+2);
  if(pdu == JsonObject::invalid()
     || !pdu.containsKey(KEY_COMMAND_ID)
     ||!pdu.containsKey(KEY_SEQUENCE)) {
    WriteSerialDebug("error message");
    return JsonObject::invalid();
  }

  if (pdu[KEY_COMMAND_ID] < 0x80)   {
    //Send the ACK for the valid PDU.
    StaticJsonBuffer<LEN_BUFFER_RCV> jsonBuffer;
    JsonObject& ack = jsonBuffer.createObject();
    ack[KEY_COMMAND_ID] = 0x80 + (int)pdu[KEY_COMMAND_ID];
    ack[KEY_SEQUENCE] = pdu[KEY_SEQUENCE];
    if (pdu[KEY_COMMAND_ID] == CMD_INIT || pdu[KEY_COMMAND_ID] == CMD_RELAY_VERIFY){
      SendSPDU(ack);
    }
    else if(queue_send.count()< 10){
      String str;
      ack.printTo(str);
      queue_send.push(str);
    }
  }
  return pdu;
}

static void ProcessSPDU(JsonObject& pdu)
{
  int command_id = pdu[KEY_COMMAND_ID];
  switch (command_id)    {
  case CMD_INIT:
    initiate();
    break;

  case CMD_SET_MODE:
    pinMode(pdu[KEY_PIN],strcmp(pdu[KEY_MODE],"in")==0?INPUT_PULLUP:OUTPUT);
    break;

  case CMD_SET_VALUE:
    digitalWrite(pdu[KEY_PIN], pdu[KEY_VALUE]);
    break;

  case CMD_QUERY_VALUE:
    {
      StaticJsonBuffer<LEN_BUFFER_RCV> jsonBuffer;
      JsonObject& event = jsonBuffer.createObject();
      event[KEY_COMMAND_ID] = CMD_EVENT_VALUE;
      event[KEY_PIN] = pdu[KEY_PIN];
      event[KEY_VALUE] = digitalRead(pdu[KEY_PIN]);
      if(queue_send.count()< 10){
        String str;
        event.printTo(str);
        queue_send.push(str);
      }
      pin_query = pdu[KEY_PIN];
    }
    break;

  case CMD_QUERY_ADC:
    {
      StaticJsonBuffer<LEN_BUFFER_RCV> jsonBuffer;
      JsonObject& event = jsonBuffer.createObject();
      event[KEY_COMMAND_ID] = CMD_EVENT_ADC;
      event[KEY_PIN] = pdu[KEY_PIN];
      event[KEY_VALUE] = analogRead(pdu[KEY_PIN]);
      if(queue_send.count()< 10){
        String str;
        event.printTo(str);
        queue_send.push(str);
      }
      pin_query_adc = pdu[KEY_PIN];
    }
    break;

  case CMD_QUERY_VERSION:
    {
      StaticJsonBuffer<LEN_BUFFER_RCV> jsonBuffer;
      JsonObject& event = jsonBuffer.createObject();
      event[KEY_COMMAND_ID] = CMD_EVENT_VERSION;
      event[KEY_VERSION] = VERSION;
      if(queue_send.count()< 10){
        String str;
        event.printTo(str);
        queue_send.push(str);
      }
    }
    break;

  case CMD_QUERY_VALUES:
    {
      StaticJsonBuffer<LEN_BUFFER_RCV> jsonBuffer;
      JsonObject& event = jsonBuffer.createObject();
      event[KEY_COMMAND_ID] = CMD_EVENT_VALUES;
      event[KEY_PINS] = pdu[KEY_PINS];
      JsonArray& values = event.createNestedArray(KEY_VALUES);
      for (int i =0;i<event[KEY_PINS].size();i++)
      {
	pinMode(pdu[KEY_PINS][i],INPUT_PULLUP);
	values.add(int(digitalRead(pdu[KEY_PINS][i])));
      }
      if(queue_send.count()< 10){
        String str;
        event.printTo(str);
        queue_send.push(str);
      }
    }
    break;
  case CMD_RELAY_VERIFY:
    {
	long value = relay_verify(pdu[KEY_OUTPUT_PIN], pdu[KEY_INPUT_PIN]);
        WriteSerialDebug("The relay_verify result is ", value);
	StaticJsonBuffer<LEN_BUFFER_RCV> jsonBuffer;
        JsonObject& event = jsonBuffer.createObject();
      	event[KEY_COMMAND_ID] = CMD_EVENT_RELAY_VERIFY;
      	event[KEY_VALUE] = value;
     	if (queue_send.count()< 10)
	{
           String str;
           event.printTo(str);
           queue_send.push(str);
      	}
    }
    break;

  }

  if (command_id >= 0x80 && sequence_mine == pdu[KEY_SEQUENCE]){
    sequence_ack = pdu[KEY_SEQUENCE];
  }
}

//write the pdu and this checksum to the SerialCom.
static void SendSPDU(JsonObject& pdu)
{
  static byte buffer_send[LEN_BUFFER_SEND];
  int temp = pdu.printTo((char*)buffer_send, LEN_BUFFER_SEND-1);
  if (temp>0)    {
    buffer_send[temp] = check_sum(buffer_send, temp);
  }
  SerialCom.write(buffer_send, temp + 1);
  WriteSerialDebug("sent:");
  SerialDbg.write(buffer_send, temp + 1);
}


//Fetch a PDU from queue and write to the SerialCom waiting until the feeback.
static int thread2_WriteSPDU(struct pt *pt)
{
  PT_BEGIN(pt);

  while(true) {
    need_init2 = false;

    PT_WAIT_UNTIL(pt, flag_thread == 2);

    //if waiting_ack, then loop until the ack arrived or timeout.
    //The timeout value is assumed as 0.1 second.
    while(waiting_ack && sequence_ack != sequence_mine && time_sent+TIMEOUT_ACK>millis()){
      flag_thread = (flag_thread+1)%NUM_THREAD;
      PT_WAIT_UNTIL(pt, flag_thread == 2);
      if(need_init2) break;
    }

    if (need_init2){
      waiting_ack = false;
      continue;
    }

    if(waiting_ack && (sequence_mine == sequence_ack || resend >= 3)){
      queue_send.pop();
      waiting_ack = false;
      resend = 0;
    }
    else if (waiting_ack)
    {
      resend ++;
    }	

    if(!queue_send.isEmpty()){
      String str = queue_send.peek();
      StaticJsonBuffer<LEN_BUFFER_RCV> jsonBuffer;
      JsonObject& pdu = jsonBuffer.parseObject(str);
      if (pdu[KEY_COMMAND_ID]<0x80){
        sequence_mine += 1;
        pdu[KEY_SEQUENCE] = sequence_mine;
        SendSPDU(pdu);
        time_sent = millis();
        if(time_sent >= 0xFFFFFFFF-TIMEOUT_ACK){
          delay(TIMEOUT_ACK*2);
          time_sent = millis();
        }
        waiting_ack = true;
      }
      else{
	    SendSPDU(pdu);
	    queue_send.pop();
      }
    }
    flag_thread = (flag_thread+1)%NUM_THREAD;
  }
  PT_END(pt);
}

static int thread0_StandBySerial(struct pt *pt)
{
  PT_BEGIN(pt);
  while(true) {
    need_init0 = false;
    PT_WAIT_UNTIL(pt, flag_thread == 0);

    StaticJsonBuffer<LEN_BUFFER_RCV> _jsonBuffer;
    JsonObject& pdu = ReadSPDU(_jsonBuffer);
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

  WriteSerialDebug("pin value ", pin_query, digitalRead(pin_query));
  WriteSerialDebug("pin value of adc ", pin_query_adc, analogRead(pin_query_adc));
}

void loop()
{
  thread0_StandBySerial(&pt0);
  thread1_Lighter(&pt1);
  thread2_WriteSPDU(&pt2);

  wdt_reset();
}
