#include <Arduino_FreeRTOS.h>
#include<semphr.h>
#include<SoftwareSerial.h>

#define RxD 11
#define TxD 10
                
SoftwareSerial mySerial(RxD, TxD);
int data=0;


void Task_ReadFromSensor(void *pvParameters);
void Led(void *pvParameters);
void Taskprint( void *pvParameters );


SemaphoreHandle_t xSerialSemaphore;

void setup()
{

  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);
  
   Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  mySerial.begin(9600);

  if (xSerialSemaphore == NULL){
    xSerialSemaphore = xSemaphoreCreateMutex();
    if ((xSerialSemaphore) != NULL)
    xSemaphoreGive((xSerialSemaphore));
    }
    
  xTaskCreate(
    Task_ReadFromSensor
    , "Sensor"
    , 100
    , NULL
    , 1 //prioritate
    , NULL
    );

  xTaskCreate(
    Led
    , "Led"
    , 128
    , NULL
    , 1
    , NULL
    );
vTaskStartScheduler();
}
void loop()
{
  
//if (mySerial.available()) {
    //Serial.read();
  //  char c = mySerial.read();
 // Serial.write(c);
 // }
 // if (Serial.available()) {
  //  mySerial.write("FOC!!");
  //}
}

const int val_min_sensor = 0; // valoarea min a senzorului
const int val_max_sensor = 1024; // valoarea max a senzorului
int speaker_pin = 3;// pin analog 3 -> pt sunet
int range;


void Task_ReadFromSensor(void *pvParameters)
{
  pinMode(8, OUTPUT);
  while(1){
   
  int sensorReading = analogRead(A0); //senzorul e conectat la pinul A0 (pin analog)
  range = map(sensorReading, val_min_sensor, val_max_sensor, 0, 3);
  
  if (range == 0)
  {
    if (xSemaphoreTake (xSerialSemaphore, (TickType_t) 10) == pdTRUE){
     // Serial.println("Foc");
        if (mySerial.available()>0) {
    mySerial.write("FOC!");
  } 
    }     
      //Serial.println(sensorReading);
      
      analogWrite (speaker_pin, 50);
      xSemaphoreGive(xSerialSemaphore);
  
   vTaskDelay(1);
  }
}
}



void Led(void *pvParameters)  
{
  pinMode(7, OUTPUT);
  while(1)
  {
    if (xSemaphoreTake (xSerialSemaphore, (TickType_t) 5) == pdTRUE){
     Serial.println("LED aprins-stins (blink)");
    }
     digitalWrite(7, HIGH);   
     vTaskDelay( 300 / portTICK_PERIOD_MS ); 
     digitalWrite(7, LOW);   
     vTaskDelay( 300 / portTICK_PERIOD_MS ); 
     xSemaphoreGive(xSerialSemaphore);
 
    vTaskDelay(1);
}
}


void Taskprint(void *pvParameters)  {
  int counter = 0;
  while(1)
  {
    if (xSemaphoreTake (xSerialSemaphore, (TickType_t) 5) == pdTRUE){
 // counter++;
  Serial.println(counter); 
    }
  xSemaphoreGive(xSerialSemaphore);
  vTaskDelay(500 / portTICK_PERIOD_MS);    
}
  vTaskDelay(1);}
