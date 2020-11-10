#include <Arduino_FreeRTOS.h>

void Task_ReadFromSensor(void *pvParameters);
void Led(void *pvParameters);
void Taskprint( void *pvParameters );

void setup()
{
  Serial.begin(9600);
  xTaskCreate(
    Task_ReadFromSensor
    , "Sensor"
    , 100
    , NULL
    , 1
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
    Serial.println("Foc");
    analogWrite (speaker_pin, 50);
  }
  }
}

void Led(void *pvParameters)  
{
  pinMode(7, OUTPUT);
  while(1)
  {
    Serial.println("LED aprins-stins (blink)");
    digitalWrite(7, HIGH);   
    vTaskDelay( 300 / portTICK_PERIOD_MS ); 
    digitalWrite(7, LOW);   
    vTaskDelay( 300 / portTICK_PERIOD_MS ); 
  }
}


void Taskprint(void *pvParameters)  {
  int counter = 0;
  while(1)
  {
counter++;
  Serial.println(counter); 
  vTaskDelay(500 / portTICK_PERIOD_MS);    }
}
