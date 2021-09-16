/*
  TrioSensor

  Reads time-of-flight distance data from three VL53L0X sensors,
  and then controls blue, yellow and red LEDs with these readings. 
  
  Data can be viewd on Serial Monitor.
*/

#include <Wire.h>
#include <VL53L0X.h>

#define SHT_SENSOR1 4
#define SHT_SENSOR2 5
#define SHT_SENSOR3 6

#define LED_BLUE 10
#define LED_YELLOW 11
#define LED_RED 12  

VL53L0X sensor;
VL53L0X sensor2;
VL53L0X sensor3;

int measure1;
int measure2;
int measure3;

int min_distance; 
int warning_distance1 = 400; // 400mm = 40cm, yellow light starts shining at this distance 
int warning_distance2 = 100; // 100mm = 10cm, red light starts shining at this distance 


/* Activating each of the sensors one by one.
 * Setting their perspective address and sensor.init() them one by one*/
void init_trio_sensor(){
  // defining and resetting all relevant XSHUT pins 
  pinMode(SHT_SENSOR1, OUTPUT);
  pinMode(SHT_SENSOR2, OUTPUT);
  pinMode(SHT_SENSOR3, OUTPUT);
  
  digitalWrite(SHT_SENSOR1, LOW);
  digitalWrite(SHT_SENSOR2, LOW);
  digitalWrite(SHT_SENSOR3, LOW);
  delay(500);
  
  // Activating and initializing sensor1 
  digitalWrite(SHT_SENSOR1, HIGH);
  delay(150);
  Serial.println(F("Sensor 1 activated"));
  if (sensor.init(true)){
    Serial.println(F("Sensor 1 initiated"));}
  else{
    Serial.println(F("Sensor 1 failed to initiate"));}
  delay(100);
  sensor.setAddress((uint8_t)01);
  Serial.println(F("Address set for sensor 1"));
  delay(150);
  
  // Activating and initializing sensor2 
  digitalWrite(SHT_SENSOR2, HIGH);
  delay(150);
  Serial.println(F("Sensor 2 activated"));
  if (sensor2.init(true)){
    Serial.println(F("Sensor 2 initiated"));}
  else{
    Serial.println(F("Sensor 2 failed to initiate"));}
  delay(100);
  sensor2.setAddress((uint8_t)02);
  Serial.println(F("Address set for sensor 2"));
  delay(150);


  // Activating and initializing sensor3 
  digitalWrite(SHT_SENSOR3, HIGH);
  delay(150);
  Serial.println(F("Sensor 3 activated"));
  if (sensor3.init(true)){
    Serial.println(F("Sensor 3 initiated"));}
  else{
    Serial.println(F("Sensor 3 failed to initiate"));}
  delay(100);
  sensor3.setAddress((uint8_t)03);
  Serial.println(F("Address set for sensor 3"));
  delay(150);
  delay(1000);
  
  // Start continuous 
  sensor.startContinuous();
  sensor2.startContinuous();
  sensor3.startContinuous();
}


/* Reading mm measurements from trio sensor
 * Printing mm measurements from trio sensor
 * Returning the smallest value of the three readings*/
int read_trio_sensor(){
  min_distance = 8191; // The initial min distance by default (will be changed by sensor values later)
  
  // Storing mm measurements from three sensos 
  measure1=sensor.readRangeContinuousMillimeters();
  measure2=sensor2.readRangeContinuousMillimeters();
  measure3=sensor3.readRangeContinuousMillimeters();
  
  // Printing measurements 
  Serial.print(F("1:"));
  Serial.print(measure1);
  Serial.print(F("mm "));
  
  Serial.print(F("2:"));
  Serial.print(measure2);
  Serial.print(F("mm "));
  
  Serial.print(F("3:"));
  Serial.print(measure3);
  Serial.print(F("mm "));
  Serial.println();

  // Determining the smallest value of the three measurements 
  if (measure1<min_distance){
    min_distance = measure1;
    }
  if (measure2<min_distance){
    min_distance = measure2;
  }
  if (measure3<min_distance){
    min_distance = measure3;
  }

  return min_distance;  // returning the smallest value of the three measurements for LED logic
    
}

/*Initializing the 3 LEDs*/
void init_led(){
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
}

/*Funciton controlling the blue LED*/
void blue_led_shines(){
  digitalWrite(LED_BLUE, HIGH);  
  delay(100);       
  digitalWrite(LED_BLUE, LOW);  
  delay(10);           
}

/*Funciton controlling the yellow LED*/
void yellow_led_shines(){
  digitalWrite(LED_YELLOW, HIGH);  
  delay(100);       
  digitalWrite(LED_YELLOW, LOW);  
  delay(10);   
}

/*Funciton controlling the red LED*/
void red_led_shines(){
  digitalWrite(LED_RED, HIGH);  
  delay(100);       
  digitalWrite(LED_RED, LOW);  
  delay(10);   
}


/*LED logic*/
void led_logic(){
  // Logic 
  if(min_distance>warning_distance1){
    blue_led_shines();
  }
  else if(min_distance<warning_distance1 and min_distance>warning_distance2){
    yellow_led_shines();
  }
  else{
    red_led_shines();
  }
}



/*Main Program*/
void setup(){
  // Setting up the I2C bus
  Wire.begin();
  Serial.begin (9600);

  init_led();         // Initialise all LEDs 
  init_trio_sensor(); // Initialise all three sensors 
}

void loop(){

  min_distance = read_trio_sensor();
  led_logic();

  
  delay(100);
}
