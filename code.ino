#include <DHT.h>
#include <HCSR04.h>
#include <LiquidCrystalWired.h>

#define DHTTYPE DHT22
#define DHTPIN 5
#define ROW_COUNT 4
#define COL_COUNT 20
#define LCD_ADDRESS (0x7c >> 1)

const int trigPin = 7;
const int echoPin = 8;
const int lcd_SCL_Pin = A5;     // Refers to the clock
const int lcd_SDA_Pin = A4;     // Refers to serial data transmission

DHT dht(DHTPIN, DHTTYPE);
HCSR04 hc(trigPin, echoPin);
LiquidCrystalWired lcd(ROW_COUNT, COL_COUNT, FONT_SIZE_5x8, BITMODE_8_BIT);

const int motorControlPin = 12;
const int controllerOutputPin = 9;

// Refers to Indicators on LCD 
const int setPointRangePin = 4;
const int aboveSetPointPin = 10;
const int belowSetPointPin = 11;

const int setPointInputPin = A1;
const int systemOutputPin = A0;

float distance;
float temperature;
float humidity;

float setPointVoltage;
float outputVoltage;

const float Kp = 50.00;
const float Ki = 1840.00; 
unsigned long previousMillis = 0;
const long interval = 10;
float integralTerm = 0;

void setup() {
  Serial.begin(9600);
  dht.begin();
  lcd.begin(LCD_ADDRESS, &Wire);
  
  pinMode(setPointRangePin, OUTPUT);
  pinMode(aboveSetPointPin, OUTPUT);
  pinMode(belowSetPointPin, OUTPUT);
  pinMode(motorControlPin, OUTPUT);
  pinMode(setPointInputPin, INPUT);
  pinMode(controllerOutputPin, OUTPUT);
  // Setting Timer1 for 10-bit fast PWM mode
  TCCR1A = (1 << WGM11) | (1 << WGM10) | (1 << COM1A1);
  TCCR1B = (1 << WGM12) | (1 << CS11);
  lcd.turnOn();
  analogReference(DEFAULT);
}

void loop() {
  distance = hc.dist();
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  
  setPointVoltage = analogRead(setPointInputPin) * (5.0 / 1023.0);
  outputVoltage = analogRead(systemOutputPin) * (5.0/1023.0);

  OCR1A = piControl(setPointVoltage, outputVoltage);

  ledIndicators(outputVoltage, setPointVoltage);
  
  printOut();
  motorControl(temperature);
  lcdPrintOut();
}

void printOut(){
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("cm\n");
  Serial.print("Setpoint Voltage: ");
  Serial.print(setPointVoltage);
  Serial.print("V\n");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  char degree = '°';
  Serial.print(degree);
  Serial.print("C\n");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("%\n");
  if(digitalRead(motorControlPin) == HIGH){
    Serial.print("Motor Status: ON\n\n");
  }else{
    Serial.print("Motor Status: OFF\n\n");
  }
}

float piControl(float setPointVoltage, float outputVoltage){
  float error = setPointVoltage - outputVoltage;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval){
    float dt = (currentMillis - previousMillis) / 1000.00;
    previousMillis = currentMillis;

    float proportionalTerm = error;
    integralTerm += (error * dt);
    float controlOutput = (Kp * proportionalTerm) + (Ki * integralTerm);

    controlOutput = constrain(controlOutput, 0, 1023);
    return controlOutput;
  }
}
void lcdPrintOut(){
  lcd.clear();
  lcd.print("Distance: ");
  lcd.print(distance, 0);
  lcd.print("cm");
  lcd.setCursorPosition(1, 0);
  lcd.print("Setpoint: ");
  lcd.print(setPointVoltage);
  lcd.print("V");
  lcd.setCursorPosition(2, 0);
  lcd.print("Temp:");
  lcd.print(temperature);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursorPosition(3, 0);
  lcd.print("Hum:");
  lcd.print(humidity);
  lcd.print("%");
}
void motorControl(float temperature){
  if(temperature > 50.0){
    digitalWrite(motorControlPin, HIGH);
  }else if(temperature < 30.0){
    digitalWrite(motorControlPin, LOW);
  }
}
void ledIndicators(float outputVoltage, float setPointVoltage){
  if(outputVoltage > setPointVoltage + 0.02){
    digitalWrite(setPointRangePin, LOW);
    digitalWrite(aboveSetPointPin, HIGH);
    digitalWrite(belowSetPointPin, LOW);    
  }else if(outputVoltage == setPointVoltage){
    digitalWrite(setPointRangePin, HIGH);
    digitalWrite(aboveSetPointPin, LOW);
    digitalWrite(belowSetPointPin, LOW);
  }else if(outputVoltage < setPointVoltage - 0.02){
    digitalWrite(setPointRangePin, LOW);
    digitalWrite(aboveSetPointPin, LOW);
    digitalWrite(belowSetPointPin, HIGH);
  }
}


