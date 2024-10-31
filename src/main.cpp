#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <LiquidCrystal_I2C.h>
#include <TM1637Display.h>

#define CLK 2
#define DIO 3
#define BUTTON_PIN_HOUR 6
#define BUTTON_PIN_MINUTE 7

Adafruit_BMP280 bmp280; // I2C interface
LiquidCrystal_I2C lcd(0x27, 16, 2);
TM1637Display display(CLK, DIO);

uint8_t data[] = {0xff, 0xff, 0xff, 0xff};
uint8_t blank[] = {0x00, 0x00, 0x00, 0x00};

int hours = 0;
int minutes = 0;
int seconds = 0;  // Track seconds as time passes
bool showColon = false;

// Variables for non-blocking timing
unsigned long lastSecondUpdate = 0;
unsigned long lastButtonCheck = 0;
unsigned long lastDisplayUpdate = 0;
const unsigned long secondInterval = 1000;  // 1 second interval for time tracking
const unsigned long buttonInterval = 100;   // 100ms interval for checking buttons
const unsigned long displayInterval = 1000; // 1 second interval for updating display

void setup()
{
  Serial.begin(115200);
  delay(10);
  Serial.println("Starting...");
  display.setBrightness(0x0f);
  display.setSegments(data);
  Wire.begin(); // Initializes the I2C bus (A4 = SDA, A5 = SCL on Nano)

  lcd.init();
  lcd.backlight();
  lcd.setCursor(3, 0);

  if (!bmp280.begin(0x76))
  { // Use 0x76 or 0x77 depending on your BMP280
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  pinMode(BUTTON_PIN_HOUR, INPUT_PULLUP);
  pinMode(BUTTON_PIN_MINUTE, INPUT_PULLUP);
}

void checkButtons();
void updateDisplayAndReadings();
void incrementTime();
void sendToSevenSegment(int16_t value, bool showDecimalPoint);

void loop()
{
  unsigned long currentMillis = millis();

  // Track time every second
  if (currentMillis - lastSecondUpdate >= secondInterval)
  {
    lastSecondUpdate = currentMillis;
    incrementTime();
  }

  // Check buttons every 100ms
  if (currentMillis - lastButtonCheck >= buttonInterval)
  {
    lastButtonCheck = currentMillis;
    checkButtons();
  }

  // Update display and readings every 1 second
  if (currentMillis - lastDisplayUpdate >= displayInterval)
  {
    lastDisplayUpdate = currentMillis;
    updateDisplayAndReadings();
  }
}

void incrementTime()
{
  seconds++;
  if (seconds >= 60)
  {
    seconds = 0;
    minutes++;
    if (minutes >= 60)
    {
      minutes = 0;
      hours++;
      if (hours >= 24)
      {
        hours = 0;  // Reset after 24 hours
      }
    }
  }
}

void checkButtons()
{
  if (digitalRead(BUTTON_PIN_HOUR) == LOW)
  {
    hours++;
    if (hours >= 24) hours = 0; // Roll over to 0 after 23
    Serial.print("Hours incremented to: ");
    Serial.println(hours);
    while (digitalRead(BUTTON_PIN_HOUR) == LOW) { } // Wait for button release
  }
  
  if (digitalRead(BUTTON_PIN_MINUTE) == LOW)
  {
    minutes++;
    if (minutes >= 60) minutes = 0; // Roll over to 0 after 59
    Serial.print("Minutes incremented to: ");
    Serial.println(minutes);
    while (digitalRead(BUTTON_PIN_MINUTE) == LOW) { } // Wait for button release
  }
}

void updateDisplayAndReadings()
{
  // Get temperature and pressure values
  float temperature = bmp280.readTemperature(); // Temperature in °C
  float pressure = bmp280.readPressure();       // Pressure in Pa

  lcd.setCursor(0, 0);
  lcd.print("T: ");
  lcd.setCursor(3, 0);
  lcd.print(temperature);
  lcd.setCursor(8, 0);
  lcd.print(" C");

  lcd.setCursor(0, 1);
  lcd.print("P: ");
  lcd.setCursor(3, 1);
  String pressureString = String(pressure / 100.0f, 2);
  lcd.print(pressureString);
  lcd.setCursor(8, 1);
  lcd.print("hPa");

  // Print to Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" C\tPressure: ");
  Serial.print(pressure);
  Serial.println(" Pa");

  // Calculate temperature in Fahrenheit
  float fahrenheit = 1.8 * temperature + 32;
  Serial.print("Temperature in Fahrenheit: ");
  Serial.print(fahrenheit);
  Serial.println(" °F");

  // Toggle colon and send time to 7-segment display
  showColon = !showColon;
  sendToSevenSegment(hours * 100 + minutes, showColon);
}

void sendToSevenSegment(int16_t value, bool showDecimalPoint)
{
  uint8_t dots = showDecimalPoint ? 0b01000000 : 0;
  display.showNumberDecEx(value, dots, true);
}
