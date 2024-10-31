#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <LiquidCrystal_I2C.h>
#include <TM1637Display.h>

#define CLK 2
#define DIO 3
#define BUTTON_PIN 6

Adafruit_BMP280 bmp280; // I2C interface
LiquidCrystal_I2C lcd(0x27, 16, 2);
TM1637Display display(CLK, DIO);

uint8_t data[] = {0xff, 0xff, 0xff, 0xff};
uint8_t blank[] = {0x00, 0x00, 0x00, 0x00};

long startTime = 0;
int lastButtonState = HIGH;
bool showPressure = true;

void setup()
{
  startTime = millis();
  Serial.begin(115200);
  delay(10);
  Serial.println("Starting...");
  display.setBrightness(0x0f);
  display.setSegments(data);
  Wire.begin(); // Initializes the I2C bus (A4 = SDA, A5 = SCL on Nano)

  lcd.init();
  lcd.backlight();
  lcd.setCursor(3, 0);
  // Initialize I2C communication and BMP280

  if (!bmp280.begin(0x76))
  { // Use 0x76 or 0x77 depending on your BMP280
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void sendToSevenSegment(int16_t value, bool showDecimalPoint);

void loop()
{
  //  Serial.println("Loop starting...");

  int currentButtonState = digitalRead(BUTTON_PIN);

  if (currentButtonState == LOW && lastButtonState == HIGH)
  {
    showPressure = !showPressure; // Toggle the boolean
    Serial.print("Toggle state: ");
    Serial.println(showPressure); // Print the new state
  }

  lastButtonState = currentButtonState;
  delay(50);
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

  // Print the results
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" C\tPressure: ");
  Serial.print(pressure);
  Serial.println(" Pa");

  if (showPressure)
  {
    sendToSevenSegment(pressure / 100, false);
  }
  else
  {
    sendToSevenSegment(temperature * 100, true);
  }

  delay(940); // Delay for 2 seconds before next reading

  // calculate the temprature in fahrenheit
   float fahrenheit = 1.8 * temperature + 32;
  // Print the results
  Serial.print("Temperature in Fahrenheit: ");
  Serial.print(fahrenheit);
  Serial.println(" °F");
  
}
void sendToSevenSegment(int16_t value, bool showDecimalPoint)
{
  uint8_t dots = showDecimalPoint ? 0b01000000 : 0;
  display.showNumberDecEx(value, dots, true);
}