#include <gfxfont.h>
#include <Adafruit_SPITFT_Macros.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_GFX.h>


#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_SSD1306.h>
//#include <splash.h>

#define ONE_WIRE_BUS 14                // DS18B20 data wire is connected to input 14
#define OLED_RESET 4                  // Adafruit needs this but we don't use for I2C

DeviceAddress thermometerAddress;     // custom array type to hold 64 bit device address

OneWire oneWire(ONE_WIRE_BUS);        // create a oneWire instance to communicate with temperature IC
DallasTemperature tempSensor(&oneWire);    // pass the oneWire reference to Dallas Temperature

Adafruit_SSD1306 display(OLED_RESET); // create a display instance

int rele = 13;

void setup() {

  pinMode(rele, OUTPUT);     // Initialize the FAN pin as an output
  digitalWrite(rele, LOW);
  
  Serial.begin(9600);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C address of the display
  display.clearDisplay();                     // clear the display buffer
  display.display();                          // update display

  Serial.println("DS18B20 Temperature IC Test");
  Serial.println("Locating devices...");
  tempSensor.begin();                         // initialize the temp sensor

  if (!tempSensor.getAddress(thermometerAddress, 0))
    Serial.println("Unable to find Device.");
  else {
    Serial.print("Device 0 Address: ");
    printAddress(thermometerAddress);
    Serial.println();
  }
  // tempSensor.setResolution(thermometerAddress, 11);      // set the temperature resolution (9-12)
}

float actualTemp = 0;
bool enfriar = true;
int estable = 0;

void loop() {
  
  tempSensor.requestTemperatures();
  actualTemp = tempSensor.getTempC(thermometerAddress);
  displayTemp(actualTemp);  // show temperature on OLED display
  if (enfriar)
    if (actualTemp < )// si temp es menor a 25 enfriar = false
      enfriar = false;
  else { // si no enfriar, 
    if (actualTemp < 28.00)// si baja de la temp minima, decremento enfriar = true
      enfriar = true;
  }
  
  if (enfriar)
    digitalWrite(rele, HIGH);   // Turn the FAN on (Note that LOW is the voltage level
  else {
    digitalWrite(rele, LOW);
  }
  delay(500);
}

void displayTemp(float temperatureReading) {             // temperature comes in as a float with 2 decimal places

  // set up OLED text size and print the temperature data
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Temp:");

  // show temperature °C
  display.print(temperatureReading, 1);  // rounded to 1 decimal place
  display.print((char)247);              // degree symbol
  display.println("C");
  Serial.print(temperatureReading);      // serial debug output
  Serial.print("°");
  Serial.println("C  ");

  display.display();                    // update the OLED display with all the new text
}

// print device address from the address array
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
