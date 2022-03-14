#include <OneWire.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define ONE_WIRE_BUS 5
#define SAMPLE_TIME 2000
#define PINA 2
#define PINB 3
#define BUTTON_PIN 4
#define COMPRESSOR_PIN 6
#define INTERRUPT 0                   // that is, pin 2
#define COMPRESSOR_MAX_RUNTIME 1800000UL //30MIn Max 
#define COMPRESSOR_MIN_RESTTIME 300000UL //1Min rest 

#define SET_INDEX 0
#define HI_INDEX 1
#define LO_INDEX 2
#define ALERT_HI_INDEX 3
#define ALERT_LO_INDEX 4
#define MENU_ITEM_NUMBER 5

#define COMPRESSOR_RUN 0
#define COMPRESSOR_STOP 1

volatile boolean fired;
volatile boolean up;
uint32_t prev_millis = 0;
static float rotaryCount = 0;
float temperature = 0;
float eeprom_variables[MENU_ITEM_NUMBER];

uint8_t compressor_state = COMPRESSOR_STOP;
uint8_t pending_compressor_state = COMPRESSOR_STOP;
uint32_t compressor_millis = 0;
uint8_t sensor_number = 0;
uint8_t menu_item = 0;
bool button_state = true;
bool prev_button_state = true;
bool conversion_started = false;
bool menu_item_selected = false;
bool sensor_error = false;
bool alarm = false;

byte i;
byte present = 0;
byte type_s;
byte data[12];
byte addr[8];

LiquidCrystal_I2C lcd(0x27, 16, 2);
OneWire ds(ONE_WIRE_BUS);

void isr()
{
  if (digitalRead(PINA))
    up = digitalRead(PINB);
  else
    up = !digitalRead(PINB);
  fired = true;

} // end of isr

void LCD_refresh()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  if (!sensor_error)
  {
    lcd.print("Temp:");
    lcd.print(temperature);
    lcd.print((char)223);
    lcd.print('C');
    lcd.setCursor(13, 0);
    if (compressor_state == COMPRESSOR_RUN)
      lcd.print("ON");
    else
      lcd.print("OFF");
  }
  else
  {
    lcd.print(" SENSOR ERROR!");
  }
  //Second Line
  lcd.setCursor(0, 1);
  if (menu_item_selected)
  {
    lcd.print((char)126);
  }
  else
  {
    lcd.print((char)62);
  }

  switch (menu_item)
  {
  case SET_INDEX:
    lcd.print("Set: ");
    lcd.print(eeprom_variables[menu_item]);
    break;
  case HI_INDEX:
    lcd.print("HI: ");
    lcd.print(eeprom_variables[menu_item]);
    break;
  case LO_INDEX:
    lcd.print("LO: ");
    lcd.print(eeprom_variables[menu_item]);
    break;
  case ALERT_HI_INDEX:
    lcd.print("AL HI: ");
    lcd.print(eeprom_variables[menu_item]);
    break;
  case ALERT_LO_INDEX:
    lcd.print("AL LO: ");
    lcd.print(eeprom_variables[menu_item]);
    break;
  default:
    break;
  }
}

void EEPROM_updateValue(uint8_t addr, float value)
{
  EEPROM.put(4 * addr, value);
}

/*
void EEPROM_initialWRITE()
{
  for(uint8_t i = 0; i<4*MENU_ITEM_NUMBER; i++)
  {

    EEPROM.put(i, 0);
  }
}
*/
void EEPROM_readValues()
{
  for (uint8_t i = 0; i < MENU_ITEM_NUMBER; i++)
  {
    EEPROM.get(4 * i, eeprom_variables[i]);
  }
}

void buttonHandler()
{
  if (menu_item_selected)
  {
    menu_item_selected = false;
    EEPROM_updateValue(menu_item, eeprom_variables[menu_item]);
  }

  else
    menu_item_selected = true;
  LCD_refresh();
}

void encoderIncrementHandler()
{
  if (!menu_item_selected)
  {
    if (!menu_item_selected)
      menu_item++;
    if (menu_item >= (MENU_ITEM_NUMBER - 1))
    {
      menu_item = 0;
    }
  }
  else
  {
    eeprom_variables[menu_item] += 0.01;
  }
}

void encoderDecrementHandler()
{
  if (!menu_item_selected)
  {
    if (menu_item == 0)
      menu_item = MENU_ITEM_NUMBER - 1;
    else
      menu_item--;
  }
  else
  {
    eeprom_variables[menu_item] -= 0.01;
  }
}

void temperatureHandler()
{
  if (temperature > (eeprom_variables[SET_INDEX] + eeprom_variables[HI_INDEX]))
  {
    if (pending_compressor_state != COMPRESSOR_RUN)
      pending_compressor_state = COMPRESSOR_RUN;
  }
  else if (temperature < (eeprom_variables[SET_INDEX] + eeprom_variables[LO_INDEX]))
  {
    if (pending_compressor_state != COMPRESSOR_STOP)
      pending_compressor_state = COMPRESSOR_STOP;
  }

  if (temperature > eeprom_variables[ALERT_HI_INDEX] || temperature < eeprom_variables[ALERT_HI_INDEX])
  {
    alarm = true;
   // compressor_state = COMPRESSOR_STOP;
  }
}

void compressorStart()
{
  compressor_state = COMPRESSOR_RUN;
  compressor_millis = millis();
  digitalWrite(COMPRESSOR_PIN, LOW);
}

void compressorStop()
{
  compressor_state = COMPRESSOR_STOP;
  compressor_millis = millis();
  digitalWrite(COMPRESSOR_PIN, HIGH);
}

void compressorHandler()
{
  switch (compressor_state)
  {
  case COMPRESSOR_STOP:
    if (((millis() - compressor_millis) > COMPRESSOR_MIN_RESTTIME) && pending_compressor_state != compressor_state)
    {
      compressorStart();
    }

    break;
  case COMPRESSOR_RUN:
    if ((millis() - compressor_millis) > COMPRESSOR_MAX_RUNTIME)
    {
      compressorStop();
    }
    break;
  default:
    compressorStop();
    delay(10000);
    break;
  }
}

void setup()
{
  pinMode(BUTTON_PIN, INPUT);
  pinMode(COMPRESSOR_PIN, OUTPUT);
  digitalWrite(COMPRESSOR_PIN, HIGH);
  attachInterrupt(INTERRUPT, isr, CHANGE); // interrupt 0 is pin 2, interrupt 1 is pin 3
  lcd.begin();
  lcd.backlight();
  // EEPROM_initialWRITE();
  EEPROM_readValues();
  compressor_millis = millis();
  Serial.begin(115200);
} // end of setup

void loop()
{

  button_state = digitalRead(BUTTON_PIN);
  if (button_state != prev_button_state && button_state == HIGH)
  {
    buttonHandler();
  }
  prev_button_state = button_state;

  if (!conversion_started)
  {
    conversion_started = true;
    if (!ds.search(addr))
    {
      if (sensor_number == 0)
      {
        sensor_error = true; //=============================================================================================<<<<<<<<<<<<<<<<<<<<<<<
        Serial.println("No more addresses.");
        compressorStop();
        Serial.println();
      }
      sensor_number = 0;
      ds.reset_search();
      delay(250);
      return;
    }
    sensor_number++;
    sensor_error = false;
    /*
    Serial.print("ROM =");
    for ( i = 0; i < 8; i++)
    {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    }

    if (OneWire::crc8(addr, 7) != addr[7])
    {
      Serial.println("CRC is not valid!");
      return;
    }
    Serial.println();
  */
    // the first ROM byte indicates which chip
    switch (addr[0])
    {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      //type_s = 1;

      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      //type_s = 0;

      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      //type_s = 0;

      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      sensor_error = true;
      return;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 0);
    prev_millis = millis();
  }

  if (millis() - prev_millis >= SAMPLE_TIME)
  {
    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE); // Read Scratchpad

    Serial.print("  Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");
    for (i = 0; i < 9; i++)
    {
      data[i] = ds.read();
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.print(" CRC=");
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.println();

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s)
    {
      if (data[7] == 0x10)
      {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    }
    else
    {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00)
        raw = raw & ~7; // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20)
        raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40)
        raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    temperature = (float)raw / 16.0;
    Serial.print("  Temperature = ");
    Serial.print(temperature);
    temperatureHandler();
    compressorHandler();
    LCD_refresh();
    conversion_started = false;
  }

  if (fired)
  {
    if (up)
    {
      encoderIncrementHandler();
    }
    else
    {
      encoderDecrementHandler();
    }
    fired = false;
    LCD_refresh();
    Serial.print("Count = ");
    Serial.println(rotaryCount);
  }
}
