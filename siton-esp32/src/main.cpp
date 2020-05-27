
/*
  ESP32 a Siton 210 - prenos dat pres protokol EasyTransfer po RS485

  InfluxDB UDP protocol:
  https://www.influxdata.com/blog/how-to-send-sensor-data-to-influxdb-from-an-arduino-uno/

  Komunikační rychlost 9600 Bd; 8 data bitů; 1 stop bit; bez parity.
  Vyčítat hodnoty lze pomocí funkce 03 (Read Holding Register). Při odeslání dat blikne na Arduinu ledka L.

  Struktura dat:
  hlavička – 06h, 85h
  počet dat – počet byte dat
  node ID – adresa, odkud jsou data vyslána
  address – adresa, komu jsou data určena
  command – 05h data odesílaná bez požadavku
  func – nevyužito
  data1 – napětí – (int) hodnota ve Voltech (245 = 245 V)
  data2 – proud – (int) hodnota ve formátu (856 = 8,56 A)
  data3 – výkon – (int) hodnota ve Wattech (1500 = 1500 W)
  data4 – teplota – (int) hodnota ve °C (62 = 62°C)
  data5 – data8 – rezerva (int)
  data9 – výroba – (unsigned long) hodnota ve Wh (1270564 = 1270,564 kWh)
  data10 – data12 – rezerva (unsigned long)
  kontr. součet – provedení operace XOR s jednotlivými byte dat a byte počtu dat

*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>

// ESP32 UART2 Tx Rx
#define TXenableRS485 4 //RE + DE RS485
#define LEDpin 2       //

// ESP32 battery voltage GPIO pin
#define voltagePin 34
// resolution 4096, max. volt 3.6
// 2275 = 1.885V = 460V
// 2180 = 461
const double voltage_koef = 2260 / 46.7;

// the voltage averaging
const int numReadings = 12;
static int voltage_readings[numReadings]; // the readings from the analog input
static int voltageReadIndex = 0;         // the index of the current reading
static int total = 0;                    // the running total

// WiFi
const char* ssid = "ufo8";
const char* password = "abcd1234";

// InfluxDB
// the IP address of your host
const byte host[] = {192, 168, 2, 4};
// the port that the InfluxDB UDP plugin is listening on
const int port = 8089;

WiFiUDP udp;

const byte SITON_ID = 12;

// global influxdb field
static String field;

const uint32_t read_data_interval = 60 * 1000; // interval pro sber dat
static uint32_t lastMillis = 0;              // will store last time LED was updated


// read voltage value
String readVoltage()
{
  static int average = 0; // the average

  int value = analogRead(voltagePin);

  // subtract the last reading:
  total = total - voltage_readings[voltageReadIndex];

  // voltage in 1/10 Volt precision
  voltage_readings[voltageReadIndex] = int(round(value / voltage_koef * 10));
  // add the reading to the total:
  total = total + voltage_readings[voltageReadIndex];
  // advance to the next position in the array:
  voltageReadIndex = voltageReadIndex + 1;

  // if we're at the end of the array...
  if (voltageReadIndex >= numReadings)
  {
    // ...wrap around to the beginning:
    voltageReadIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;

  // Serial.print("Voltage=");
  // Serial.print(value);
  // Serial.print("|");
  // Serial.println(voltage_readings[voltageReadIndex]);

  String measurement = "battery";

  String line = measurement + " " + "voltage=";
  line += average;
  line += "i";      // influx integer type

  return line;
}


// add to global variable field
static void influxAddValue(String name, float value)
{
  if (field.length() > 0)
  {
    field += ',';
  }

  field += name;
  field += '=';
  field += value;
}

String getSitonMeasurement(byte prijem[40])
{

  //  | measurement |, tag_set | | field_set | | timestamp |
  String measurement = "siton";

  // tag
  String tag = "nodeid=" + String(prijem[0]);

  // reset field in variable
  field = "";
  influxAddValue("napeti", word(prijem[5], prijem[4]));
  influxAddValue("proud", word(prijem[7], prijem[6]));
  influxAddValue("vykon", word(prijem[9], prijem[8]));
  influxAddValue("teplota", word(prijem[11], prijem[10]));

  // vyroba k W
  long vyroba = prijem[23] << 24;
  vyroba += prijem[22] << 16;
  vyroba += prijem[21] << 8;
  vyroba += prijem[20];
  influxAddValue("vyroba", vyroba);

  // add wifi signal
  influxAddValue("rssi", WiFi.RSSI());

  return measurement + "," + tag + " " + field;
}


String readSitonData(HardwareSerial mySerial)
{
  int nodeid;
  int byteRec;
  int delka;
  int cs;
  byte prijem[40];

  // inicializace prazdne influxdb row
  String line = "";

  byteRec = mySerial.read(); // Precte prvni byte
  // Serial.printf("receive header: 0x%02x\n", byteRec);
  if (byteRec == 0x06)
  {
    byteRec = mySerial.read();
    if (byteRec == 0x85)
    {
      delka = mySerial.read();
      cs = delka;
      // Serial.printf("receive delka: %02d\n", delka);

      for (byte i = 0; i < delka; i++)
      {
        byteRec = mySerial.read();
        prijem[i] = byteRec;
        cs ^= byteRec; //kontrolni soucet
      }

      byteRec = mySerial.read();
      if (cs == byteRec)
      {
        // kontrola na ID 12
        nodeid = int(prijem[0]);
        // Serial.printf("receive nodeid: %02d\n", nodeid);
        // data OK
        if (nodeid == SITON_ID)
        {
          // get Siton measurement
          line = getSitonMeasurement(prijem);
          Serial.print("line: ");
          Serial.println(line);
        }
      }
    }
  }
  return line;
}


// send UDP the packet to influx
void influxSendData(String line)
{
  // Serial.println("Sending UDP packet...");
  if (line.length() > 0)
  {
    Serial.print("send udp: ");
    Serial.println(line);

    udp.beginPacket(host, port);
    udp.print(line);
    udp.endPacket();
  }
}

void setup()
{

  pinMode(TXenableRS485, OUTPUT);
  pinMode(LEDpin, OUTPUT);

  Serial.begin(9600);
  Serial2.begin(9600);

  // MAX485 Tx
  digitalWrite(TXenableRS485, LOW);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
}


void loop()
{
  // read voltage every 5 sec
  String line = readVoltage();

  // send data to influx
  if (millis() - lastMillis > read_data_interval)
  {
    lastMillis = millis();
    digitalWrite(LEDpin, HIGH);

    // sned voltage
    influxSendData(line);

    // receiving data
    if (Serial2.available() > 0x25)
    {
      // read siton data into line
      // delay(10);
      line = readSitonData(Serial2);
      // Serial.println(line);
      Serial2.flush();

      // send UDP data
      influxSendData(line);
    }

    // finish led blink
    delay(60);
    digitalWrite(LEDpin, LOW);
  }

  // read kazde 5 sec
  delay(5000);
}
