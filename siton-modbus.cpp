
/*
  EmonHUB ridi komunikaci protokolem Modbus RTU po RS485
  a posila pozadavky na data jednotlivym Sitonum

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

  TODO:
  - pridat zapis na SD card
    https://randomnerdtutorials.com/esp32-data-logging-temperature-to-microsd-card/
  - pridat sqlite inicializace a INSERT
  - zkusit deep sleep time

struct {
  int id,
  unsigned long timestamp,
  int napeti;
  int proud;
  int vykon;
  int teplota;
  unsigned long vyroba;
} measured_data;

struct point my_point1;

struct point my_point2 = {2, 5, 3.7};

my_point2.x = 4;

*/

#include <Arduino.h>
#include <ModbusRtu.h> // https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino

#define TXenableRS485 2        //RE + DE RS485
#define LEDpin 13              //

uint16_t SITON_ADDR = 12; // siton slave adresa
unsigned long currentMillis;
unsigned int perioda = 2000; //cas mezi zasilanim pozadavku nodum
unsigned int datacnt;
uint16_t au16data[20]; //pole ulozeni dat modbus
uint8_t u8state;
byte u8query; //!< pointer to message query

Modbus master(0, Serial, TXenableRS485); // master, Serial, RS485 enabler port

modbus_t telegram; //
unsigned long u32wait;

union EmonData //Siton
{
  unsigned char item[];
  struct
  {
    int napeti;
    int proud;
    int vykon;
    int teplota;
    unsigned long vyroba;
  } val;
} data;

//=========================Setup================================================
void setup()
{

  pinMode(TXenableRS485, OUTPUT);
  pinMode(LEDpin, OUTPUT);
  Serial.begin(9600); //rychlost komunikace Raspberry

  master.start(); // rychlost komunikace ModBus 9600Bd
  master.setTimeOut(1000);       // pokud neni odpoved do 1000 ms, pokracuj na dalsi
  u32wait = millis();
  u8state = 0;
  u8query = 0;

  telegram.u8fct = 3;          // Modbus funkce (3=cteni holding registru)
  telegram.u16RegAdd = 0;      //startovaci adresa registru v slave
  telegram.u16CoilsNo = 20;    // pocet registru k cteni
  telegram.au16reg = au16data; // pointer pole pro ulozeni dat
}

//============================Hlavni smycka=====================================
void loop()
{

  switch (u8state)
  {
  case 0: //0 stav cekani na periodu komunikace
    currentMillis = millis();
    if ((unsigned long)(currentMillis - u32wait >= perioda))
      u8state++;
    break;
  case 1:                                //1 stav odeslani pozadavku
    telegram.u8id = (SITON_ADDR); // slave adresa
    digitalWrite(LEDpin, HIGH);
    delay(60);
    master.query(telegram); // zasle pozadavek na data
    digitalWrite(LEDpin, LOW);
    u8state++; //dalsi stav
    break;
  case 2:          //stav prijem dat
    master.poll(); // kontrola prichozich dat a ulozeni
    if (master.getState() == COM_IDLE)
    {
      u8state = 0;
      u32wait = millis();

      // kontrola jestli prisly nove data ze slave a odeslani na raspberry
      if (master.getInCnt() != datacnt && au16data[0] == SITON_ADDR)
      {
        data.val.napeti = au16data[4];                                          // napeti ve Voltech
        data.val.proud = au16data[5];                                           // proud ve formatu 865 (8,65A)
        data.val.vykon = au16data[6];                                           // vykon ve Wattech
        data.val.teplota = au16data[7];                                         // teplota ve °C
        data.val.vyroba = (((unsigned long)au16data[13] << 16) | au16data[12]); // vyroba ve Wh
        // toto jsou data z nodu SITON_ADDR
        for (unsigned int i = 1; i < sizeof(data.val); i++)
        {
          Serial.print(" ");
          Serial.print(data.item[i]);
        }
        Serial.println();
        datacnt = master.getInCnt();
      }

    }
    break;
  }
}
