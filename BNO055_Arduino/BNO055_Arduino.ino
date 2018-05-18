///BenodigdeLibrariesBNO055/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
///BenodigdeLibrariesCanBus/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>

///VariabelenBNO055/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
///VariabelenCANbus/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long prevTX1 = 0;                                        // Variabele om laatste tijd te onthouden
const unsigned int invlTX1 = 100;                                 // Interval tussen de CAN-berichten
unsigned long prevTX2 = 0;                                        // Variabele om laatste tijd te onthouden
const unsigned int invlTX2 = 100;                                 // Interval tussen de CAN-berichten

///Gebruikte Functies///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*****
Doel: Basisinformatie weergeven over de sensor
Parameters: /
Return value: /
*****/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

///Setup functie////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(void)
{
  Serial.begin(115200);
///BNO055///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
   
  /* Display some basic information on this sensor */
  displaySensorDetails();
///CANbus///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println("CAN Write - Testing transmission of CAN Bus testBerichts");
    delay(1000);
  
    if(Canbus.init(CANSPEED_500))       //Initialise MCP2515 CAN controller at the specified speed
    {
    Serial.println("CAN Init ok");
    }
    else
    {
    Serial.println("Can't init CAN");
    }
}

///Loop/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(void)
{
///BNO055 + print statements for debugging//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  sensors_event_t event;
  bno.getEvent(&event);
  
  Serial.print(F("Orientation: "));
  Serial.println((int)event.orientation.x);
  delay(BNO055_SAMPLERATE_DELAY_MS);
///BNO0555 data in CANbericht plaatsen//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
tCAN BNOcoor;

    BNOcoor.id = 0xB1;                                      //B1 is de hexadecimale identifier van het bericht die de rotatiedata bevat
    BNOcoor.header.rtr = 0;
    BNOcoor.header.length = 2;                              //de data length code is 2 decimaal
    BNOcoor.data[0] = (int)event.orientation.x;             //Intel byte order event.orientation.x (LSB eerst)
    BNOcoor.data[1] = (int)event.orientation.x>>8;        //Om een waarde die varieert tussen 0 en 360 door te sturen hebben we genoeg aan twee bytes
    

    if (millis() - prevTX1 >= invlTX1)                     //Interval waarop canbericht 1 verstuurd wordt
    {
    prevTX1 = millis();
    mcp2515_send_message(&BNOcoor);                         //Message is een pointer naar een geheugenlocatie waar we de gewenste data kunnen terugvinden
    }  
}
