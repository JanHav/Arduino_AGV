/*
Purpose: Collecting data of the ultrasonic sensors mounted on the AGV, sending that data via the CAN bus to the main controller
1. start_sensor(address)
  Description:Uses the I2C library to start a sensor at the given address. You must start the sensor 100ms before requesting a range from it if you want the most recent information.
  Parameters:addr: an even byte value corresponding to the 8-bit address of the sensor you want to command a range reading at. The default value for I2C‑MaxSonar sensors is 224
  Returns: errorlevel: defaults to FALSE (value of 0) when the communication is successfully completed. Set to TRUE (value of 1) if there was a communication error
  
2. read_sensor (address)
  Description: Uses the I2C library to read a sensor at the given address
  Paramters: addr: an even byte value corresponding to the 8-bit address you are reading from. The default value for I2C‑MaxSonar sensors is 224
  Returns: range: an int value corresponding to the distance found by the sensor (for the current I2C‑MaxSonar sensors this is a value between 20cm and 765cm). 
  A value of “0” represents that the sensor could not be read and that there was an error in communication 
  
3. change_address(old address, new address)
  Description: Uses the I2C library to change the sensor address at oldaddr to newaddr.
  Parameters: Parameters: oldaddr: an even byte value corresponding to the 8-bit address of the sensor you want to update to a new address. 
  The default value for the I2C‑MaxSonar sensors is 224.
  Returns: errorlevel: defaults to FALSE (value of 0) when the communication is successfully completed. Set to TRUE (value of 1) if there was a communication error.
  Note: For other devices operating on the I2C bus: It is recommended that you use addresses for the I2C‑MaxSonar that are below 240. This keeps the sensor away from the reserved address space.
*/

///Define statements for I2C communication between UNO and Ultrasonic sensors//////////////////////////////////////////////////
#define SCL_PIN 5               //Default SDA is Pin5 PORTC for the UNO -- you can set this to any tristate pin
#define SCL_PORT PORTC 
#define SDA_PIN 4               //Default SCL is Pin4 PORTC for the UNO -- you can set this to any tristate pin
#define SDA_PORT PORTC
#define I2C_TIMEOUT 100         //Define a timeout of 100 ms -- do not wait for clock stretching longer than this time

///I2C library////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <SoftI2CMaster.h>      //Not a standard Arduino library (you can download it from https://github.com/felias-fogg/SoftI2CMaster)

///BenodigdeLibrariesCanBus//////////////////////////////////////////////////////////////////
#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>

///Variables MaxBotix ultrasonic sensors/////////////////////////////////////////////////////////////////////////////////////
boolean error1 = 0;  //Create a bit to check for catch errors as needed.
int range1;
boolean error2 = 0;  //Create a bit to check for catch errors as needed.
int range2;
boolean error3 = 0;  //Create a bit to check for catch errors as needed.
int range3;
boolean error4 = 0;  //Create a bit to check for catch errors as needed.
int range4;

///VariabelenCanBus//////////////////////////////////////////////////////////////////////////
int TestCanWaarde = 8888;                                         // Data die we verzenden in D0 en D1 (intel byte order) en D2 en D3 (motorola byte order)
int dataSonar = 15900;                                            // Bitwise operations kan je enkel uitvoeren op een int datatype
unsigned long prevTX1 = 0;                                        // Variabele om laatste tijd te onthouden
const unsigned int invlTX1 = 100;                                  // Interval tussen de CAN-berichten
unsigned long prevTX2 = 0;                                        // Variabele om laatste tijd te onthouden
const unsigned int invlTX2 = 100;                                 // Interval tussen de CAN-berichten

///SetupLoop/////////////////////////////////////////////////////////////////////////////////
void setup() {
///Snelheid seriële monitor instellen////////////////////////////////////////////////////////////////////////////////////////
  Serial.begin(115200);
///Initialisatie I2C/////////////////////////////////////////////////////////////////////////////////////////////////////////
  i2c_init();
///SetupCanBus///////////////////////////////////////////////////////////////////////////////
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

///VoidLoop//////////////////////////////////////////////////////////////////////////////////
void loop() 
{
  
///Binnenlezen waarden verschillende ultrasone sensoren////////////////////////////////////// 
  //Take a range reading at the address of 222
  error1 = start_sensor(222);    //Start the sensor and collect any error codes.
  if (!error1)
  {                  //If you had an error starting the sensor there is little point in reading it as you will get old data.
    delay(100);
    range1 = read_sensor(222);   //reading the sensor will return an integer value -- if this value is 0 there was an error
    Serial.print("Sensor1:");Serial.println(range1); 
  }
  
  //Take a range reading at the address of 220
  error2 = start_sensor(220);    //Start the sensor and collect any error codes.
  if (!error2)
  {                  //If you had an error starting the sensor there is little point in reading it as you will get old data.
    delay(100);
    range2 = read_sensor(220);   //reading the sensor will return an integer value -- if this value is 0 there was an error
    Serial.print("Sensor2:");Serial.println(range2);
  }

  //Take a range reading at the address of 218
  error3 = start_sensor(218);    //Start the sensor and collect any error codes.
  if (!error3)
  {                  //If you had an error starting the sensor there is little point in reading it as you will get old data.
    delay(100);
    range3 = read_sensor(218);   //reading the sensor will return an integer value -- if this value is 0 there was an error
    Serial.print("Sensor3:");Serial.println(range3);
  }

  //Take a range reading at the address of 216
  error4 = start_sensor(216);    //Start the sensor and collect any error codes.
  if (!error4)
  {                  //If you had an error starting the sensor there is little point in reading it as you will get old data.
    delay(100);
    range4 = read_sensor(216);   //reading the sensor will return an integer value -- if this value is 0 there was an error
    Serial.print("Sensor4:");Serial.println(range4);
   
  //change_address(224,216);
  }
///Versturen CAN bus bericht met de verschillende ranges in//////////////////////////////////////////
tCAN Ranges;

    Ranges.id = 0xC1;                                 //C1 is de hexadecimale identifier
    Ranges.header.rtr = 0;
    Ranges.header.length = 8;                         //8 decimaal
    Ranges.data[0] = range1;                          //Intel byte order x coor (LSB eerst)
    Ranges.data[1] = range1 >> 8;
    Ranges.data[2] = range2;
    Ranges.data[3] = range2 >> 8;
    Ranges.data[4] = range3;                          //Intel byte order y coordinaat 
    Ranges.data[5] = range3 >> 8;
    Ranges.data[6] = range4;
    Ranges.data[7] = range4 >> 8;

    if (millis() - prevTX1 >= invlTX1)                                      //Interval waarop canbericht 1 verstuurd wordt
    {
    prevTX1 = millis();
    mcp2515_send_message(&Ranges);                                   //Message is een pointer naar een geheugenlocatie waar we de gewenste data kunnen terugvinden
    }  

}



///////////////////////////////////////////////////
// Function: Start a range reading on the sensor //
///////////////////////////////////////////////////
//Uses the I2C library to start a sensor at the given address
//Collects and reports an error bit where: 1 = there was an error or 0 = there was no error.
//INPUTS: byte bit8address = the address of the sensor that we want to command a range reading
//OUPUTS: bit  errorlevel = reports if the function was successful in taking a range reading: 1 = the function
//  had an error, 0 = the function was successful
boolean start_sensor(byte bit8address){
  boolean errorlevel = 0;
  bit8address = bit8address & B11111110;               //Do a bitwise 'and' operation to force the last bit to be zero -- we are writing to the address.
  errorlevel = !i2c_start(bit8address) | errorlevel;   //Run i2c_start(address) while doing so, collect any errors where 1 = there was an error.
  errorlevel = !i2c_write(81) | errorlevel;            //Send the 'take range reading' command. (notice how the library has error = 0 so I had to use "!" (not) to invert the error)
  i2c_stop();
  return errorlevel;
}



///////////////////////////////////////////////////////////////////////
// Function: Read the range from the sensor at the specified address //
///////////////////////////////////////////////////////////////////////
//Uses the I2C library to read a sensor at the given address
//Collects errors and reports an invalid range of "0" if there was a problem.
//INPUTS: byte  bit8address = the address of the sensor to read from
//OUPUTS: int   range = the distance in cm that the sensor reported; if "0" there was a communication error
int read_sensor(byte bit8address){
  boolean errorlevel = 0;
  int range = 0;
  byte range_highbyte = 0;
  byte range_lowbyte = 0;
  bit8address = bit8address | B00000001;  //Do a bitwise 'or' operation to force the last bit to be 'one' -- we are reading from the address.
  errorlevel = !i2c_start(bit8address) | errorlevel;
  range_highbyte = i2c_read(0);           //Read a byte and send an ACK (acknowledge)
  range_lowbyte  = i2c_read(1);           //Read a byte and send a NACK to terminate the transmission
  i2c_stop();
  range = (range_highbyte * 256) + range_lowbyte;  //compile the range integer from the two bytes received.
  if(errorlevel){
    return 0;
  }
  else{
    return range;
  }
}



/////////////////////////////////////////
// Function: Change the sensor address //
/////////////////////////////////////////
//Uses the I2C library to change the address of a sensor at a given address
//Collects and reports an error bit where: 1 = there was an error or 0 = there was no error.
//INPUTS: byte oldaddress = the current address of the sensor that we want to change
//INPUTS: byte newddress  = the address that we want to change the sensor to
//OUPUTS: bit  errorlevel = reports if the function was successful in changing the address: 1 = the function had an
//      error, 0 = the function was successful
boolean change_address(byte oldaddress,byte newaddress){
  //note that the new address will only work as an even number (odd numbers will round down)
  boolean errorlevel = 0;
  oldaddress = oldaddress & B11111110;  //Do a bitwise 'and' operation to force the last bit to be zero -- we are writing to the address.
  errorlevel = !i2c_start(oldaddress) | errorlevel; //Start communication at the new address and track error codes
  errorlevel = !i2c_write(170) | errorlevel;        //Send the unlock code and track the error codes
  errorlevel = !i2c_write(165) | errorlevel;        //Send the unlock code and track the error codes
  errorlevel = !i2c_write(newaddress) | errorlevel; //Send the new address
  i2c_stop();
  return errorlevel;
}
