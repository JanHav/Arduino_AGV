// Please read the ready-to-localize tuturial together with this example.
// https://www.pozyx.io/Documentation/Tutorials/ready_to_localize
/**
  The Pozyx ready to localize tutorial (c) Pozyx Labs
  Please read the tutorial that accompanies this sketch: https://www.pozyx.io/Documentation/Tutorials/ready_to_localize/Arduino
  This tutorial requires at least the contents of the Pozyx Ready to Localize kit. It demonstrates the positioning capabilities
  of the Pozyx device both locally and remotely. Follow the steps to correctly set up your environment in the link, change the
  parameters and upload this sketch. Watch the coordinates change as you move your device around!
*/

///BenodigdeLibrariesPozyx///////////////////////////////////////////////////////////////////
#include <SPI.h>
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
///BenodigdeLibrariesCanBus//////////////////////////////////////////////////////////////////
#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>

///VariabelenPozyx///////////////////////////////////////////////////////////////////////////
uint16_t remote_id = 0x6965;                            // set this to the ID of the remote device
bool remote = false;                                    // set this to true to use the remote ID

boolean use_processing = false;                         // set this to true to output data for the processing sketch

const uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[num_anchors] = {0x6971, 0x602a, 0x6042, 0x6026};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[num_anchors] = {1635,1195,7080,6970};               // anchor x-coorindates in mm
int32_t anchors_y[num_anchors] = {650,11130,11610,760};                  // anchor y-coordinates in mm
int32_t heights[num_anchors] = {871,1760,470,1300};              // anchor z-coordinates in mm

  //Nodige variabelen om een filtering in te schakelen
uint8_t filter_type = FILTER_TYPE_MOVINGMEDIAN;                //Klassiek signaal filteren (keuze uit low pass FIR filter, moving average and moving median) 
uint8_t filter_strength = 12;                                   //Filter strengt van minimaal 3 nemen (hoe hoger hoe trager je coordinaten verkrijgt)

  //Nodige variabelen om het position algoritme vast te leggen
uint8_t algorithm =  POZYX_POS_ALG_TRACKING  ;             // positioning algorithm to use. try  POZYX_POS_ALG_TRACKING for fast moving objects. POZYX_POS_ALG_UWB_ONLY
uint8_t dimension = POZYX_3D;                           // positioning dimension
int32_t height = 800;                                  // height of device, required in 2.5D positioning
///VariabelenCanBus//////////////////////////////////////////////////////////////////////////
int TestCanWaarde = 8888;                                         // Data die we verzenden in D0 en D1 (intel byte order) en D2 en D3 (motorola byte order)
int dataSonar = 15900;                                            // Bitwise operations kan je enkel uitvoeren op een int datatype
unsigned long prevTX1 = 0;                                        // Variabele om laatste tijd te onthouden
const unsigned int invlTX1 = 10;                                  // Interval tussen de CAN-berichten
unsigned long prevTX2 = 0;                                        // Variabele om laatste tijd te onthouden
const unsigned int invlTX2 = 100;                                 // Interval tussen de CAN-berichten

///SetupLoop/////////////////////////////////////////////////////////////////////////////////
void setup(){
  Serial.begin(115200);

///SetupPozyx////////////////////////////////////////////////////////////////////////////////
    if(Pozyx.begin() == POZYX_FAILURE)
    {
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
    }
    if(!remote){
    remote_id = NULL;
    }

    Serial.println(F("----------POZYX POSITIONING V1.1----------"));
    Serial.println(F("NOTES:"));
    Serial.println(F("- No parameters required."));
    Serial.println();
    Serial.println(F("- System will auto start anchor configuration"));
    Serial.println();
    Serial.println(F("- System will auto start positioning"));
    Serial.println(F("----------POZYX POSITIONING V1.1----------"));
    Serial.println();
    Serial.println(F("Performing manual anchor configuration:"));

    // clear all previous devices in the device list
    Pozyx.clearDevices(remote_id);
    // sets the anchor manually
    setAnchorsManual();
    //Filter instellen voor de UWB coördinaten
    Pozyx.setPositionFilter(filter_type,filter_strength,remote_id);
    // sets the positioning algorithm
    Pozyx.setPositionAlgorithm(algorithm, dimension, remote_id);

    printCalibrationResult();
    delay(2000);

    Serial.println(F("Starting positioning: "));
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
void loop(){
  coordinates_t position;
  int status;
  if(remote){
    status = Pozyx.doRemotePositioning(remote_id, &position, dimension, height, algorithm);
  }else{
    status = Pozyx.doPositioning(&position, dimension, height, algorithm);
  }

  if (status == POZYX_SUCCESS){
    // prints out the result
    printCoordinates(position);
  }else{
    // prints out the error code
    printErrorCode("positioning");
  } 
}

///Prints the coordinates for either humans or for processing + CAN bericht//////////////////
void printCoordinates(coordinates_t coor){
  uint16_t network_id = remote_id;
  if (network_id == NULL){
    network_id = 0;
  }
  if(!use_processing){
    //Serial.print("POS ID 0x");
    //Serial.print(network_id, HEX);
    //Serial.print(", x(mm): ");
    Serial.print(coor.x);
    Serial.print(",");
    //Serial.print(", y(mm): ");
    Serial.print(coor.y);
    Serial.println(",");
    //Serial.print(", z(mm): ");
    //Serial.println(coor.z);
  }else{
    Serial.print("POS,0x");
    Serial.print(network_id,HEX);
    Serial.print(",");
    Serial.print(coor.x);
    Serial.print(",");
    Serial.print(coor.y);
    Serial.print(",");
    Serial.println(coor.z);
  }
///XcoordinaatYcoordinaat in CANbericht plaatsen/////////////////////////////////////////////
tCAN XYcoor;

    XYcoor.id = 0xA1;                                 //A1 is de hexadecimale identifier
    XYcoor.header.rtr = 0;
    XYcoor.header.length = 8;                         //8 decimaal
    XYcoor.data[0] = coor.x;                          //Intel byte order x coor (MSB eerst)
    XYcoor.data[1] = coor.x >> 8;
    XYcoor.data[2] = coor.x >> 16;
    XYcoor.data[3] = coor.x >> 24;
    XYcoor.data[4] = coor.y;                          //Intel byte order y coordinaat 
    XYcoor.data[5] = coor.y >> 8;
    XYcoor.data[6] = coor.y >> 16;
    XYcoor.data[7] = coor.y >> 24;

    if (millis() - prevTX1 >= invlTX1)                                      //Interval waarop canbericht 1 verstuurd wordt
    {
    prevTX1 = millis();
    mcp2515_send_message(&XYcoor);                                   //Message is een pointer naar een geheugenlocatie waar we de gewenste data kunnen terugvinden
    }  
}

// error printing function for debugging
void printErrorCode(String operation){
  uint8_t error_code;
  if (remote_id == NULL){
    Pozyx.getErrorCode(&error_code);
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(", local error code: 0x");
    Serial.println(error_code, HEX);
    return;
  }
  int status = Pozyx.getErrorCode(&error_code, remote_id);
  if(status == POZYX_SUCCESS){
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(" on ID 0x");
    Serial.print(remote_id, HEX);
    Serial.print(", error code: 0x");
    Serial.println(error_code, HEX);
  }else{
    Pozyx.getErrorCode(&error_code);
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(", couldn't retrieve remote error code, local error: 0x");
    Serial.println(error_code, HEX);
  }
}

// print out the anchor coordinates (also required for the processing sketch)
void printCalibrationResult(){
  uint8_t list_size;
  int status;

  status = Pozyx.getDeviceListSize(&list_size, remote_id);
  Serial.print("list size: ");
  Serial.println(status*list_size);

  if(list_size == 0){
    printErrorCode("configuration");
    return;
  }

  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids, list_size, remote_id);

  Serial.println(F("Calibration result:"));
  Serial.print(F("Anchors found: "));
  Serial.println(list_size);

  coordinates_t anchor_coor;
  for(int i = 0; i < list_size; i++)
  {
    Serial.print("ANCHOR,");
    Serial.print("0x");
    Serial.print(device_ids[i], HEX);
    Serial.print(",");
    Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor, remote_id);
    Serial.print(anchor_coor.x);
    Serial.print(",");
    Serial.print(anchor_coor.y);
    Serial.print(",");
    Serial.println(anchor_coor.z);
  }
}

// function to manually set the anchor coordinates
void setAnchorsManual(){
  for(int i = 0; i < num_anchors; i++){
    device_coordinates_t anchor;
    anchor.network_id = anchors[i];
    anchor.flag = 0x1;
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = heights[i];
    Pozyx.addDevice(anchor, remote_id);
  }
  if (num_anchors > 4){
    Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, num_anchors, remote_id);
  }
}
