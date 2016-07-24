
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

#ifndef SEND_ALL
#define SEND_ALL
#endif

/*
#ifndef DEBUG
#define DEBUG
#endif
*/
#define POZYX_POS_ALL 0xFF
#define POZYX_POS_ERR_ALL 0xFE
#define POZYX_QUAT_ALL 0xFD

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[4] = {0x600f, 0x6047, 0x6040, 0x6042};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t heights[4] = {2750, 2000, 1900, 2350};              // anchor z-coordinates in mm
boolean bProcessing = false;                                // set this to true to output data for the processing sketch         

// only required for manual anchor calibration. Please change this to the coordinates measured for the anchors
int32_t anchors_x[4] = {0, 10000, 1000, 9000};              // anchor x-coorindates in mm
int32_t anchors_y[4] = {0, 0, 7000, 8000};                  // anchor y-coordinates in mm

////////////////////////////////////////////////

void setup(){
  Serial.begin(115200);
  
  if(Pozyx.begin() == POZYX_FAILURE){
    delay(100);
    abort();
  }
  
  // clear all previous devices in the device list
  Pozyx.clearDevices();
     
  int status = Pozyx.doAnchorCalibration(POZYX_2_5D, 10, num_anchors, anchors, heights);
  if (status != POZYX_SUCCESS){
    delay(100);
    abort();
  }
  
  // if the automatic anchor calibration is unsuccessful, try manually setting the anchor coordinates.
  // fot this, you must update the arrays anchors_x, anchors_y and heights above
  // comment out the doAnchorCalibration block and the if-statement above if you are using manual mode
  //SetAnchorsManual();

  delay(3000);

}

int packetData32(byte * tx_data, byte address, int32_t * data, int size_of_data, byte packet_type)
{
  uint16_t computed_checksum = 0;
  int index = 0;
  
  tx_data[0] = 'i';
  tx_data[1] = 'n';
  tx_data[2] = 'o';
  tx_data[3] = packet_type;
  tx_data[4] =  address;

  for ( index = 0; index < size_of_data; index++)
  {
    tx_data[5+index*4] = (data[index] >> 24) & 0x000000FF;
    tx_data[6+index*4] = (data[index] >> 16) & 0x000000FF;
    tx_data[7+index*4] = (data[index] >> 8) & 0x000000FF;
    tx_data[8+index*4] = data[index] & 0x000000FF;  
  }
  --index;
  
  for(int i = 0; i < (9+index*4); i++) {
    computed_checksum += tx_data[i];
  }
  tx_data[9+index*4] = (computed_checksum >> 8) & 0x00FF;
  tx_data[10+index*4] = computed_checksum & 0x00FF;

  return 11 + index*4;
}

int packetData16(byte * tx_data, byte address, int16_t * data, int size_of_data, byte packet_type)
{
  uint16_t computed_checksum = 0;
  int index = 0;
  
  tx_data[0] = 'i';
  tx_data[1] = 'n';
  tx_data[2] = 'o';
  tx_data[3] = packet_type;
  tx_data[4] =  address;

  for ( index = 0; index < size_of_data; index++)
  {
    tx_data[5+index*2] = (data[index] >> 8) & 0x00FF;
    tx_data[6+index*2] = data[index] & 0x00FF;  
  }
  --index;
  
  for(int i = 0; i < 7+index*2; i++) {
    computed_checksum += tx_data[i];
  }
  tx_data[7+index*2] = (computed_checksum >> 8) & 0x00FF;
  tx_data[8+index*2] = computed_checksum & 0x00FF;

  return 9 + index*2;
}

void SendSerialData(byte * data, int size_of_data)
{
  for(int i = 0; i < size_of_data; i++) {
    Serial.write(data[i]);
  }
  memset(data, 0, sizeof(data));
  delay(100);
}

void loop(){

  byte tx_data[200];
  int ret = 0;
  coordinates_t position;
  pos_error_t pos_error;
  quaternion_t quat;
  
  int32_t pos_all[3];
  int16_t pos_err_all[6], quat_all[4];

  
  int status = Pozyx.doPositioning(&position, POZYX_2_5D, 1000);
  
  if (status == POZYX_SUCCESS)
  {
    // print out the result
    if(!bProcessing){

      // Retrieve the last error covariance of the position for the device or the remote device.
      Pozyx.getPositionError(&pos_error);
      Pozyx.getQuaternion(&quat);

      #ifdef DEBUG
       Serial.println();
       Serial.print("POS X: ");
       Serial.print(position.x);
       Serial.print(" POS Y: ");
       Serial.print(position.y);
       Serial.print(" POS Z: ");
       Serial.println(position.z);

       Serial.print("POS ERR X: ");
       Serial.print(pos_error.x);
       Serial.print(" POS ERR Y: ");
       Serial.print(pos_error.y);
       Serial.print(" POS ERR Z: ");
       Serial.print(pos_error.z);
       Serial.print(" POS ERR XY: ");
       Serial.print(pos_error.xy);
       Serial.print(" POS ERR XZ: ");
       Serial.print(pos_error.xz);
       Serial.print(" POS ERR YZ: ");
       Serial.println(pos_error.yz);

       Serial.print("QUAT X: ");
       Serial.print(quat.x * 1000);
       Serial.print(" QUAT Y: ");
       Serial.print(quat.y * 1000);
       Serial.print(" QUAT Z: ");
       Serial.print(quat.z * 1000);
       Serial.print(" QUAT W: ");
       Serial.println(quat.weight * 1000);

      #endif
      
      #ifndef SEND_ALL
      // pos X
      ret = packetData32(tx_data, POZYX_POS_X, & position.x, 1, 0xA0);
      SendSerialData(tx_data, ret);

      // pos Y
      ret = packetData32(tx_data, POZYX_POS_Y, & position.y, 1, 0xA0);
      SendSerialData(tx_data, ret);
      
      // pos Z
      ret = packetData32(tx_data, POZYX_POS_Z, & position.z, 1, 0xA0);
      SendSerialData(tx_data, ret);
      #else
      pos_all[0] = position.x;
      pos_all[1] = position.y;
      pos_all[2] = position.z;      

      ret = packetData32(tx_data, POZYX_POS_ALL, pos_all, 3, 0x9C);
      SendSerialData(tx_data, ret);
      
      #endif

      #ifdef SEND_ALL
      quat_all[0] = quat.x * 1000;
      quat_all[1] = quat.y * 1000;
      quat_all[2] = quat.z * 1000;
      quat_all[3] = quat.weight * 1000;
      
      ret = packetData16(tx_data, POZYX_QUAT_ALL, quat_all, 4, 0x8E);
      SendSerialData(tx_data, ret);
      #endif
      
      #ifndef SEND_ALL
      // POZYX_POS_ERR_X
      ret = packetData16(tx_data, POZYX_POS_ERR_X, & pos_error.x, 1, 0x8A);
      SendSerialData(tx_data, ret);

      // POZYX_POS_ERR_Y
      ret = packetData16(tx_data, POZYX_POS_ERR_Y, & pos_error.y, 1, 0x8A);
      SendSerialData(tx_data, ret);

      // POZYX_POS_ERR_Z
      ret = packetData16(tx_data, POZYX_POS_ERR_Z, & pos_error.z, 1, 0x8A);
      SendSerialData(tx_data, ret);

      // POZYX_POS_ERR_XY
      ret = packetData16(tx_data, POZYX_POS_ERR_XY, & pos_error.xy, 1, 0x8A);
      SendSerialData(tx_data, ret);
      
      // POZYX_POS_ERR_XZ
      ret = packetData16(tx_data, POZYX_POS_ERR_XZ, & pos_error.xz, 1, 0x8A);
      SendSerialData(tx_data, ret);
      
      // POZYX_POS_ERR_YZ
      ret = packetData16(tx_data, POZYX_POS_ERR_YZ, & pos_error.yz, 1, 0x8A);
      SendSerialData(tx_data, ret);

      #else
      pos_err_all[0] = pos_error.x;
      pos_err_all[1] = pos_error.y;
      pos_err_all[2] = pos_error.z;
      pos_err_all[3] = pos_error.xy;
      pos_err_all[4] = pos_error.xz;
      pos_err_all[5] = pos_error.yz;

      ret = packetData16(tx_data, POZYX_POS_ERR_ALL, pos_err_all, 6, 0x9C);
      SendSerialData(tx_data, ret);

      #endif
 
    }
    
  }
}

// function to manually set the anchor coordinates
void SetAnchorsManual(){
 
 int i=0;
 for(i=0; i<num_anchors; i++){
   device_coordinates_t anchor;
   anchor.network_id = anchors[i];
   anchor.flag = 0x1; 
   anchor.pos.x = anchors_x[i];
   anchor.pos.y = anchors_y[i];
   anchor.pos.z = heights[i];
   Pozyx.addDevice(anchor);
 }
 
}
