#ifndef NOS_TYPEDEFS
#define NOS_TYPEDEFS
#include <stdint.h>
#include <stdbool.h>
#include "StatPM.h"
#include "stm32f1xx.h"
/* Unions begin */

typedef union Short_t //uint16_t data wrap
{ 
    uint8_t bytes[2];
    uint16_t data;
} NOS_Short;

typedef union Float_t //float data wrap
{
    uint8_t bytes[4];
    float data;
} NOS_Float;



/* Unions end */

/* Enums begin */

typedef enum ModBusState_t{Free,ReceiveFromMaster,ReceiveFromSlave,TransmitToMaster,TransmitToSlave} ModBusState;

/* Enums end */

/* Structures begin */

/*WS2812B Matrix*/
typedef struct PixelColor_t
{
   uint8_t R;
   uint8_t G;
   uint8_t B;
} PixelColor;

typedef struct MatrixSize_t
{
  uint8_t col;
  uint8_t row;
} MatrixSize;

typedef struct  Symvol_t
{
  uint8_t data[5];
} Symvol;


typedef struct WS2812B_Matrix_t
{
    uint8_t* buffer;
    NOS_Short ledsCount;
    MatrixSize* size;
    PixelColor* textColor;
    PixelColor* foneColor;
    Symvol* symvols;
} WS2812B_Matrix;

/*WS2812B Matrix*/

/*ModBus*/

typedef struct ModBus_Master_Command_t
{
    uint8_t type; 
    uint8_t Addr;
    uint8_t Command;
    uint8_t Byte_Count;
    uint16_t Reg_Addr;
    uint16_t Reg_Count;
    NOS_Short ShortValue;
    NOS_Float FloatValue;
} ModBus_Master_Command;

typedef struct ModBus_Slave_Command_t
{
    uint8_t type;
    uint8_t Addr;
    uint8_t Command;
    uint8_t Byte_Count;
    NOS_Short ShortValue;
    NOS_Float FloatValue;
} ModBus_Slave_Command;

typedef struct Pin_T
{
    uint16_t* Port;
    uint16_t Pin; 
}Pin;


typedef struct URE_GM_Detector_t
{
  /* ModBus Values */
  float uSvValue;
  float temperature;
  float firstDangerValue;
  float secondDangerValue;
  uint16_t voltage;
  uint8_t address;
  /* ModBus Values */

  /* System Info */
  uint16_t tickCount1;
  uint16_t tickCount2;
  uint16_t systemCounter;
  TStatus_Stat* status;
  /* System Info */

  /* Pins Data */
  Pin normalRelay;
  Pin firstDangerRelay;
  Pin secondDangerRelay;
  /* Pins Data */

} URE_GM_Detector;


/*ModBus*/

/* Structures end */
#endif