#include "Detector.h"
#
static float Detector_Kg =_default_KG;
static float Detector_Bg =_default_BG;

// Setting Param Detector
void  Detector_Init_Param(float Set_Kg, float Set_Bg) {
 Detector_Kg = Set_Kg;
 Detector_Bg = Set_Bg;
}

// Get Value [uSv/h]
float Detector_GetuZvValue(TStatus_Stat* Stat) {
  float DER;
  DER  = (Stat->CPS>Detector_Bg) ? (Stat->CPS-Detector_Bg) : (Stat->CPS);
  DER *= Detector_Kg;
  return DER; // [uSv/h]
}

void Detector_Init(URE_GM_Detector* detector,float firstDg,float secondDg,Pin norm,Pin firstRl,Pin secondRl)
{
  detector->firstDangerValue = firstDg;
  detector->secondDangerValue = secondDg;
  detector->normalRelay = norm;
  detector->firstDangerRelay = firstRl;
  detector->secondDangerRelay = secondRl;
}

void RelaysStateProc(URE_GM_Detector* detector)
{
  if(detector->uSvValue <= detector->firstDangerValue)
  {
    HAL_GPIO_WritePin(detector->normalRelay.Port,detector->normalRelay.Pin,1);
    HAL_GPIO_WritePin(detector->firstDangerRelay.Port,detector->firstDangerRelay.Pin,0);
    HAL_GPIO_WritePin(detector->secondDangerRelay.Port,detector->secondDangerRelay.Pin,0);
  }
  else if(detector->uSvValue >= detector->firstDangerValue && detector->uSvValue < detector->secondDangerValue)
  {
    HAL_GPIO_WritePin(detector->normalRelay.Port,detector->normalRelay.Pin,0);
    HAL_GPIO_WritePin(detector->firstDangerRelay.Port,detector->firstDangerRelay.Pin,1);
    HAL_GPIO_WritePin(detector->secondDangerRelay.Port,detector->secondDangerRelay.Pin,0);
  }
  else 
  {
    HAL_GPIO_WritePin(detector->normalRelay.Port,detector->normalRelay.Pin,0);
    HAL_GPIO_WritePin(detector->firstDangerRelay.Port,detector->firstDangerRelay.Pin,0);
    HAL_GPIO_WritePin(detector->secondDangerRelay.Port,detector->secondDangerRelay.Pin,1);
  }
}





