#ifndef _DETECTOR_H_
#define _DETECTOR_H_

#include "StatPM.h"

// Param Detector

#define _default_KG    (float) (1.395E-3)       // ���������������� [����/[���/���]] �� Cs137
#define _default_BG    0.0                     //  ��� ��������� [���/���] (CPS)

void  Detector_Init_Param(float Set_Kg, float Set_Bg);    // Setting Param Detector
float Detector_GetuZvValue(TStatus_Stat* Stat);           // Get Value [uS/h] ([����/�])
void  RelaysStateProc(URE_GM_Detector* detector); //Set Relays State


/* Example Use */

// �������������
// Stat_Init(1.395E-3,0.0); // Kg = 1.395E-3 - ���������������� ��������� ; Bg=0.0 - ���������� ��� ��������� [CPS]

// �����
// DER = Detector_GetuZvValue(&Status); //   ����� ���:  [uSv/h]

/* End Example Use */


#endif /* _DETECTOR_H_ */
