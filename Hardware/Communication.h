#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H
#include "Measure.h"
void Bluetooth_transmission1(void);
void Bluetooth_transmission2(void);
void Data_Send_F1(int *pst, unsigned char len);
void Data_Send_F2(int *pst, unsigned char len);
void communication_can_send();
#endif
