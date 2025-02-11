/*
 * loadcell.h
 *
 *  Created on: Aug 22, 2024
 *      Author: Hritik Kumar
 */

#ifndef LOADCELL_H_
#define LOADCELL_H_

/* Load Cell */
void HX711_Init(void);
uint32_t HX711_Read(void);
void microDelay(uint16_t us);
int32_t getHX711(void);
int weigh(void);
void setTare(void);

#endif /* LOADCELL_H_ */
