#ifndef OLED_H
#define OLED_H

#include <Arduino.h>

void initOLED();
void oledSetContrast(uint8_t v);
void displayOverheat();
void displayScreen1_TempHumid();
void displayScreen2_PressureAlt();
void displayScreen3_GasIAQ();
void displayScreen4_Uptime();
void displayErrorScreen();
void updateOLEDDisplayContent();

#endif
