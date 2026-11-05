#pragma once
#include <cstdint>
#define OUTPUT 1
#define LOW 0
bool ledcAttachChannel(uint8_t,int,int,int);
bool ledcDetach(uint8_t);
bool ledcWrite(uint8_t,uint32_t);
void pinMode(uint8_t,int);
void digitalWrite(uint8_t,int);
