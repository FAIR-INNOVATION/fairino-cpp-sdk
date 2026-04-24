#pragma once

#include "stdint.h"
#include <vector>

#define CNDE_FRAME_TYPE_OUTPUT_CONFIG 1
#define CNDE_FRAME_TYPE_START 2
#define CNDE_FRAME_TYPE_STOP 3
#define CNDE_FRAME_TYPE_OUTPUT_STATE 4
#define CNDE_FRAME_TYPE_MESSAGE 6

typedef struct _CNDE_PKG
{
    uint16_t head = 0x5A5A;
    uint8_t count = 0;
    uint8_t type = 0;
    uint16_t len = 0;
    std::vector<char> data;
    uint16_t end = 0xA5A5;

    void Clear()
    {
        count = 0;
        type = 0;
        len = 0;
        data.clear();
    }
}CNDE_PKG;

std::vector<char> CNDEPkgToFrame(CNDE_PKG& pkg);
int FrameToCNDEPkg(std::vector<char> frame, CNDE_PKG& pkg);
void Int16ToByte(unsigned short arrInt16, char destByteArr[2]);
void ByteToInt16(const unsigned char arrByte[2], unsigned short& destInt16Arr);



   