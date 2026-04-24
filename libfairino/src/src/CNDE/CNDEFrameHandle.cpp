#include "CNDEFrameHandle.h"
#include <iostream>

using namespace std;

std::vector<char> CNDEPkgToFrame(CNDE_PKG& pkg)
{
	vector<char> frame;
    frame.clear();
    frame.push_back(0x5A);
    frame.push_back(0x5A);
    frame.push_back(pkg.count);
    frame.push_back(pkg.type);

    char lenByte[2] = { 0, 0 };
    Int16ToByte(pkg.len, lenByte);

    frame.insert(frame.end(), lenByte, lenByte + 2);
    frame.insert(frame.end(), pkg.data.begin(), pkg.data.end());
    frame.push_back(0xA5);
    frame.push_back(0xA5);

    //for (int i = 0; i < frame.size(); i++)
    //{
    //    printf(" %x", (unsigned char)frame[i]);
    //}

    return frame;
}

int FrameToCNDEPkg(std::vector<char> frame, CNDE_PKG& pkg)
{
    if (frame.size() < 8) 
    {
        std::cout << "error pkg length too small " << std::endl;
        return -1;
    }

    std::vector<char> headBuf = { frame[0], frame[1] };
    std::vector<unsigned char> lenBuf = { (unsigned char)frame[4], (unsigned char)frame[5] };
    std::vector<char> tailBuf = { frame[frame.size() - 2], frame[frame.size() - 1] };

    ByteToInt16(lenBuf.data(), pkg.len);

    if (pkg.len != static_cast<uint16_t>(frame.size() - 8)) 
    {
        std::cout << "error pkg length " << pkg.len << "   " << frame.size() - 8 << "   "
            << static_cast<int>(frame[4]) << "   " << static_cast<int>(frame[5]) << std::endl;
        return -2;
    }

    if (((unsigned char)headBuf[0]) != 0x5A || ((unsigned char)headBuf[1]) != 0x5A)
    {
        std::cout << "error pkg head " << std::endl;
        return -3;
    }

    if (((unsigned char)tailBuf[0]) != 0xA5 || ((unsigned char)tailBuf[1]) != 0xA5)
    {
        std::cout << "error pkg end " << std::endl;
        return -4;
    }

    pkg.type = frame[3];
    pkg.data.clear();

    // 从索引6开始，取pkg.len个元素
    if (frame.size() >= 6 + pkg.len) 
    {
        pkg.data.insert(pkg.data.end(), frame.begin() + 6, frame.begin() + 6 + pkg.len);
    }

    return 0;
}

void Int16ToByte(unsigned short arrInt16, char destByteArr[2]) 
{
    destByteArr[1] = static_cast<unsigned char>((arrInt16 & 0xFF00) >> 8);
    destByteArr[0] = static_cast<unsigned char>(arrInt16 & 0x00FF);
}

void ByteToInt16(const unsigned char arrByte[2], unsigned short& destInt16Arr) 
{
    destInt16Arr = static_cast<unsigned short>((arrByte[1] << 8) + arrByte[0]);
}