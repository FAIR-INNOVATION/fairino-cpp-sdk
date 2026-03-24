#include "FrameHandle.h"

using namespace std;

std::vector<std::string> SplitFrame(const std::string& data) {
    std::vector<std::string> result;
    size_t pos = 0;

    while (pos < data.length()) {
        size_t start = data.find("/f/b", pos);
        if (start == std::string::npos) break;

        size_t end = data.find("/b/f", start);
        if (end == std::string::npos) break;

        // 提取完整帧
        result.push_back(data.substr(start, end + 4 - start));
        pos = end + 4;
    }

    return result;
}

FRAME UnpacketFrame(std::string frameStr)
{
    FRAME frame;

    // 1. 基本长度检查
    if (frameStr.length() < 27)
    {
        return frame;
    }

    // 2. 验证帧头帧尾
    if (frameStr.substr(0, 4) != "/f/b")
    {
        return frame;
    }

    if (frameStr.substr(frameStr.length() - 4) != "/b/f")
    {
        return frame;
    }

    // 3. 分割字符串
    std::vector<std::string> parts;
    std::string data = frameStr.substr(4, frameStr.length() - 8);

    size_t start = 0;
    for (int i = 0; i < 5; i++)
    {  // 分割前4个字段
        size_t pos = data.find("III", start);
        if (pos == std::string::npos) return frame;  // 格式错误

        parts.push_back(data.substr(start, pos - start));
        start = pos + 3;
    }
    // 4. 最后一部分是content
    if (start < data.length())
    {
        parts.push_back(data.substr(start));
    }
    else
    {
        parts.push_back("");
    }
    // 5. 填充并验证数据
    frame.head = "/f/b";
    frame.tail = "/b/f";

    try
    {
        frame.count = std::stoi(parts[1]);
        frame.cmdID = std::stoi(parts[2]);
        frame.contentLen = std::stoi(parts[3]);
    }
    catch (...)
    {
        return frame;  // 数据转换错误
    }

    frame.content = parts[4];

    // 6. 验证内容长度
    if (frame.contentLen > 0 && frame.content.length() != (size_t)frame.contentLen)
    {
        frame.content = "";
        frame.contentLen = 0;
    }

    return frame;
}

// 获取lua程序500错误行号和错误码
void GetRobotLUAProgram500ErrCode(const std::string& content, int& errLinNum, int& luaErrCode)
{
    // 检查是否是lua错误
    size_t luaPos = content.find(".lua");
    if (luaPos == std::string::npos) return;

    // 找第一个冒号（文件名后的冒号）
    size_t colon1 = content.find(':', luaPos);
    if (colon1 == std::string::npos) return;

    // 找第二个冒号（行号后的冒号）
    size_t colon2 = content.find(':', colon1 + 1);
    if (colon2 == std::string::npos) return;

    // 提取行号
    std::string lineStr = content.substr(colon1 + 1, colon2 - colon1 - 1);
    errLinNum = std::stoi(lineStr);

    // 找错误码
    size_t errcodePos = content.find("errcode", colon2);
    if (errcodePos == std::string::npos) return;

    // 提取错误码数字
    size_t codeStart = content.find_first_of("0123456789", errcodePos + 7);
    std::string codeStr;
    for (size_t i = codeStart; i < content.length() && isdigit(content[i]); i++) {
        codeStr.push_back(content[i]);
    }
    luaErrCode = std::stoi(codeStr);
}

std::string PackFrame(FRAME frame)
{
    string frameStr = "";
    frameStr += "/f/bIII";
    frameStr += to_string(frame.count);
    frameStr += "III";
    frameStr += to_string(frame.cmdID);
    frameStr += "III";
    frameStr += to_string(frame.content.length());
    frameStr += "III";
    frameStr += frame.content;
    frameStr += "III/b/f";

    return frameStr;
}

bool VerifyFrame(const std::string& frameStr)
{
    if (frameStr.length() < 20 ||
        frameStr.substr(0, 4) != "/f/b" ||
        frameStr.substr(frameStr.length() - 4) != "/b/f")
    {
        return false;
    }

    std::string data = frameStr.substr(4, frameStr.length() - 8);

    std::vector<std::string> parts;
    size_t start = 0;
    size_t pos;

    for (int i = 0; i < 5; i++)
    {
        pos = data.find("III", start);
        if (pos == std::string::npos)
        {
            return false;
        }
        parts.push_back(data.substr(start, pos - start));
        start = pos + 3;
    }
    parts.push_back(data.substr(start)); 

    try
    {
        return parts.size() == 6 &&                   
            !parts[1].empty() && !parts[2].empty() && !parts[3].empty() && !parts[4].empty() &&
            std::stoi(parts[3]) == (int)parts[4].length(); 
    }
    catch (...)
    {
        return false;
    }
}
