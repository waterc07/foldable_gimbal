#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "FreeRTOS.h"
#include "drv_uart.h"
#include "std_typedef.h"
#include "task.h"
#include "usart.h"

extern "C" __weak void UART6RxCallback(uint8_t *pRxData, uint16_t rxDataLength);

using VofaUart6RxHandler = void (*)(uint8_t *pRxData, uint16_t rxDataLength);

void Vofa_SetUART6RxHandler(VofaUart6RxHandler handler);

template <size_t N>
class Vofa
{
public:
    static constexpr size_t PARAMETER_MAX_COUNT = 32;
    using CallbackFunction                      = fp32 (*)();

    class Frame
    {
    private:
        static constexpr size_t DATA_FRAME_SIZE = N;
        fp32 m_data[DATA_FRAME_SIZE + 1]        = {};
        uint16_t m_pos                          = 0;

    public:
        bool write(fp32 data)
        {
            if (m_pos >= DATA_FRAME_SIZE) {
                return false;
            }

            m_data[m_pos++] = data;
            return true;
        }

        const fp32 *getData()
        {
            uint8_t *tail = reinterpret_cast<uint8_t *>(&m_data[m_pos]);
            tail[0]       = 0x00;
            tail[1]       = 0x00;
            tail[2]       = 0x80;
            tail[3]       = 0x7F;
            return m_data;
        }

        uint16_t getSize() const
        {
            return m_pos * sizeof(fp32) + 4;
        }

        bool isEmpty() const
        {
            return m_pos == 0;
        }

        void reset()
        {
            m_pos = 0;
        }
    };

private:
    struct Parameter {
        const char *name;
        fp32 value;
        void (*callback)(fp32 *newValue);
    };

    Frame m_frame;
    CallbackFunction m_functionPointers[N]      = {};
    size_t m_functionCount                      = 0;
    Parameter m_parameters[PARAMETER_MAX_COUNT] = {};
    size_t m_parameterCount                     = 0;

    static Vofa<N> *&getActiveInstance()
    {
        static Vofa<N> *instance = nullptr;
        return instance;
    }

    static char *skipLeadingSpace(char *str)
    {
        while (*str == ' ' || *str == '\t') {
            ++str;
        }
        return str;
    }

    static void trimTrailingSpace(char *str)
    {
        size_t len = std::strlen(str);
        while (len > 0) {
            const char ch = str[len - 1];
            if (ch != ' ' && ch != '\t') {
                break;
            }

            str[len - 1] = '\0';
            --len;
        }
    }

    void handleCommandLine(char *line)
    {
        line = skipLeadingSpace(line);
        trimTrailingSpace(line);
        if (*line == '\0') {
            return;
        }

        char *separator = std::strchr(line, ':');
        if (separator == nullptr) {
            return;
        }

        *separator     = '\0';
        char *namePart = skipLeadingSpace(line);
        trimTrailingSpace(namePart);

        char *valueStr = skipLeadingSpace(separator + 1);
        trimTrailingSpace(valueStr);
        if (*namePart == '\0' || *valueStr == '\0') {
            return;
        }

        char *endPtr   = nullptr;
        float newValue = std::strtof(valueStr, &endPtr);
        if (endPtr == valueStr) {
            return;
        }

        endPtr = skipLeadingSpace(endPtr);
        if (*endPtr != '\0') {
            return;
        }

        for (size_t i = 0; i < m_parameterCount; ++i) {
            if (m_parameters[i].name == nullptr) {
                continue;
            }

            if (std::strcmp(m_parameters[i].name, namePart) == 0) {
                m_parameters[i].value = static_cast<fp32>(newValue);
                if (m_parameters[i].callback != nullptr) {
                    m_parameters[i].callback(&m_parameters[i].value);
                }
                break;
            }
        }
    }

    void handleRxData(uint8_t *pRxData, uint16_t rxDataLength)
    {
        if (pRxData == nullptr || rxDataLength == 0) {
            return;
        }

        constexpr size_t RX_CMD_BUFFER_SIZE = 128;
        char buffer[RX_CMD_BUFFER_SIZE]     = {};

        size_t copyLen = rxDataLength;
        if (copyLen >= RX_CMD_BUFFER_SIZE) {
            copyLen = RX_CMD_BUFFER_SIZE - 1;
        }

        std::memcpy(buffer, pRxData, copyLen);
        buffer[copyLen] = '\0';

        char *cursor = buffer;
        while (*cursor != '\0') {
            while (*cursor == '\r' || *cursor == '\n') {
                ++cursor;
            }
            if (*cursor == '\0') {
                break;
            }

            char *lineEnd = cursor;
            while (*lineEnd != '\0' && *lineEnd != '\r' && *lineEnd != '\n') {
                ++lineEnd;
            }

            const char original = *lineEnd;
            *lineEnd            = '\0';
            handleCommandLine(cursor);

            if (original == '\0') {
                break;
            }

            cursor = lineEnd + 1;
        }
    }

    static void parameterRxCallback(uint8_t *pRxData, uint16_t rxDataLength)
    {
        Vofa<N> *instance = getActiveInstance();
        if (instance != nullptr) {
            instance->handleRxData(pRxData, rxDataLength);
        }
    }

public:
    bool BindFunction(CallbackFunction func)
    {
        if (m_functionCount >= N) {
            return false;
        }

        m_functionPointers[m_functionCount++] = func;
        return true;
    }

    bool UnbindFunction(CallbackFunction func)
    {
        for (size_t i = 0; i < m_functionCount; ++i) {
            if (m_functionPointers[i] != func) {
                continue;
            }

            for (size_t j = i; j + 1 < m_functionCount; ++j) {
                m_functionPointers[j] = m_functionPointers[j + 1];
            }
            m_functionPointers[m_functionCount - 1] = nullptr;
            --m_functionCount;
            return true;
        }

        return false;
    }

    void ExecuteFunctions()
    {
        for (size_t i = 0; i < m_functionCount; ++i) {
            if (m_functionPointers[i] == nullptr) {
                continue;
            }

            m_frame.write(m_functionPointers[i]());
        }
    }

    void writeData(fp32 data)
    {
        m_frame.write(data);
    }

    void sendFrame()
    {
        sendFrame(m_frame);
    }

    void sendFrame(Frame &frame)
    {
        if (frame.isEmpty()) {
            return;
        }

        HAL_UART_Transmit_DMA(&huart6, reinterpret_cast<const uint8_t *>(frame.getData()), frame.getSize());
        frame.reset();
    }

    void Init()
    {
        getActiveInstance() = this;
        Vofa_SetUART6RxHandler(parameterRxCallback);

        if (huart6.gState == HAL_UART_STATE_RESET) {
            MX_USART6_UART_Init();
        }

        UART_Init(&huart6, UART6RxCallback, 128);
    }

    void AddParameterListener(const char *parameterName, void (*callback)(fp32 *newValue))
    {
        if (parameterName == nullptr || callback == nullptr) {
            return;
        }

        for (size_t i = 0; i < m_parameterCount; ++i) {
            if (m_parameters[i].name != nullptr && std::strcmp(m_parameters[i].name, parameterName) == 0) {
                m_parameters[i].callback = callback;
                return;
            }
        }

        if (m_parameterCount < PARAMETER_MAX_COUNT) {
            m_parameters[m_parameterCount++] = {parameterName, 0.0f, callback};
        }
    }
};
