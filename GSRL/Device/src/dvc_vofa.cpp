#include "dvc_vofa.hpp"

static VofaUart6RxHandler s_vofaUart6RxHandler = nullptr;

void Vofa_SetUART6RxHandler(VofaUart6RxHandler handler)
{
    s_vofaUart6RxHandler = handler;
}

extern "C" __weak void UART6RxCallback(uint8_t *pRxData, uint16_t rxDataLength)
{
    if (s_vofaUart6RxHandler != nullptr) {
        s_vofaUart6RxHandler(pRxData, rxDataLength);
    }
}
