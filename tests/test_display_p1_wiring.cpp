#include "../firmware/common/src/BUSA_DisplayProtocol.h"
#include "../firmware/BUSA_Display_N4R8/BUSA_GuitionFactoryInit.h"
#include <cassert>
int main() {
    // Owner-approved cable: yellow P1 TXD -> main IO8; blue main IO9 -> P1 RXD.
    assert(busa_uart::mainRx == 8 && busa_uart::mainTx == 9);
    assert(busa_uart::displayRx == 44 && busa_uart::displayTx == 43);
    assert(busa_uart::baud == 115200);
    assert(busa_guition::MAIN_RX == busa_uart::mainRx && busa_guition::MAIN_TX == busa_uart::mainTx);
    assert(busa_guition::DISPLAY_RX == busa_uart::displayRx && busa_guition::DISPLAY_TX == busa_uart::displayTx);
}
