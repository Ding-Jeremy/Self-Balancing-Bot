#include "TMC2226.h"

/// @brief Sends a frame to a TMC2226
/// @param frame
/// @param length
/// @param adress
void TMC_send_frame(uint8_t *frame, uint8_t length)
{
    // Send a wait until the end of sending
    Serial2.write(frame, length);
    Serial2.flush();
}

void TMC_init(uint32_t baud_rate, uint8_t tx_pin, uint8_t rx_pin)
{
    Serial2.begin(9600, SERIAL_8N1, rx_pin, tx_pin);
}