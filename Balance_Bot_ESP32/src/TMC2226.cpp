#include "TMC2226.h"

/// @brief Sends a frame to a TMC2226
/// @param frame
/// @param length
/// @param adress
void TMC_send_frame(uint8_t *frame)
{
    // Send a wait until the end of sending
    Serial2.write(frame, D_TMC_FRAME_LENGTH);
    Serial2.flush();
}

/// @brief Pulses the step pin
void TMC_step()
{
    digitalWrite(D_TMC_PIN_STEP, HIGH);
    delayMicroseconds(500);
    digitalWrite(D_TMC_PIN_STEP, LOW);
}

/// @brief Initializes the TMC, (initializes UART protocol)
/// @param baud_rate
/// @param tx_pin
/// @param rx_pin
void TMC_init(uint32_t baud_rate, uint8_t tx_pin, uint8_t rx_pin)
{
    // Set pins
    pinMode(D_TMC_PIN_STEP, OUTPUT);
    // Start UART
    Serial2.begin(9600, SERIAL_8N1, rx_pin, tx_pin);
    // Send config (use uart pin)
    U_TMC_GCONF gconf;
    // Set all to 0
    gconf.bytes[0] = 0x00;
    gconf.bytes[1] = 0x00;
    // Disable pdn
    gconf.bits.pdn_disable = 1;
    // Write config to TMC
    TMC_write_to_register(E_TMC_REG_GCONF, gconf.bytes);
}

/// @brief Computes the CRC, last byte of transmission.
/// @param frame
/// @param framelength
void calculate_crc(uint8_t *frame)
{
    int i, j;
    uint8_t crc = 0;
    uint8_t current_byte;

    for (i = 0; i < (D_TMC_FRAME_LENGTH - 1); i++)
    {
        current_byte = frame[i];
        for (j = 0; j < 8; j++)
        {
            if ((crc >> 7) ^ (current_byte & 0x01))
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc = (crc << 1);
            }
            current_byte = current_byte >> 1;
        }
    }
    frame[D_TMC_FRAME_LENGTH - 1] = crc;
}

/// @brief Writes data to a specified register
/// @param reg_address
/// @param data
/// @param length
void TMC_write_to_register(E_TMC_REG reg_address, uint8_t *data)
{
    // Prepare frame
    uint8_t frame[D_TMC_FRAME_LENGTH]; // Add space for init, chip adress, CRC

    frame[0] = D_TMC_INIT_BYTE;    // Init
    frame[1] = D_TMC_NODE_ADDRESS; // Chip address
    frame[2] = reg_address | 0x80; // Reg address + W

    uint32_t speed = 100;
    for (uint8_t i = 0; i < 4; i++)
    {
        frame[i + 3] = data[3 - i]; // MSB first
    }
    calculate_crc(frame);
    // Send frame
    TMC_send_frame(frame);
}
