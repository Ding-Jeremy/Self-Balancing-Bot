#include "TMC2226.h"

/// @brief Sends a frame to a TMC2226
/// @param frame
/// @param length
/// @param adress
void TMC_send_frame(uint8_t *frame, uint8_t frame_length)
{
    // Send a wait until the end of sending
    uart_flush_input(UART_NUM_2); // Flush input
    uart_write_bytes(UART_NUM_2, frame, frame_length);
    uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(10));
    vTaskDelay(pdMS_TO_TICKS(1));
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
    pinMode(D_TMC_PIN_STEP, OUTPUT);

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    esp_err_t err;

    err = uart_param_config(UART_NUM_2, &uart_config);
    printf("param_config = %s\n", esp_err_to_name(err));

    err = uart_set_pin(UART_NUM_2, GPIO_NUM_25, GPIO_NUM_26, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    printf("set_pin = %s\n", esp_err_to_name(err));

    err = uart_driver_install(UART_NUM_2, 256, 256, 0, NULL, 0);
    printf("driver_install = %s\n", esp_err_to_name(err));
    // Send config (use uart pin)
    U_TMC_GCONF gconf;
    // Set all to 0
    gconf.value = 0;
    // Disable pdn
    gconf.bits.pdn_disable = 1;      // Enable uart communication
    gconf.bits.mstep_reg_select = 1; // Microstepping from register
    gconf.bits.shaft = 1;            // Inverse motor
    gconf.bits.scale_analog = 1;     // Vref a sense
    gconf.bits.multistep_filt = 1;
    // Write config to TMC
    TMC_write_to_register(E_TMC_REG_GCONF, gconf.bytes);
}

/// @brief Computes the CRC, last byte of transmission.
/// @param frame
/// @param framelength
void TMC_calculate_crc(uint8_t *frame, uint8_t frame_length)
{
    int i, j;
    uint8_t crc = 0;
    uint8_t current_byte;

    for (i = 0; i < (frame_length - 1); i++)
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
    frame[frame_length - 1] = crc;
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

    for (uint8_t i = 0; i < 4; i++)
    {
        frame[i + 3] = data[3 - i]; // MSB first
    }
    TMC_calculate_crc(frame, D_TMC_FRAME_LENGTH);
    // Send frame
    TMC_send_frame(frame, D_TMC_FRAME_LENGTH);
}

/// @brief Order from LSB->MSB to MSB->LSB
/// @param frame
void TMC_order_bytes(uint8_t frame[D_TMC_FRAME_LENGTH - 4])
{
    uint8_t new_frame[D_TMC_FRAME_LENGTH - 4];
    for (uint8_t i = 0; i < D_TMC_FRAME_LENGTH - 4; i++)
        new_frame[i] = frame[3 - i];
    for (uint8_t i = 0; i < D_TMC_FRAME_LENGTH - 4; i++)
        frame[i] = new_frame[i];
}

void TMC_read_from_register(E_TMC_REG reg_address, uint8_t rx[D_TMC_FRAME_LENGTH])
{
    // Write the register and read the response
    // Prepare frame
    uint8_t frame[D_TMC_FRAME_LENGTH - 4]; // Add space for init, chip adress, CRC

    frame[0] = D_TMC_INIT_BYTE;    // Init
    frame[1] = D_TMC_NODE_ADDRESS; // Chip address
    frame[2] = reg_address;        // Reg address + R

    TMC_calculate_crc(frame, D_TMC_FRAME_LENGTH - 4);
    TMC_send_frame(frame, D_TMC_FRAME_LENGTH - 4);

    // Read echo+response
    uint8_t rx_buffer[D_TMC_FRAME_LENGTH + 4];

    int len = uart_read_bytes(
        UART_NUM_2,
        rx_buffer,
        D_TMC_FRAME_LENGTH + 4,
        pdMS_TO_TICKS(50));

    // Save only payload (32 bit data response)
    // order bit correctly (MSB frs)
    for (uint8_t i = 7; i < 7 + 4; i++)
    {
        rx[i - 7] = rx_buffer[i];
    }
    TMC_order_bytes(rx);
}

void TMC_reset()
{
    U_TMC_GCONF gconf;
    gconf.value = 0;
    TMC_write_to_register(E_TMC_REG_GCONF, gconf.bytes);
}
