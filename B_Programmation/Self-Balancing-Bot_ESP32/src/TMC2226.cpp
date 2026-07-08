/**
 * @file TMC2226.cpp
 * @brief Implementation of the TMC2226 UART driver.
 *
 * This file implements the TMC2226 class, which provides methods for
 * configuring and controlling a TMC2226 stepper motor driver over UART.
 * It includes functions for driver initialization, register read/write
 * operations, CRC calculation, and motor control.
 *
 * @author Ding Jérémy
 * @date 05/2026
 */
#include "TMC2226.h"

/// @brief TMC2226 class, pass module address in parameters
/// @param node_address
TMC2226::TMC2226(uint8_t nd_addr)
{
    node_address = nd_addr;
}

/// @brief Sends a UART frame to the TMC2226.
/// @param frame Pointer to the frame to transmit.
/// @param frame_length Number of bytes to send.
void TMC2226::send_frame(uint8_t *frame, uint8_t frame_length)
{
    // Clear the receive buffer before transmitting.
    // Wait until the transmission is complete.
    uart_flush_input(UART_NUM_2);
    uart_write_bytes(UART_NUM_2, frame, frame_length);
    uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(10));
    vTaskDelay(pdMS_TO_TICKS(1));
}

/// @brief Enables the motors
void TMC2226::enable()
{
    digitalWrite(D_TMC_PIN_EN, LOW);
}

/// @brief Disables the motors
void TMC2226::disable()
{
    digitalWrite(D_TMC_PIN_EN, HIGH);
}

/// @brief Initializes the TMC2226 driver and configures UART communication.
void TMC2226::init()
{
    // Set pins output
    pinMode(D_TMC_PIN_STEP0, OUTPUT);
    pinMode(D_TMC_PIN_STEP1, OUTPUT);
    pinMode(D_TMC_PIN_DIR0, OUTPUT);
    pinMode(D_TMC_PIN_DIR1, OUTPUT);
    pinMode(D_TMC_PIN_EN, OUTPUT);

    delay(10);
    uart_config_t uart_config = {
        .baud_rate = D_TMC_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    esp_err_t err;

    err = uart_param_config(UART_NUM_2, &uart_config);
    printf("param_config = %s\n", esp_err_to_name(err));

    err = uart_set_pin(UART_NUM_2, D_TMC_PIN_TX2, D_TMC_PIN_RX2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    printf("set_pin = %s\n", esp_err_to_name(err));

    err = uart_driver_install(UART_NUM_2, 256, 256, 0, NULL, 0);
    printf("driver_install = %s\n", esp_err_to_name(err));

    // Configure the driver to use UART communication.
    GCONF gconf;
    CHOPCONF chopconf;

    // Clear all configuration bits.
    gconf.value = 0;
    // Disable PDN mode to enable UART communication.
    gconf.bits.pdn_disable = 1;      // Enable uart communication
    gconf.bits.mstep_reg_select = 1; // Microstepping from register
    gconf.bits.scale_analog = 1;     // Vref a sense
    gconf.bits.multistep_filt = 1;   // Activate multistep filtering

    chopconf.value = D_TMC_REGDFV_CHOPCONF; // DEFAULT VALUE
    chopconf.bits.mres = D_TMC_DEF_MRES;    // Microstepping
    // Write the configuration to the GCONF register.
    write_to_register(Register::E_REG_GCONF, gconf.bytes);
    write_to_register(Register::E_REG_CHOPCONF, chopconf.bytes);
}

/// @brief Computes the CRC and stores it in the last byte of the frame.
/// @param frame Frame buffer.
/// @param frame_length Total frame length in bytes.
void TMC2226::calculate_crc(uint8_t *frame, uint8_t frame_length)
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

/// @brief Writes a 32-bit value to the specified register.
/// @param reg_address Register to write.
/// @param data Pointer to the 4-byte data payload.
void TMC2226::write_to_register(Register reg_address, uint8_t *data)
{
    // Prepare frame
    uint8_t frame[D_TMC_FRAME_LENGTH]; // Add space for init, chip address, CRC

    frame[0] = D_TMC_INIT_BYTE;    // Synchronization byte.
    frame[1] = node_address;       // Chip address
    frame[2] = reg_address | 0x80; // Register address with the write bit set.

    for (uint8_t i = 0; i < 4; i++)
    {
        frame[i + 3] = data[3 - i]; // MSB first
    }
    calculate_crc(frame, D_TMC_FRAME_LENGTH);
    // Send frame
    send_frame(frame, D_TMC_FRAME_LENGTH);
}

/// @brief Reverses the byte order of a 32-bit data buffer.
/// @param frame
void TMC2226::invert_bytes(uint8_t frame[D_TMC_DATA_LENGTH])
{
    uint8_t new_frame[D_TMC_FRAME_LENGTH - 4];
    for (uint8_t i = 0; i < D_TMC_FRAME_LENGTH - 4; i++)
        new_frame[i] = frame[3 - i];
    for (uint8_t i = 0; i < D_TMC_FRAME_LENGTH - 4; i++)
        frame[i] = new_frame[i];
}

/// @brief Reads the 32-bit value stored in a register.
/// @param reg_address Register to read.
/// @return Register contents.
uint32_t TMC2226::read_register(Register reg_address)
{
    // Write the register and read the response
    // Prepare frame
    uint8_t frame[D_TMC_FRAME_LENGTH - 4]; // Add space for init, chip address, CRC

    frame[0] = D_TMC_INIT_BYTE; // Init
    frame[1] = node_address;    // Chip address
    frame[2] = reg_address;     // Reg address + R

    calculate_crc(frame, D_TMC_FRAME_LENGTH - 4);
    send_frame(frame, D_TMC_FRAME_LENGTH - 4);

    // Read echo+response
    uint8_t rx_buffer[D_TMC_FRAME_LENGTH + 4];

    // Read all response
    int len = uart_read_bytes(
        UART_NUM_2,
        rx_buffer,
        D_TMC_FRAME_LENGTH + 4,
        pdMS_TO_TICKS(50));

    // Check master's address and register validity
    if (rx_buffer[4] != D_TMC_INIT_BYTE || rx_buffer[5] != D_TMC_MASTER_ADDRESS || rx_buffer[6] != reg_address)
    {
        Serial.println("ERR_COM_TMC");
    }
    // Save only payload (32 bit data response)
    // order bit correctly (MSB frs)
    uint32_t payload =
        ((uint32_t)rx_buffer[7] << 24) |
        ((uint32_t)rx_buffer[8] << 16) |
        ((uint32_t)rx_buffer[9] << 8) |
        ((uint32_t)rx_buffer[10]);

    return payload;
}

/// @brief Runs a motor to a desired speed (automatic)
/// @param node_addr
/// @param speed
void TMC2226::run_speed(int32_t speed)
{
    TMC2226::VACTUAL vactual;
    vactual.value = speed;
    write_to_register(Register::E_REG_VACTUAL, vactual.bytes);
}