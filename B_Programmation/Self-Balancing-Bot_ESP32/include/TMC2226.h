/**
 * @file TMC2226.h
 * @brief TMC2226 UART driver interface.
 *
 * Declares the TMC2226 class, register definitions, and data structures
 * required to communicate with a TMC2226 stepper motor driver over UART.
 */
#ifndef TMC2226_H
#define TMC2226_H

#include <stdint.h>
#include <Arduino.h>
#include <driver/uart.h>

// GPIO pin assignments
#define D_TMC_PIN_STEP0 25
#define D_TMC_PIN_DIR0 32
#define D_TMC_PIN_STEP1 26
#define D_TMC_PIN_DIR1 33
#define D_TMC_PIN_EN 27

// UART pins
#define D_TMC_PIN_TX2 17
#define D_TMC_PIN_RX2 16

// UART constants
#define D_TMC_INIT_BYTE 0x05
#define D_TMC_NODE_ADDRESS 0x00
#define D_TMC_MASTER_ADDRESS 0xFF
#define D_TMC_FRAME_LENGTH 8 // bytes
#define D_TMC_DATA_LENGTH 4
#define D_TMC_BAUDRATE 500000

// Default register values
#define D_TMC_REGDFV_CHOPCONF 0x15010053
#define D_TMC_REGDFV_GCONF 0x101
#define D_TMC_DEF_MRES 0b0000          // Datasheet value
#define D_TMC_DEF_MICROSTP 256         // Microstep/stp
#define D_TMC_CLKFRQ 11.7e6f           // Hz
#define D_TMC_VACTUALSCALE 16777216.0f //(2^24)
#define D_TMC_VACTUAL_CONVERSION D_TMC_VACTUALSCALE / D_TMC_CLKFRQ
#define D_TMC_STPPERREV 200
// Class
class TMC2226
{
public:
    // Enumerates
    enum Register : uint8_t
    {
        E_REG_GCONF = 0x00,
        E_REG_IOIN = 0x06,
        E_REG_VACTUAL = 0x22,
        E_REG_CHOPCONF = 0x6C
    };
    // Registers data structures
    union CHOPCONF
    {
        struct
        {
            uint32_t toff : 4;
            uint32_t hstrt : 3;
            uint32_t hend : 4;
            uint32_t reserved : 4;
            uint32_t tbl : 2;
            uint32_t vsense : 1;
            uint32_t reserved2 : 6;
            uint32_t mres : 4;
            uint32_t intpol : 1;
            uint32_t dedge : 1;
            uint32_t diss2g : 1;
            uint32_t diss2vs : 1;
        } bits;

        uint32_t value;
        uint8_t bytes[4];
    };
    union GCONF
    {
        struct
        {
            uint32_t scale_analog : 1;
            uint32_t internal_rsense : 1;
            uint32_t en_spreadcycle : 1;
            uint32_t shaft : 1;
            uint32_t index_otpw : 1;
            uint32_t index_step : 1;
            uint32_t pdn_disable : 1;
            uint32_t mstep_reg_select : 1;
            uint32_t multistep_filt : 1;
            uint32_t test_mode : 1;
        } bits;
        uint32_t value;
        uint8_t bytes[4];
    };

    union VACTUAL
    {
        int32_t value;
        uint8_t bytes[4];
    };

    uint8_t node_address;

    TMC2226(uint8_t nd_addr);

    void init();

    void enable();
    void disable();

    void run_speed(float rad_s);

    void write_to_register(Register reg_address, uint8_t *data);
    uint32_t read_register(Register reg_address);
    int32_t rad_s_to_vactual(float rad_s);

private:
    uint8_t m_nodeAddress;

    void send_frame(uint8_t *frame, uint8_t length);
    void calculate_crc(uint8_t *frame, uint8_t frame_length);
    void invert_bytes(uint8_t frane[D_TMC_FRAME_LENGTH - 4]);
};

#endif
