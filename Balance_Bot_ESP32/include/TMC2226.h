#ifndef TMC2226_H
#include <stdint.h>
#include <Arduino.h>
#include <driver/uart.h>

// Pining
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
#define D_TMC_MASTER_ADRESS 0xFF
#define D_TMC_FRAME_LENGTH 8 // bytes
#define D_TMC_BAUDRATE 200000

// Registers default values
#define D_TMC_REGDFV_CHOPCONF 0x15010053
#define D_TMC_REGDFV_GCONF 0x101

// ENUMERATES
typedef enum
{
    E_TMC_REG_GCONF = 0x00,
    E_TMC_REG_VACTUAL = 0x22,
    E_TMC_REG_CHOPCONF = 0x6C,
    E_TMC_REG_IOIN = 0x06
} E_TMC_REG;
// REGISTERS
typedef struct
{
    uint32_t toff : 4;
    uint32_t hstrt : 3;
    uint32_t hend : 4;
    uint32_t reserved : 4;
    uint32_t tbl : 2;
    uint32_t vsense : 1;
    uint32_t reserved_2 : 6;
    uint32_t mres : 4;
    uint32_t intpol : 1;
    uint32_t dedge : 1;
    uint32_t diss2g : 1;
    uint32_t diss2vs : 1;
} S_TMC_CHOPCONF;

typedef struct
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
} S_TMC_GCONF;

// UNION
typedef union
{
    int32_t value;
    uint8_t bytes[4];
} U_TMC_VACTUAL;

typedef union
{
    S_TMC_CHOPCONF bits;
    uint8_t bytes[4];
    uint32_t value;
} U_TMC_CHOPCONF;
typedef union
{
    S_TMC_GCONF bits;
    uint8_t bytes[4];
    uint32_t value;
} U_TMC_GCONF;

// Prototypes

void TMC_enable();

void TMC_disable();

void TMC_send_frame(uint8_t *frame, uint8_t frame_length);

void TMC_init();

void TMC_calculate_crc(uint8_t *frame);

void TMC_write_to_register(uint8_t node_addr, E_TMC_REG reg_address, uint8_t *data);

uint32_t TMC_read_from_register(uint8_t node_addr, E_TMC_REG reg_address);

void TMC_invert_bytes(uint8_t frame[D_TMC_FRAME_LENGTH - 4]);

void TMC_runspeed(uint8_t node_addr, int32_t speed);

#endif
#define TMC2226_H
