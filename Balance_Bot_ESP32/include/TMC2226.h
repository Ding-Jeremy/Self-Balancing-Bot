#ifndef TMC2226_H
#include <stdint.h>
#include <Arduino.h>
#include <driver/uart.h>
// Pining
#define D_TMC_PIN_STEP 32

// UART constants
#define D_TMC_INIT_BYTE 0x05
#define D_TMC_NODE_ADDRESS 0x00
#define D_TMC_FRAME_LENGTH 8 // bytes

// Registers default values
#define D_TMC_REGDFV_CHOPCONF 0x15010053

// ENUMERATES
typedef enum
{
    E_TMC_REG_GCONF = 0x00,
    E_TMC_REG_VACTUAL = 0x22,
    E_TMC_REG_CHOPCONF = 0x6C,
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

void TMC_step();
void TMC_send_frame(uint8_t *frame, uint8_t frame_length);

void TMC_init(uint32_t baud_rate, uint8_t tx_pin, uint8_t rx_pin);

void TMC_calculate_crc(uint8_t *frame);

void TMC_write_to_register(E_TMC_REG reg_address, uint8_t *data);

void TMC_read_from_register(E_TMC_REG reg_address, uint8_t rx[D_TMC_FRAME_LENGTH]);

void TMC_order_bytes(uint8_t frame[D_TMC_FRAME_LENGTH - 4]);
void TMC_reset();
#endif
#define TMC2226_H
