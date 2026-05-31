#ifndef TMC2226_H
#include <stdint.h>
#include <Arduino.h>

#define D_TMC_INIT_BYTE 0x05
#define D_TMC_NODE_ADDRESS 0x00
#define D_TMC_FRAME_LENGTH 8 // bytes

#define D_TMC_PIN_STEP 34

typedef enum
{
    E_TMC_REG_GCONF = 0x00,
    E_TMC_REG_VACTUAL = 0x22
} E_TMC_REG;

typedef struct
{
    uint8_t scale_analog : 1;
    uint8_t internal_rsense : 1;
    uint8_t en_spreadcycle : 1;
    uint8_t shaft : 1;
    uint8_t index_otpw : 1;
    uint8_t index_step : 1;
    uint8_t pdn_disable : 1;
    uint8_t mstep_reg_select : 1;
    uint8_t multistep_filt : 1;
    uint8_t test_mode : 1;
} S_TMC_GCONF;

typedef union
{
    S_TMC_GCONF bits;
    uint8_t bytes[4];
} U_TMC_GCONF;

// Prototypes
void TMC_send_frame(uint8_t *frame);

void TMC_init(uint32_t baud_rate, uint8_t tx_pin, uint8_t rx_pin);

void calculate_crc(uint8_t *frame);

void TMC_write_to_register(E_TMC_REG reg_address, uint8_t *data);

#endif
#define TMC2226_H
