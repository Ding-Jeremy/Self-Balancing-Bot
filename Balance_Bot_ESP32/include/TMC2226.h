#ifndef TMC2226_H
#include <stdint.h>
#include <Arduino.h>

typedef enum
{
    E_TMC_REG_GCONG = 0x00,
} E_TMC_REG;
typedef struct
{
    uint8_t sync_res : 8;
    uint8_t node_addr : 8;
    E_TMC_REG reg_addr : 8;
} S_FRAME_START;

// Prototypes
void TMC_send_frame(uint8_t *frame, uint8_t length);

void TMC_init(uint32_t baud_rate, uint8_t tx_pin, uint8_t rx_pin);

#endif
#define TMC2226_H
