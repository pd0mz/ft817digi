#ifndef __TNC_H__
#define __TNC_H__

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <inttypes.h>

#define AX25_MARK           0
#define AX25_SPACE          1
#define TNC_MIN_PACKET_LEN  10
#define TNC_PACKET_SIZE     200
#define TNC_MAX_SYNC_ERRORS 5
#define TNC_MIN_DCD         20
#define TNC_T3TOP           1212
#define TNC_READY_TO_SEND   (UCSR0A & (1<<UDRE0))
// 189 delay for 14.7456 MHz
// 205 delay for 16 MHz
#define TNC_BIT_DELAY       206
// Number of 6.7ms delay cycles
#define TNC_TX_DELAY        50
// About 2228 Hz (with 16 MHz clock)
#define TNC_SPACE           (55)
// About 1200.98 Hz (with 16 MHz clock)
#define TNC_MARK            (103)
// Educated guess
#define TNC_BUF_SIZE        (200)

// Function prototypes
void        tnc_process_serial(void);
inline void tnc_handle_message(unsigned char data);
void        tnc_send_serial(const char *data);
void        tnc_ax25_decode(void);
void        tnc_transmit(void);
void        tnc_receive(void);
void        tnc_delay(unsigned char);
void        tnc_ax25_crcbit(int);
void        tnc_ax25_send(unsigned char);
void        tnc_ax25_send_header(void);
void        tnc_ax25_send_footer(void);


#endif // __TNC_H__
