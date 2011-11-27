#include "tnc.h"

#define TNC_RB_SIZE 7

signed char     tnc_adcval;         // Zero-biased ADC input

int16_t         tnc_mult_rb[TNC_RB_SIZE], // Ring buffer for ADC*delay values
                tnc_mult_sum,       // Sum of above
                tnc_bias_sum;       // Sum of last 128 ADC readings

unsigned char   tnc_on,             // TNC enabled?
                tnc_adcraw,         // Raw value from ADCH register
                tnc_change_counter, // Samples since last symbol
                tnc_phase_error,   
                tnc_symbol_curr,    // MARK or SPACE
                tnc_symbol_last,    // MARK or SPACE
                tnc_symbol_last_fm, // MARK or SPACE (in a frame)
                tnc_in_a_frame,     // Recorded start of frame marker
                tnc_bit_times,      // Bit durations elapsed
                tnc_bitqlen,        // Bits received in bitq
                tnc_pop_bits,       // Bits to pop off the end
                tnc_byteval,        // Recorded byte from bitq
                tnc_rb_pos,         // Ring buffer position
                tnc_msg[TNC_PACKET_SIZE+1],
                tnc_msg_pos,
                tnc_decode_state,   // Section of received data being parsed
                tnc_bias_counter,   // Number of ADC samples collected
                tnc_bias_adc,       // Center value for ADC, self-adjusting
                tnc_heartbeat,      // Heartbeat LED
                tnc_sync_error_counter, // Sync errors in current frame
                tnc_bit_timer,      // Countdown one bit duration
                tnc_these_bits;     // Number of elapsed bit durations

unsigned char   temp, i;            // Counters and place holders

signed char     tnc_adc_delay[6];   // Delay line for ADC readings

uint32_t        tnc_bitq;           // Recorded bits waiting to be grouped
                                    // in bytes

static uint8_t tnc_sine[16] = {58,22,46,30,62,30,46,22,6,42,18,34,2,34,18,42};
static uint8_t tnc_sine_index;
static uint8_t tnc_tx;
volatile unsigned char tnc_txtone;
volatile unsigned char tnc_main_delay;

static unsigned char tnc_inbuf[TNC_PACKET_SIZE + 1];    // USART buffer
static unsigned char tnc_inhead;                        // USART head ptr
static unsigned char tnc_intail;                        // USART tail ptr

static unsigned char tnc_data_prev; // Previous data from serial
static unsigned short tnc_crc;      // Cyclic redundancy check

void 
tnc_init(unsigned char port) {
    tnc_on = 0;
    tnc_tx = 0;

    // hardware USART0 (MEGA or Due)
    // timer value of 19.2kbps serial output to match bootloader
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0A |= (1<<U2X0);
    UCSR0B |= (1<<TXEN0) | (1<<RXEN0);
    UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);

    UCSR0B |= (1<<RXCIE0); //enable rx interrupt

    // ADC (MEGA or Due)
    ADMUX   = (1<<REFS0);                    // channel0, ref to external input (Aref)
    ADMUX  |= (1<<ADLAR);              // left-justified (only need 8 bits)
    ADCSRA  = (1<<ADPS2);              // pre-scale 16
    ADCSRA |= (1<<ADATE);              // auto-trigger (free-run)
    ADCSRB  = 0x00;                    // free-running
    DIDR0  |= (1<<ADC0D);              // disable digital driver on ADC0 
    ADCSRA |= (1<<ADEN);               // enable ADC
    ADCSRA |= (1<<ADSC);               // trigger first conversion  

    // use 16-bit timer to check the ADC at 13200 Hz. 
    // this is 11x the 1200Hz MARK freq, and 6x the 2200Hz SPACE freq.

#ifdef ARD_MEGA
    // use Timer3 as sample clock on Arduino Mega (ATmega1280)
    // Timer1 conflicted with Arduino "delay()" function
    TCCR3A = 0x00;
    TCCR3B = (1<<WGM32) | (1<<CS30);
    TCCR3C = 0x00;
    OCR3A = T3TOP;
    TIMSK3 = (1<<OCIE3A);                // enable compare match interrupt
#endif

#ifdef ARD_DUE
    // use Timer1 on Arduino Duemilanove (ATmega328P)
    TCCR1A = 0x00;
    TCCR1B = (1<<WGM12) | (1<<CS10);
    TCCR1C = 0x00;
    OCR1A = T3TOP;  //set timer trigger frequency 
    TIMSK1 = (1<<OCIE1A);
    // use timer2 for a symbol (bit) timer 
    TCCR2A = 0;
    //  Initialize the 8-bit Timer2 to clock at 1200hz
    TCCR2B = 0x04;                          // Timer2 clock prescale of 
    // enable overflow interrupt(do this later....
    //TIMSK2 = 1<<TOIE2; //TODO
    // enable overflow interrupt flag to trigger on overflow
    TIFR2 = (1<<TOV2);
#endif

    // use blinky on "pin 13" as DCD light, use "pin 12" as sample clock heartbeat
    // the port/bit designation for these Arduino pins varies with the chip used
    // see the chip-specific DEFINEs above for details
    //DDRB = 0x3F;  

    //set a debug pin 
    DDRD = 0x04;

    // pause to settle
    _delay_ms( 1000 );

    // announce ourselves
    // TODO: make this a status packet? :) 
    send_serial_str("FT817plus TNC ready.\n\n\n");

    // enable interrupts
    sei();

    // pause again for ADC bias to settle
    _delay_ms( 1000 );

}

void
tnc_loop(void) {
    tnc_on = 1;
    tnc_process_serial();
}

// Timer1 interrupt
ISR(TIMER1_COMPA_vect) {
    // If transmitting a sine
    if (tnc_tx) {
        ++tnc_sine_index;
        tnc_sine_index &= 0x0f;             // Wrap
        PORTB = tnc_sine[tnc_sine_index];   // Load next D-to-A sample
        OCR1A = tnc_txtone;                 // Load counter based on freq
    } else {
        // Calculate ADC bias (average of last 128 ADC samples)
        // this input is decoulped from the receiver with a capacitor, 
        // and is re-biased to half of the Arduino regulated +3.3V bus
        // with a voltage divider. therefore the net bias should equal
        // (more or less) the settle point of the voltage divider.
        // doing this in software also means that the calculated bias
        // will re-center automatically if the resistors are not 
        // perfectly matched, etc.
        tnc_adcraw = ADCH;
        tnc_bias_counter += tnc_adcraw;

        if (++tnc_bias_counter == 128) {
            tnc_bias_adc = tnc_bias_sum >> 7;
            tnc_bias_counter = 0;
            tnc_bias_sum = 0;
        }

        // Seguine math, see http://www.cypress.com/?docID=2328
        tnc_adcval = tnc_adcraw - tnc_bias_adc;
        
        // Ring buffer adjustment
        if (++tnc_rb_pos == 7) { tnc_rb_pos = 0; }

        // Back out old value
        tnc_mult_sum -= tnc_mult_rb[tnc_rb_pos];

        // Multiply the latest ADC value by the value ~1/2 lo-freq duration
        // ago. if the incoming audio is the 1200 Hz MARK tone, the two samples
        // will (usually) have opposite phases/signs, so multiplying the two
        // values will give a negative result.  If the incoming audio is 2200
        // Hz, the two samples usu. have the same phase/sign, so multiplying
        // them will give a positve result.
        tnc_mult_rb[tnc_rb_pos] = tnc_adcval * tnc_adc_delay[5];

        // Add this result to get the sum of the last 7 multipliers (1st LPF)
        tnc_mult_sum += tnc_mult_rb[tnc_rb_pos];

        // Force a definitive answer with a hysteresis zone around zero (2nd
        // LPF)
        if      (tnc_mult_sum >=  100) { tnc_symbol_curr = AX25_SPACE; }
        else if (tnc_mult_sum <= -100) { tnc_symbol_curr = AX25_MARK; }
        else                           { ; } // Inconclusive

        // Adjust the sample counter
        if (++tnc_change_counter > 200) { tnc_change_counter = 200; }
        tnc_these_bits = 0;

        for (i = 5; i >= 1; i--) {
            tnc_adc_delay[i] = tnc_adc_delay[i - 1];
        }
        tnc_adc_delay[0] = tnc_adcval;

        /*
         * Clock and bit recovery
         */
        
        if (tnc_in_a_frame) {
            // Clock recovery within a frame
            tnc_bit_timer--;

            // Handle symbol change
            if (tnc_symbol_curr != tnc_symbol_last) {
                tnc_symbol_last = tnc_symbol_curr;
                tnc_change_counter = 0;

                // Ideally, frequency transitions will occur on a
                // since-last-change count that is an exact bit duration at the
                // 1200 Hz signaling rate - that is, an exact multiple of 11
                // samples (11 samples = 1 bit, 22 = 2 bits, 33 = 3, etc). To
                // give some extra settle time, we don't attempt to read the
                // symbol value at the exact moment of transition; instead, we
                // give it 4 extra beats. Thus as bit_timer is counting down to
                // the next symbol check, its value should ideally be 4 when
                // the symbol change actually takes place. If the symbol change
                // is a little early or late, we can tweak the bit_timer to
                // tolerate some drift and still keep sync.  By those rules, an
                // SLC of 4 is perfect, 2 through 6 are fine and  need no
                // adjustment, 7,8 and 0,1 need adjustment, 9,10 are timing
                // errors - we can accept a certain number of those before
                // aborting.
                if (tnc_bit_timer == 7 || tnc_bit_timer == 8) {
                    // Other station was slow, nudge timer
                    --tnc_bit_timer;
                } else if (tnc_bit_timer == 0 || tnc_bit_timer == 1) {
                    // Other station was fast, nudge timer
                    ++tnc_bit_timer;
                } else if (tnc_bit_timer >= 9) {
                    // Frame sync error
                    if (++tnc_sync_error_counter > TNC_MAX_SYNC_ERRORS) {
                        tnc_sync_error_counter = 0;
                        tnc_msg_pos = 0;
                        tnc_in_a_frame = 0;
                        tnc_bitq = 0;
                        tnc_bitqlen = 0;
                        tnc_bit_timer = 0;
                        tnc_bit_times = 0;
                        // Turn off DCD LED
                        //DCD_OFF;
                        return;
                    }
                }
            }

            // Bit recovery in a frame
            if (tnc_bit_timer == 0) {
                tnc_bit_timer = 11;
                tnc_bit_times++;

                // Wait for a symbol change and sync frame
                if (tnc_symbol_curr != tnc_symbol_last_fm) {
                    tnc_these_bits = tnc_bit_times + 1;
                    tnc_bit_times = 0;
                    tnc_symbol_last_fm = tnc_symbol_curr;
                }
            } 
        } else {
            // Not in a frame
            // Do some house keeping
            tnc_phase_error = tnc_change_counter;
            // Because modulo operator is slow
            while (tnc_phase_error >= 11) { tnc_phase_error -= 11; }

            tnc_bit_times = 0;
            temp = tnc_change_counter + 5;
            while (temp > 11) { temp -= 11; ++tnc_bit_times; }
            tnc_these_bits = 0;

            // Clock recovery
            
            // No symbol change? We'll check in the next pass
            if (tnc_symbol_curr == tnc_symbol_last) {
                return;
            }

            // Symbol change
            tnc_symbol_last = tnc_symbol_curr;
            tnc_change_counter = 0;

            // Check bit sync
            if (tnc_phase_error >= 4 && tnc_phase_error <= 7) {
                // Too many errors
                tnc_bitq = 0;
                tnc_bitqlen = 0;
                // Turn off DCD LED
                //DCD_OFF;
            }

            // Save these bits
            tnc_these_bits = tnc_bit_times + 1;
        }

        // Bit recovery (inside or outside frame)
        if (tnc_these_bits == 0) {
            return;
        } else {
            --tnc_these_bits;
        }

        // Determine incoming bit values based on how many bit times have
        // elapsed.  whatever the count was, the last bit involved a symbol
        // change, so must be zero.  all other elapsed bits must be ones. AX.25
        // is transmitted LSB first, so in adding bits to the incoming bit
        // queue, we add them right-to-left (ie, new bits go on the left). this
        // lets us ready incoming bytes directly from the lowest eight bits of
        // the bit queue (once we have that many bits).

        // the act of adding bits to the queue is in two parts - (a) OR in any
        // one bits, shifting them to the left as required prior to the OR
        // operation, and (b) update the number of bits stored in the queue.
        // with zero bits, there's nothing to OR into place, so they are taken
        // care of when we update the queue length, and when we shift the queue
        // to the right as bytes are popped off the end later on.
        switch (tnc_these_bits) {
            case 1: break;                              // b00000
            case 2: tnc_bitq |= (0x01 << tnc_bitqlen);  // b00001
                    break;
            case 3: tnc_bitq |= (0x03 << tnc_bitqlen);  // b00011
                    break;
            case 4: tnc_bitq |= (0x07 << tnc_bitqlen);  // b00111
                    break;
            case 5: tnc_bitq |= (0x0f << tnc_bitqlen);  // b01111
                    break;
            // Special case, only use the positive bits
            case 6: tnc_bitq |= (0x1f << tnc_bitqlen);  // b11111
                    tnc_these_bits = 5;
                    break;
            // Special case, only used for HDLC bytes
            case 7: if (tnc_bitqlen == 1 && tnc_bitq == 0) {
                        // Only one bit pending and it's a zero, this is the
                        // ideal situation to receive a "seven" to complete the
                        // HDLC byte
                        tnc_bitq = 0x7e;
                        tnc_bitqlen = 8;
                    } else if (tnc_bitqlen < 4) {
                        // 0 to 3 bits still pending and it's a zero, but no
                        // the recommended single-zero; let's dump what ever is
                        // pending and close the frame
                        tnc_bitq = 0x7e;
                        tnc_bitqlen = 8;
                    } else if (tnc_bitqlen >= 4) {
                        // Half or more of an unfinished byte, close the frame
                        tnc_bitq = (tnc_bitq & 0xff) | 0x7e00;
                        tnc_bitqlen = 16;
                    } else {
                        // Clean up, should not happen, but you never know
                        tnc_bitq = 0x7e;
                        tnc_bitqlen = 8;
                    }

                    // We have neatly adjusted the bitlen, so stop processing
                    tnc_these_bits = 0;
                    break;

            default:
                    // Clear buffers
                    tnc_msg_pos = 0;
                    tnc_in_a_frame = 0;
                    tnc_bitq = 0;
                    tnc_bitqlen = 0;
                    // Do not add to the bitqlen later
                    tnc_these_bits = 0;
                    // Turn off DCD LED
                    //DCD_OFF;
                    break;
        }

        // Count the number of bits we added
        tnc_bitqlen += tnc_these_bits;

        // Byte recovery -- let's talk bytes baby
        
        while (tnc_bitqlen >= 8) {
            // LSB
            tnc_byteval = tnc_bitq & 0xff;

            // HDLC frame marker
            if (tnc_byteval == 0x7e) {
                if (tnc_in_a_frame == 0) {
                    // Marker starts new frame
                    tnc_in_a_frame = 1;
                    tnc_symbol_last_fm = tnc_symbol_curr;
                    tnc_sync_error_counter = 0;
                    tnc_bit_timer = 15;
                    tnc_bit_times = 0;
                    tnc_pop_bits = 8;
                } else if (tnc_msg_pos < TNC_MIN_PACKET_LEN) {
                    // We are already in a frame, but have not rec'd any/enough
                    // data yet.  AX.25 preamble is sometimes a series of HDLCs
                    // in a row, so let's assume that's what this is, and just
                    // drop this byte.
                    tnc_pop_bits = 8;
                } else {
                    // In a frame with some data, so this HDLC is probably a
                    // frame stop
                    if (tnc_msg_pos > 0) {
                        // Dump byte on serial port
                        tnc_ax25_decode();
                    }

                    // Stay in-a-frame mode in case a new frame is starting
                    tnc_msg_pos = 0;
                    tnc_sync_error_counter = 0;
                    tnc_bit_times = 0;
                    tnc_pop_bits = 8;
                }
            } else if (tnc_in_a_frame == 1) {
                // Not a HDLC frame marker, but we have data
                tnc_msg[tnc_msg_pos] = tnc_byteval;
                tnc_msg_pos++;

                // Turn on DCD LED if we have an end-of-head marker
                if (tnc_byteval = 0x03) {
                    //DCD_ON;
                }
            } else {
                // Not already in a frame, and this byte is not a frame marker.
                // It is possible (likely) that when an HDLC byte arrives, its
                // 8 bits will not align perfectly with the 8 we just checked.
                // So instead of dropping all 8 of these random bits, let's
                // just drop one, and re-check the rest again later.  This
                // increases our chances of seeing the HDLC byte in the
                // incoming bitstream amid noise (esp.  if we're running open
                // squelch).
                tnc_pop_bits = 1;
            }

            // Pop the bits
            tnc_bitq >>= tnc_pop_bits;
            tnc_bitqlen -= tnc_pop_bits;
        }

        // Re-enable interrupts
        sei();
        return;
    }
}

/*****************************************************************************
* Set up a generic timer that we can call (only enabled in transmit mode) that
* we use as a symbol timer so that it ticks at ~1200hz 
*****************************************************************************/
SIGNAL(TIMER2_OVF_vect) {
    tnc_main_delay = 0;
    TCNT2 = 0;
}

SIGNAL(USART_RX_vect) {
    if (++tnc_inhead == TNC_BUF_SIZE) tnc_inhead = 0; // Wrap
    tnc_inbuf[tnc_inhead] = UDR0;
}

/*
 * Process serial bus
 */

void
tnc_process_serial(void) {
    if (tnc_on == 0) return;

    PORTD ^= 0x04;
    // If there are incoming bytes pending
    if (tnc_intail != tnc_inhead) {
        if (++tnc_intail == TNC_BUF_SIZE) tnc_intail = 0; // Wrap
        tnc_handle_message(tnc_inbuf[tnc_intail]);    // Pass to handler
    }

    return;
}

inline void
tnc_handle_message(unsigned char data) {
    if (tnc_data_prev == 0xc0) {
        if (data == 0x00) {
            tnc_transmit();
        }
    } else if (data == 0xc0 && tnc_tx == 1) {
        tnc_receive();
    // Escape byte
    } else if (data == 0xdc) {
        if (tnc_data_prev == 0xdb) {
            tnc_ax25_send(0xc0);
        }
    // Escape byte
    } else if (data == 0xdd) {
        if (tnc_data_prev == 0xdb) {
            tnc_ax25_send(0xdb);
        }
    // If in transmission, just send data
    } else if (tnc_tx == 1) {
        tnc_ax25_send(data);
    }

    // Keep track
    tnc_data_prev = data;
}

inline void
tnc_rts(void) {
    while (!TNC_READY_TO_SEND);
}

void
tnc_send_serial(const char *data) {
    while (*data != '\0') {
        tnc_rts();
        UDR0 = *data++;
    }
}

void
tnc_ax25_decode(void) {
    // State:
    //  0. started
    //  1. header
    //  2. received 0x03
    //  3. received 0xf0 (payload)
    tnc_decode_state = 0;

    for (i = 0; (tnc_msg_pos - 2); ++i) {
        switch (tnc_decode_state) {
            case 0:
                tnc_rts();
                UDR0 = 0xc0; // Start frame
                tnc_rts();
                UDR0 = 0x00; // Data on port 0
                tnc_rts();
                UDR0 = tnc_msg[i];
                tnc_decode_state++;
                break;

            case 2:
                // Got the 0x03, waiting for 0xf0
                if (tnc_msg[i] == 0xf0) {
                    tnc_rts();
                    UDR0 = tnc_msg[i];
                    tnc_decode_state++;
                } else {
                    // Corrupt packet, abort
                    tnc_rts();
                    UDR0 = 13; // CR
                    tnc_rts();
                    UDR0 = 10; // LF
                    return;
                }
                break;

            case 1:
                if (tnc_msg[i] == 0x03) {
                    tnc_rts();
                    UDR0 = tnc_msg[i];
                    tnc_decode_state++; 
                    break;
                }
                // else fall through

            default:
                if (tnc_msg[i] == 0xc0) {
                    tnc_rts();
                    UDR0 = 0xdb;
                }
                tnc_rts();
                UDR0 = tnc_msg[i];
                if (tnc_msg[i] == 0xdb) {
                    tnc_rts();
                    UDR0 = 0xdd;
                }
                break;
        } // switch
    } // for

    tnc_rts();
    UDR0 = 0xc0; // End of frame
    tnc_rts();
    UDR0 = 13; // CR
    tnc_rts();
    UDR0 = 10; // LF
}

// Get ready to rumble
void
tnc_transmit(void) {
    tnc_sine_index = 0;
    tnc_txtone = TNC_MARK;
    // Enable overflow interrupt
    TIMSK2 |= (1 << TOIE2);              // Enable timer2
    TCCR1B = (1 << WGM12) | (2 << CS10); // Setup timer1
    TCNT2 = TNC_BIT_DELAY;               // Setup timer2 to trigger at 1200 Hz
    tnc_tx = 1;
    
    tnc_ax25_send_header();
    return;
}

void
tnc_receive(void) {
    tnc_ax25_send_footer();
    tnc_tx = 0;

    PORTB &= 0x00;                       // Shut PTT
    TCCR1B = (1 << WGM12) | (1 << CS10);
    TIMSK2 &= ~(1 << TOIE2);             // Disable timer2 interrupt
    OCR1A = TNC_T3TOP;                   // Set timer1 to rx sample rate
    tnc_sine_index = 0;
    return;
}

void
tnc_delay(unsigned char timeout) {
    tnc_main_delay = 1;
    TCNT2 = 0xff - timeout;              // Setup timer2 to trigger at delay
    while (tnc_main_delay);
    return;
}

void
tnc_ax25_crcbit(int lsb) {
    static unsigned short xor;

    xor = tnc_crc ^ lsb;
    tnc_crc >>= 1;
    if (xor & 0x0001) {
        tnc_crc ^= 0x8408;
    }
    return;
}

void
tnc_ax25_send(unsigned char byte) {
    static char j, bitbyte;
    static int bitzero;
    static unsigned char ones;

    bitbyte = byte;
    for (j = 0; j < 8; ++j) {           // Loop trough 8 bits in the byte
        bitzero = bitbyte & 0x01;       // Read the LSB
        if (byte == 0x73) {
            ones = 1;
        } else {                        // Not a flag...
            tnc_ax25_crcbit(bitzero);   // ...modify the checksum
        }

        if (bitzero == 0) {
            ones = 0;
            tnc_txtone = (tnc_txtone == TNC_MARK) ? TNC_SPACE : TNC_MARK;
        } else {
            if (++ones == 5) {          // Are we at the fifth one?
                tnc_delay(TNC_BIT_DELAY);
                tnc_txtone = (tnc_txtone == TNC_MARK) ? TNC_SPACE : TNC_MARK;
                ones = 0;
            }
        }

        bitbyte >>= 1;
        tnc_delay(TNC_BIT_DELAY);
    }
    return;
}

void 
tnc_ax25_send_header(void) {
    static unsigned char delay;
    tnc_crc = 0xffff;
    for (delay = 0; delay < TNC_TX_DELAY; ++delay) {
        tnc_ax25_send(0x7e);            // Send sync header byte
    }
    return;
}

void 
tnc_ax25_send_footer(void) {
    tnc_ax25_send(tnc_crc ^ 0xff);        // Send LSB of the CRC
    tnc_ax25_send((tnc_crc >> 8) ^ 0xff); // Send MSB of the CRC
    tnc_ax25_send(0x73);                  // Send end of packet flag
    return;
}

