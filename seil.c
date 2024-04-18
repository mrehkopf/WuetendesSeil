#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/cpufunc.h>
#include <avr/pgmspace.h>
#include <stdbool.h>

#define F_CPU 8000000UL
#include <util/delay.h>

#define LOST_THRES 120
#define SYNC_THRES 240

/* 

   W Ü T E N D E S   S E I L
   =========================

   HYDRA v1 CFW by ikari_01 <otakon@gmx.net>
   Features:
   * MANUAL input selection (press middle button to toggle)
   * fast input seeking, no running light bullshit
   * LEDs are OFF during seeking, only the input found will be illuminated
   * LED PWM disabled to eliminate humming noise in the audio output
   * 2s input loss tolerance (to avoid seeking on console reset)
//   * active input stack
//     - remembers the order of inputs used and rewinds them on
//       loss of an input (not yet)
   * ... (forgot)
*/

/* PORT MAP FOR HYDRA
 PB2   out   front LED + video buffer LPF (0=on, 1=off)
 PB3   out   extension option select (input  9-24)
 PB4   out   extension option select (input 17-24)

 PC0   in    right button
 PC1   in    center button
 PC2   in    left button
 PC3   in    input extension sense 0
 PC4   in    input extension sense 1

 PD0   out   UART SPI master in (set as output to enable UART SPI)
 PD1   out   UART SPI master out
 PD2   in    CSync (triggers INT0)
 PD3   out   LED shift register load
 PD4   out   UART SPI clock out
 PD5   out   input enable shift register load
*/

/* BUTTON MAPPING FOR HYDRA CFW:

 center button:
     - push: toggle manual/auto input select
     - hold: toggle SCART input illumination

 left/right button:
     - manual mode: select input
     - auto mode: force input change
*/

FUSES = {
  .low = LFUSE_DEFAULT | ~(FUSE_CKDIV8),
  .high = FUSE_SPIEN,
  .extended = EFUSE_DEFAULT
};

uint8_t autoselect;
uint8_t last_manual_input;
uint8_t num_inputs;
uint8_t current_input;
volatile uint8_t debounce_buf[8];
uint8_t debounce_count;
uint8_t buttons;
uint8_t phasecount;
uint32_t leds;
uint8_t tick_cnt;
uint8_t enable_front_led;
uint8_t enable_input_leds;
uint8_t input_lost = 1;
volatile uint8_t csync_count;
uint8_t p0, p1, p2, p3;
uint8_t prev_input;
volatile uint8_t input_sample_en = 0;

/* This sets two additional GPIOs low, depending on the
   currently selected input. Purpose unknown yet
   PB3 = 0 for input >= 8
   PB4 = 0 for input >= 16 */
void set_option_outputs() {
  if(current_input >= 8) {
    PORTB &= ~(_BV(3));
    if(current_input >= 16) {
      PORTB &= ~(_BV(4));
    } else {
      PORTB |= _BV(4);
    }
  } else {
    PORTB |= _BV(3) | _BV(4);
  }
  return;
}

void init_GPIO() {
  /* PD2 pullup */
  PORTD |= _BV(2);
  DDRC &= ~(_BV(0) | _BV(1) | _BV(2) | _BV(3) | _BV(4));
  DDRD = _BV(0) | _BV(1) | _BV(3) | _BV(4) | _BV(5);
  DDRB |= _BV(2) | _BV(3) | _BV(4);
  PORTB &= ~(_BV(3) | _BV(4));
  for (volatile uint16_t i = 0; i < 512; i++);
  return;
}

void init_SPI() {
  /* reset baud rate */
  UBRR0H = 0;
  UBRR0L = 0;
  /* set USART to SPI mode */
  UCSR0C = (_BV(UMSEL01) | _BV(UMSEL00));
  UCSR0B = _BV(TXEN0);
  /* must reset baud rate again after TX enable */
  UBRR0H = 0;
  UBRR0L = 0;
  return;
}

void init_timer_IRQ() {
  GTCCR = _BV(PSRSYNC); /* reset prescaler */
  TCCR0A = 0;           /* no compare/PWM/signal gen */
  TCCR0B = 0x02;        /* clock select: clk_io / 8 */
  TIMSK0 = _BV(TOIE0);  /* enable Timer0 overflow interrupt */
  return;
}

void init_pin_IRQ() {
  EICRA = _BV(ISC01) | _BV(ISC00); /* generate INT0 on rising edge of INT0 pin PD2 */
  EIMSK = _BV(INT0);               /* enable INT0 */
  return;
}

void clear_PD0() {
  PORTD &= ~(_BV(0));
  return;
}

void set_PD0() {
  PORTD |= _BV(0);
  return;
}

void spi_send(uint8_t data) {
  UDR0 = data;
  while(!(UCSR0A & _BV(TXC0)));
  UCSR0A = _BV(TXC0);
  return;
}

void spi_send24(uint32_t data) {
  spi_send((data >> 16) & 0xff);
  spi_send((data >> 8) & 0xff);
  spi_send((data >> 0) & 0xff);
  return;
}

void update_debounce_buf() {
  debounce_count = (debounce_count + 1) & 0x07;
  debounce_buf[debounce_count] = (~PINC) & 0x1f;
  return;
}

const uint8_t masktable[8] PROGMEM = {
    0x08, 0x04, 0x02, 0x01, 0x10, 0x20, 0x40, 0x80
};

uint32_t get_input_enable(uint8_t input) {
  union {
    uint32_t result;
    uint8_t ena[4];
  } bits;
  bits.result = 0;
  bits.ena[(input >> 3) & 0x3] = pgm_read_byte(&(masktable[input & 0x7]));
  return bits.result;
}

void sr_latch_inputs() {
  PORTD &= ~(_BV(5));
  _NOP();
  PORTD |= _BV(5);
  _NOP();
  PORTD &= ~(_BV(5));
  return;
}

void sr_latch_leds() {
  PORTD &= ~(_BV(3));
  _NOP();
  PORTD |= _BV(3);
  _NOP();
  PORTD &= ~(_BV(3));
  return;
}

bool get_debounced_buttons(uint8_t *changed) {
  uint8_t tmp = 0x07;
  for(int i=0; i < 8; i++) {
    tmp &= debounce_buf[i] & 7;
  }
  if(tmp == 0) {
    // no buttons pressed for at least 8 ticks
    buttons = 0;
    *changed = 0;
    return false;
  } else if(buttons != tmp) {
    // buttons pressed and state changed since last poll
    buttons = tmp;
    *changed = 1;
    return true;
  } else {
    // buttons pressed but state unchanged since last poll
    *changed = 0;
    return true;
  }
}

int main(void) {
  init_GPIO();
  init_SPI();
  init_timer_IRQ();
  init_pin_IRQ();
  PORTD &= ~(_BV(0) | _BV(1) | _BV(4));
  sei();
  enable_front_led = 1;
  PORTB &= ~(_BV(2));
  enable_input_leds = 1;
  autoselect = 0;
  uint8_t buttons_changed;
  int8_t search_dir, next_input;
/* Detect input extension (Hydra "HEADS") by pulling up the presence
   pins and checking for GNDed inputs
   no ext   ->  8 inputs
   one ext  -> 16 inputs
   two exts -> 24 inputs */
  PORTC = _BV(3) | _BV(4);
  _NOP(); _NOP();
  uint8_t ext_sense = (~PINC) & (_BV(3) | _BV(4));
  if(ext_sense == (_BV(3) | _BV(4))) {
    num_inputs = 24;
  } else if(ext_sense == _BV(3) || ext_sense == _BV(4)) {
    num_inputs = 16;
  } else {
    num_inputs = 8;
  }
  search_dir = 1;

  uint8_t frames_lost = 0;
  uint8_t input_active = 0;

  while(1) {
    /* wait for input change complete */
    while(!input_sample_en);
    csync_count = 0;
    /* count hsync for a given time (via INT0 handler) */
    _delay_ms(20);
    bool force_input_change;
    bool repeat_button_poll;
    do {
      /* less than 128 scanlines -> "no sync" */
      if(csync_count < SYNC_THRES && autoselect) {
        if(input_active) {
          /* grace period on sync loss when input was active
          -> don't lose input when the console/computer is merely reset */
          input_active = 0;
          frames_lost = 1;
        } else if (frames_lost) {
          frames_lost++;
        }
        if(frames_lost >= LOST_THRES) {
          /* No sync for a longer time -> finally switch input */
          input_lost = 1;
          frames_lost = 0;
        }
      } else {
        input_lost = 0;
        frames_lost = 0;
        input_active = 1;
      }
      if(input_lost || force_input_change) {
        input_active = 0;
        input_lost = 1;
        next_input = current_input + search_dir;
        if(next_input >= num_inputs) {
          next_input = 0;
        }
        if(next_input < 0) {
          next_input = num_inputs - 1;
        }
        current_input = next_input;
        input_sample_en = 0;
      }
      force_input_change = false;
      do {
        repeat_button_poll = false;
        bool buttons_pressed = get_debounced_buttons(&buttons_changed);
        if(buttons_pressed && buttons_changed) {
          // get buttons
          if(buttons == _BV(1)) {
            autoselect ^= 1;
            // Toggle autoselect and force immediate scan on inactive input.
            // An active input will override this before scan is forced.
            input_lost = 1;
          } else if(buttons == _BV(2)) {
            // l button pushed - direction left
            search_dir = 1;
            force_input_change = true;
          } else if(buttons == _BV(0)) {
            // r button pushed - direction right
            search_dir = -1;
            force_input_change = true;
          } else if(buttons == (_BV(0) | _BV(2))) {
            // l+r buttons pushed - toggle active input LED on/off
            enable_input_leds ^= 1;
            repeat_button_poll = true;
          }
        }
      } while (repeat_button_poll);
    } while (force_input_change);
  }
}

ISR(INT0_vect) {
  /* gets triggered on rising edge of CSync */
  if(csync_count <= SYNC_THRES && input_sample_en) csync_count++;
}

ISR(TIMER0_OVF_vect, ISR_BLOCK) {
  /* gets triggered periodically */
  update_debounce_buf();
  tick_cnt = (tick_cnt + 1) & 0x1f;
  if(!tick_cnt) {
    // every 32 ticks: update LEDs
    prev_input = p3;
    p3=p2;
    p2=p1;
    p1=p0;
    p0=current_input;
    uint32_t input_en;
    uint32_t input_en_p;
    input_en = get_input_enable(current_input);
    input_en_p = get_input_enable(prev_input);
    uint32_t input_en_final = input_en | input_en_p;
    if(input_en == input_en_p) {
      input_sample_en = 1;
    } else {
      input_sample_en = 0;
    }
    spi_send24(input_en_final);
    sr_latch_inputs();
    if(!enable_input_leds || input_lost) {
      spi_send24(0);
    }
    sr_latch_leds();
  }
}
