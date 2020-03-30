/*
 * tiny814-clock.c
 *
 * Created: 3/26/2020 11:50:51 AM
 * Author : George
 */ 

#define F_CPU 32768

#include <avr/io.h>
#include <util/delay.h>

// #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
// #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

//
#define gpio_data_high() PORTA_OUTSET = 1 << 7;
#define gpio_data_low() PORTA_OUTCLR = 1 << 7;
//
#define gpio_clock_high() PORTA_OUTSET = 1 << 5;
#define gpio_clock_low() PORTA_OUTCLR = 1 << 5;
//
#define gpio_latch_high() PORTA_OUTSET = 1 << 4;
#define gpio_latch_low() PORTA_OUTCLR = 1 << 4;
//
#define gpio_lcd_com_high() PORTA_OUTSET = 1 << 3;
#define gpio_lcd_com_low() PORTA_OUTCLR = 1 << 3;
//

const uint16_t DIGIT0A[10] = { 3, 0, 5, 5, 6, 7, 7, 1, 7, 7 };
const uint16_t DIGIT0B[10] = { 15, 3, 13, 7, 3, 6, 14, 3, 15, 7 };
const uint16_t DIGIT1A[10] = { 56, 8, 88, 88, 104, 112, 112, 24, 120, 120 };
const uint16_t DIGIT1B[10] = { 224, 32, 192, 96, 32, 96, 224, 32, 224, 96 };
const uint16_t DIGIT2A[10] = { 1792, 256, 2816, 2816, 3328, 3584, 3584, 768, 3840, 3840 };
const uint16_t DIGIT2B[10] = { 3584, 512, 3072, 1536, 512, 1536, 3584, 512, 3584, 1536 };
const uint16_t DIGIT3A[10] = { 28672, 4096, 45056, 45056, 53248, 57344, 57344, 12288, 61440, 61440 };
const uint16_t DIGIT3B[10] = { 28672, 4096, 24576, 12288, 4096, 12288, 28672, 4096, 28672, 12288 };

uint8_t t3 = 0; // hours X10
uint8_t t2 = 0; // hours  X1
uint8_t t1 = 0; // mins  X10
uint8_t t0 = 0; // mins   X1

#define short_delay() _delay_us(5);

void update_lcd(void) {
  uint16_t lcdA = 0; // LCD Content A
  uint16_t lcdB = 0; // LCD Content B
  uint16_t dA = 0;   // LCD Content with polarity
  uint16_t dB = 0;   // LCD Content with polarity
  uint16_t i = 0;    // temp counter
  uint16_t d = 0;    // temp
  uint16_t lcdP = 0; // polarity of LCD

  lcdA = DIGIT0A[t0] | DIGIT1A[t1] | DIGIT2A[t2];
  lcdB = DIGIT0B[t0] | DIGIT1B[t1] | DIGIT2B[t2];

  if (t3) {
    lcdA |= DIGIT3A[t3];
    lcdB |= DIGIT3B[t3];
  }

  lcdA |= 1<<7; ////////

  if (lcdP) {
    dA = lcdA;
    dB = lcdB;
    lcdP = 0;
    } else {
    dA = ~lcdA;
    dB = ~lcdB;
    lcdP = 1;
  }
  
  gpio_latch_low();
  short_delay();

  // output
  i = 0;
  d = dA;
  while (i < 32) {
    if (i == 16) {
      d = dB;
    }

    gpio_clock_low();
    
    if (d & 1) {
      gpio_data_high();
      } else {
      gpio_data_low();
    }

    short_delay();
    
    gpio_clock_high();
    short_delay();

    d = d >> 1;
    i++;
  }

  short_delay();
  gpio_latch_high();

  if (lcdP) {
    gpio_lcd_com_high();
    } else {
    gpio_lcd_com_low();
  }
}

void use_32k_crystal(void) {
  // Set clock to use 32k crystal
  CPU_CCP = CCP_IOREG_gc;                      // Un-protect
  CLKCTRL_XOSC32KCTRLA = (1 << 0) | (1 << 1);  // Enable & Run Standby 32k crystal
  
  CPU_CCP = CCP_IOREG_gc;   // Un-protect
  CLKCTRL_MCLKCTRLA = 0x02; // Use 32k crystal for main clock
  
  while ((CLKCTRL_MCLKSTATUS & CLKCTRL_XOSC32KS_bm) == 0); // wait for stable 32k crystal
  
  CPU_CCP = CCP_IOREG_gc; // Un-protect
  CLKCTRL_MCLKCTRLB = 0;  // Disable pre-scaler
}

/*
void use_32k_rc(void) {
  CPU_CCP = CCP_IOREG_gc;   // Un-protect
  CLKCTRL_MCLKCTRLA = 0x01; // Use 32k RC for main clock
  
  while ((CLKCTRL_MCLKSTATUS & CLKCTRL_OSC32KS_bm) == 0); // wait for stable 32k RC
  
  CPU_CCP = CCP_IOREG_gc; // Un-protect
  CLKCTRL_MCLKCTRLB = 0;  // Disable pre-scaler
}
*/

// Needed for low power
void disable_inputs(void) {
	for (uint8_t i = 0; i < 8; i++) {
  	*((uint8_t *)&PORTA + 0x10 + i) = PORT_ISC_INPUT_DISABLE_gc;
	}
	for (uint8_t i = 0; i < 8; i++) {
  	*((uint8_t *)&PORTB + 0x10 + i) = PORT_ISC_INPUT_DISABLE_gc;
	}
}

/*
Power usage notes:
  - disable_inputs + use_32k_rc      + while(1)            => 16.4 uA
  - disable_inputs + use_32k_crystal + while(1)            => 15.7 uA
  - disable_inputs + use_32k_crystal + update_lcd [NO LCD] => 16.9 uA
  - disable_inputs + use_32k_crystal + update_lcd [W/ LCD] => 17.7 uA
*/

int main(void) {
  disable_inputs();
  use_32k_crystal();

  // port direction
  PORTA_DIRSET = (1 << 3) | (1 << 4) | (1 << 5) | (1 << 7);

  // main loop
  while (1) {
    update_lcd();
    t0++;
    if (t0 >= 9) t0=0;
  }
}
