/*
 * tiny814-clock.c
 *
 * Created: 3/26/2020 11:50:51 AM
 * Author : George
 */ 

#define F_CPU 16000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "mytime.h"

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

static const uint16_t DIGIT0A[10] = { 3, 0, 5, 5, 6, 7, 7, 1, 7, 7 };
static const uint16_t DIGIT0B[10] = { 15, 3, 13, 7, 3, 6, 14, 3, 15, 7 };
static const uint16_t DIGIT1A[10] = { 56, 8, 88, 88, 104, 112, 112, 24, 120, 120 };
static const uint16_t DIGIT1B[10] = { 224, 32, 192, 96, 32, 96, 224, 32, 224, 96 };
static const uint16_t DIGIT2A[10] = { 1792, 256, 2816, 2816, 3328, 3584, 3584, 768, 3840, 3840 };
static const uint16_t DIGIT2B[10] = { 3584, 512, 3072, 1536, 512, 1536, 3584, 512, 3584, 1536 };
static const uint16_t DIGIT3A[10] = { 28672, 4096, 45056, 45056, 53248, 57344, 57344, 12288, 61440, 61440 };
static const uint16_t DIGIT3B[10] = { 28672, 4096, 24576, 12288, 4096, 12288, 28672, 4096, 28672, 12288 };

static volatile uint8_t lcdP = 0; // polarity of LCD

static volatile MYTIME current_time = {0,0,0,0,0,0,0};
static volatile MYTIME alarm_time = {0,7,3,0,0,0,0};

enum {
  MODE_DEFAULT = 0,
  MODE_SECONDS = 1,
  MODE_ALARM   = 2,
  MODES_COUNT  = 3,
};
static volatile uint8_t mode = MODE_DEFAULT;

/*
  Current test:
  - All Off    =>  1.50 mA (Fast CPU / NO Sleep)
  - 1x G10     =>  1.65 mA
  - 1x G50     =>  3.50 mA
  - 1x G100    =>  6.20 mA
  - 2x G100    => 10.90 mA
  - 3x G100    => 15.50 mA
  - 1x R10     =>  1.64 mA
  - 1x R50     =>  3.55 mA
  - 1x R100    =>  6.25 mA
  - 1x B10     =>  1.65 mA
  - 1x B50     =>  3.50 mA
  - 1x B100    =>  6.20 mA
  - ALL 100    =>  30+ mA *NOT STABLE*
  - 3x 2x 75   =>  20+ mA *NOT STABLE*
*/
static volatile uint8_t pixels[9] = {0,  0,  0,
                                     0,  0,  0,
                                     0,  0,  0};

#define COLORS_COUNT 5
static const uint8_t colors[3 * COLORS_COUNT] =
{
  50, 0,  0,
  0, 50,  0,
  0,  0, 50,
  50, 0, 50,
  0, 50, 50,
};

static volatile uint8_t current_color = 0;
static volatile uint8_t color_time_out = 0; // in 1/8-seconds


void update_lcd(void) {
  uint16_t lcdA = 0; // LCD Content A
  uint16_t lcdB = 0; // LCD Content B
  uint16_t dA = 0;   // LCD Content with polarity
  uint16_t dB = 0;   // LCD Content with polarity
  uint16_t i = 0;    // temp counter
  uint16_t d = 0;    // temp

  uint8_t t0, t1, t2, t3;

  switch (mode) {
    case MODE_DEFAULT:
      t0 = current_time.min0;
      t1 = current_time.min1;
      t2 = current_time.hr0;
      t3 = current_time.hr1;

      lcdA = DIGIT0A[t0] | DIGIT1A[t1] | DIGIT2A[t2];
      lcdB = DIGIT0B[t0] | DIGIT1B[t1] | DIGIT2B[t2];

      if (t3) {
        lcdA |= DIGIT3A[t3];
        lcdB |= DIGIT3B[t3];
      }

      if (current_time.frac >> 2) {
        lcdA |= 1 << 7;
      }
      break;

    case MODE_SECONDS:
      t0 = current_time.sec0;
      t1 = current_time.sec1;
      t2 = current_time.min0;
      t3 = current_time.min1;

      lcdA = DIGIT0A[t0] | DIGIT1A[t1] | DIGIT2A[t2] | DIGIT3A[t3];
      lcdB = DIGIT0B[t0] | DIGIT1B[t1] | DIGIT2B[t2] | DIGIT3B[t3];

      if (current_time.frac & 1) {
        lcdA |= 1 << 7;
      }
      break;

    case MODE_ALARM:
      t0 = alarm_time.min0;
      t1 = alarm_time.min1;
      t2 = alarm_time.hr0;
      t3 = alarm_time.hr1;

      lcdA = DIGIT0A[t0] | DIGIT1A[t1] | DIGIT2A[t2];
      lcdB = DIGIT0B[t0] | DIGIT1B[t1] | DIGIT2B[t2];

      if (t3) {
        lcdA |= DIGIT3A[t3];
        lcdB |= DIGIT3B[t3];
      }
      lcdA |= 1 << 7;
      break;
  }

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
    
    gpio_clock_high();

    d = d >> 1;
    i++;
  }

  gpio_latch_high();

  if (lcdP) {
    gpio_lcd_com_high();
  } else {
    gpio_lcd_com_low();
  }
}

void start_and_use_32k_crystal(void) {
  CPU_CCP = CCP_IOREG_gc;                      // Un-protect
  CLKCTRL_XOSC32KCTRLA = (1 << 0) | (1 << 1);  // Enable & Run Standby 32k crystal

  CPU_CCP = CCP_IOREG_gc;   // Un-protect
  CLKCTRL_MCLKCTRLA = 0x02; // Use 32k crystal for main clock

  while ((CLKCTRL_MCLKSTATUS & CLKCTRL_XOSC32KS_bm) == 0); // wait for stable 32k crystal
  while ((CLKCTRL_MCLKSTATUS & CLKCTRL_SOSC_bm) != 0); // wait for main clock switch

  CPU_CCP = CCP_IOREG_gc; // Un-protect
  CLKCTRL_MCLKCTRLB = 0;  // Disable pre-scaler
}

void use_32k_crystal(void) {
  CPU_CCP = CCP_IOREG_gc;   // Un-protect
  CLKCTRL_MCLKCTRLA = 0x02; // Use 32k crystal for main clock

  while ((CLKCTRL_MCLKSTATUS & CLKCTRL_SOSC_bm) != 0); // wait for main clock switch
}

void use_int_osc(void) {
  CPU_CCP = CCP_IOREG_gc;   // Un-protect
  CLKCTRL_MCLKCTRLA = 0x00; // Use internal 16/20Mhz osc for main clock

  while ((CLKCTRL_MCLKSTATUS & CLKCTRL_SOSC_bm) != 0); // wait for main clock switch
}

void neoPixel_output(void) {
  // WS2811 and WS2812 have different hi/lo duty cycles; this is
  // similar but NOT an exact copy of the prior 400-on-8 code.

  // 20 inst. clocks per bit: HHHHHxxxxxxxxLLLLLLL
  // ST instructions:         ^   ^        ^       (T=0, 5, 13)

  // Output is PB1
  volatile uint16_t *port = &PORTB_OUT;
  volatile uint8_t  hi   = PORTB_OUT | (1 << 1);
  volatile uint8_t  lo   = PORTB_OUT & ~(1 << 1);
  volatile uint8_t  next = lo;
  volatile uint8_t  bit  = 8;
  
  volatile uint16_t i    = 9;      // bytes total
  volatile uint8_t  *ptr = pixels; // Pointer to next byte
  volatile uint8_t  b    = *ptr++; // Current byte value

  asm volatile(
    "head20:"                   "\n\t" // Clk  Pseudocode    (T =  0)
    "st   %a[port],  %[hi]"    "\n\t" // 2    PORT = hi     (T =  2)
    "sbrc %[byte],  7"         "\n\t" // 1-2  if(b & 128)
    "mov  %[next], %[hi]"     "\n\t" // 0-1   next = hi    (T =  4)
    "dec  %[bit]"              "\n\t" // 1    bit--         (T =  5)
    "st   %a[port],  %[next]"  "\n\t" // 2    PORT = next   (T =  7)
    "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T =  8)
    "breq nextbyte20"          "\n\t" // 1-2  if(bit == 0) (from dec above)
    "rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 10)
    "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 12)
    "nop"                      "\n\t" // 1    nop           (T = 13)
    "st   %a[port],  %[lo]"    "\n\t" // 2    PORT = lo     (T = 15)
    "nop"                      "\n\t" // 1    nop           (T = 16)
    "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 18)
    "rjmp head20"              "\n\t" // 2    -> head20 (next bit out)
    "nextbyte20:"               "\n\t" //                    (T = 10)
    "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 11)
    "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 13)
    "st   %a[port], %[lo]"     "\n\t" // 2    PORT = lo     (T = 15)
    "nop"                      "\n\t" // 1    nop           (T = 16)
    "sbiw %[count], 1"         "\n\t" // 2    i--           (T = 18)
    "brne head20"             "\n"   // 2    if(i != 0) -> (next byte)
  :
    [port]  "+e" (port),
    [byte]  "+r" (b),
    [bit]   "+r" (bit),
    [next]  "+r" (next),
    [count] "+w" (i)
  :
    [ptr]    "e" (ptr),
    [hi]     "r" (hi),
    [lo]     "r" (lo)
  );
}

void output_color(void) {
  // enable NeoPixels (and wait if needed)
  if (!(PORTB_OUT & 1)) {
    PORTB_OUTSET = 1 << 0;
    _delay_us(10); // actual delay is ~1000x more
  }

  // fast speed
  cli();
  use_int_osc();

  // copy colors
  pixels[0] = colors[3 * current_color + 0];
  pixels[1] = colors[3 * current_color + 1];
  pixels[2] = colors[3 * current_color + 2];
  pixels[3] = colors[3 * current_color + 0];
  pixels[4] = colors[3 * current_color + 1];
  pixels[5] = colors[3 * current_color + 2];
  pixels[6] = colors[3 * current_color + 0];
  pixels[7] = colors[3 * current_color + 1];
  pixels[8] = colors[3 * current_color + 2];
  
  neoPixel_output();

  // slow speed
  use_32k_crystal();
}

/*
void dac_play(void) {
  // change speed
  cli();
  use_int_osc();

  // turn on DAC and select VREF
  VREF_CTRLA = 0x03; // 4.3V
  DAC0_CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm;

  uint16_t i = 0;
  for (i = 0; i < 2000; i++) {
    DAC0_DATA = 0x00;
    _delay_us(200);
    DAC0_DATA = 0xFF;
    _delay_us(200);
  }
  for (i = 0; i < 2000; i++) {
    DAC0_DATA = 0x00;
    _delay_us(300);
    DAC0_DATA = 0xFF;
    _delay_us(300);
  }

  // disable DAC
  DAC0_CTRLA = 0x00;
  VREF_CTRLA = 0x00;
  
  // restore speed
  use_32k_crystal();
  // sei(); ??
}*/

/*
void speaker_play(void) {
  // change speed
  cli();
  use_int_osc();
  
  // make output
  PORTA_DIRSET = (1 << 6);
  
  uint16_t i = 0;
  for (i = 0; i < 2000; i++) {
    PORTA_OUTCLR = (1 << 6);
    _delay_us(200);
    PORTA_OUTSET = (1 << 6);
    _delay_us(200);
  }
  for (i = 0; i < 2000; i++) {
    PORTA_OUTCLR = (1 << 6);
    _delay_us(300);
    PORTA_OUTSET = (1 << 6);
    _delay_us(300);
  }
  
  // disable output
  PORTA_DIRCLR = (1 << 6);
  
  // restore speed
  use_32k_crystal();
  sei();
} */

void enable_adc(void) {
  ADC0_INTCTRL = ADC_RESRDY_bm;                                     // Result Ready interrupt
  ADC0_MUXPOS = 0x01;                                               // PA1
  ADC0_CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_VDDREF_gc;               // Use VDD Ref
  ADC0_CTRLB = ADC_SAMPNUM_ACC64_gc;                                // Accumulate 64 samples
  ADC0_CTRLA = ADC_RESSEL_8BIT_gc | ADC_FREERUN_bm | ADC_ENABLE_bm; // Enable + Free Run
  ADC0_COMMAND = ADC_STCONV_bm;                                     // Start
}

void disable_adc(void) {
  ADC0_INTCTRL = 0;      // No interrupts
  ADC0_CTRLA = 0;        // Disable
  ADC0_INTFLAGS = 0xFF;  // Clear any interrupts
}  

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
                                     + change digits       => 18.3 uA
                                     + Idle/Standby Sleep  => ~7.0 uA
*/

ISR(RTC_PIT_vect) {
  // Clear flag
  RTC_PITINTFLAGS = 0x01;

  mytime_add_fraction(&current_time);
  update_lcd();
  
  if (color_time_out > 0) {
    color_time_out--;
    if (color_time_out == 0) {
      PORTB_OUTCLR = 1 << 0; // turn off light
      disable_adc();
    }
  }
}

ISR(PORTA_PORT_vect) {
  // clear interrupt flag
  PORTA_INTFLAGS = 1 << 2;

  if (color_time_out > 0) { // do not change color on resume
    current_color++;
    if (current_color == COLORS_COUNT) {
      current_color = 0;
    }      
  }

  output_color();
  color_time_out = 40; // 5 seconds
  enable_adc();
}

void button_press(uint8_t button) {
  if (button == 2) {
    mode++;
    if (mode == MODES_COUNT) {
      mode = 0;
    }
    return;
  }

  switch (mode) {
    case MODE_DEFAULT:
      if (button == 1) mytime_add_hour(&current_time);
      if (button == 3) mytime_add_min(&current_time);
      break;

    case MODE_SECONDS:
      current_time.sec0 = 0;
      current_time.sec1 = 0;
      current_time.frac = 0;
      break;

    case MODE_ALARM:
      if (button == 1) mytime_add_hour(&alarm_time);
      if (button == 3) mytime_add_min(&alarm_time);
      break;
  }
}

/*
  PU only      => 0xFF (4.50 V) ~20 kOhm
  1k pull down => 0x0D (0.23 V)
*/
ISR(ADC0_RESRDY_vect) {
  // clear flag
  ADC0_INTFLAGS = ADC_RESRDY_bm;

  // check results
  // NO BUTTONS: 16000
  //      MINUS:     0
  //        SET:  3800
  //       PLUS:  6200
  uint16_t r = ADC0_RES;

  if (r < 7500) {
    // key pressed

    if (r < 1000) {
      // MINUS
      button_press(1);
    } else if (r < 5500) {
      // SET
      button_press(2);
    } else {
      // PLUS
      button_press(3);
    }

    update_lcd();
    color_time_out = 40; // 5 seconds
  }
}

int main(void) {
  cli();
  disable_inputs();
  start_and_use_32k_crystal();

  // Enable RTC
  RTC_CLKSEL = 0x02; // Use Crystal/TOSC32K
  RTC_PITINTCTRL = 0x01; // Enable interrupt
  RTC_PITCTRLA = RTC_PERIOD_CYC4096_gc | RTC_PITEN_bm; // Enable, every 1/8 of a second

  // port direction
  PORTA_DIRSET = (1 << 3) | (1 << 4) | (1 << 5) | (1 << 7); // LCD
  PORTB_DIRSET = (1 << 0) | (1 << 1); // NeoPIXEL
  
  // button press interrupt (PA2)
  PORTA_PIN2CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
  
  // Pull-Up for 3-buttons (PA1)
  PORTA_PIN1CTRL = PORT_PULLUPEN_bm;

  // Enable interrupts
  sei();

  // go to sleep
  while (1) {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sleep_cpu();
  }
}
