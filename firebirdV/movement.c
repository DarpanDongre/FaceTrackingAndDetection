#define F_CPU 14745600UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

#define BAUD      115200UL
#define UBRR_VAL  ((F_CPU / (16UL * BAUD)) - 1)

/* ==== SENSORS ==== */
#define SHARP_FRONT_CH   11 
#define PROX_CH_4        7   /* Right side IR */

#define SHARP_STOP_CM    15  /* Safe trigger distance */
#define RIGHT_WALL_THRES 100 /* Below this means wall is present, above means open air */

/* ==== TIMING ==== */
#define TURN_BURST_MS   200  /* Reduced from 300 for smaller, tighter tracking turns */
#define PAUSE_MS        150  

#define AVOID_REV_MS    400  /* Clearance backup */
#define AVOID_TURN90_MS 450  /* Approx 90 degree turn (used for Left) */
#define AVOID_TURN_RIGHT_MS 300 /* NEW: Specific shorter timer for the final right turn */
#define AVOID_SAFETY_MS 600  /* Extra forward push to clear back wheels */

/* ==== AVOIDANCE STATES ==== */
#define ST_IDLE          0
#define ST_REVERSE       1
#define ST_TURN_LEFT     2
#define ST_WALL_FOLLOW   3
#define ST_SAFETY_FWD    4
#define ST_TURN_RIGHT    5

/* ==== LCD ==== */
#define RS 0
#define RW 1
#define EN 2
#define sbit(r,b) ((r) |=  (1<<(b)))
#define cbit(r,b) ((r) &= ~(1<<(b)))

static void lcd_pulse(void) {
    sbit(PORTC, EN); _delay_us(500); cbit(PORTC, EN); _delay_us(500);
}
void lcd_port_config(void) { DDRC |= 0xF7; PORTC &= 0x80; }

void lcd_cmd(uint8_t c) {
    PORTC = (PORTC & 0x0F) | (c & 0xF0);
    cbit(PORTC,RS); cbit(PORTC,RW); lcd_pulse();
    PORTC = (PORTC & 0x0F) | ((c<<4) & 0xF0);
    cbit(PORTC,RS); cbit(PORTC,RW); lcd_pulse();
}
void lcd_char(char c) {
    PORTC = (PORTC & 0x0F) | (c & 0xF0);
    sbit(PORTC,RS); cbit(PORTC,RW); lcd_pulse();
    PORTC = (PORTC & 0x0F) | ((c<<4) & 0xF0);
    sbit(PORTC,RS); cbit(PORTC,RW); lcd_pulse();
}
void lcd_init(void) {
    _delay_ms(20);
    PORTC=(PORTC&0x0F)|0x30; cbit(PORTC,RS); cbit(PORTC,RW); lcd_pulse();
    PORTC=(PORTC&0x0F)|0x30; cbit(PORTC,RS); cbit(PORTC,RW); lcd_pulse();
    PORTC=(PORTC&0x0F)|0x20; cbit(PORTC,RS); cbit(PORTC,RW); lcd_pulse();
    lcd_cmd(0x28); lcd_cmd(0x01); _delay_ms(2);
    lcd_cmd(0x06); lcd_cmd(0x0C); lcd_cmd(0x80);
}
void lcd_clear(void) { lcd_cmd(0x01); _delay_ms(2); }
void lcd_pos(uint8_t row, uint8_t col) {
    lcd_cmd(row == 1 ? (0x80+col-1) : (0xC0+col-1));
}
void lcd_str(const char *s) { while(*s) lcd_char(*s++); }

/* ==== MOTORS ==== */
void motion_pin_config(void) {
    DDRA  |= 0x0F; PORTA &= 0xF0;
    DDRL  |= 0x18; PORTL |= 0x18;
    TCCR5A = 0xA9; TCCR5B = 0x0B; 
    OCR5AL = 255;  OCR5BL = 255;
}
static inline void motor_forward(void)  { PORTA = (PORTA & 0xF0) | 0x06; }
static inline void motor_reverse(void)  { PORTA = (PORTA & 0xF0) | 0x09; }
static inline void motor_left(void)     { PORTA = (PORTA & 0xF0) | 0x05; }
static inline void motor_right(void)    { PORTA = (PORTA & 0xF0) | 0x0A; }
static inline void motor_stop(void)     { PORTA = (PORTA & 0xF0) | 0x00; }
static inline void set_turn_speed(void) { OCR5AL = 150; OCR5BL = 150; } /* Reduced from 170 for smoother tracking turns */
static inline void set_full_speed(void) { OCR5AL = 255; OCR5BL = 255; }

/* ==== ADC ==== */
void adc_pin_config(void) { DDRF=0; PORTF=0; DDRK=0; PORTK=0; }
void adc_init(void) {
    ADCSRA=0; ADCSRB=0; ADMUX=0x20; ACSR=0x80; ADCSRA=0x86;
}
unsigned char ADC_Conversion(unsigned char Ch) {
    if(Ch>7) ADCSRB=0x08;
    Ch &= 0x07;
    ADMUX = 0x20|Ch;
    ADCSRA |= 0x40;
    while((ADCSRA & 0x10)==0);
    unsigned char a = ADCH;
    ADCSRA |= 0x10; ADCSRB=0;
    return a;
}
unsigned int Sharp_GP2D12_Estimation(unsigned char adc_reading) {
    if(adc_reading < 3) adc_reading = 3;
    unsigned int dist_mm = (unsigned int)(10.0f * (2799.6f * powf((float)adc_reading, -1.1546f)));
    return (dist_mm > 800) ? 800 : dist_mm;
}
void sensors_power_on(void) {
    DDRH  |= 0x0C; PORTH &= ~0x0C;
    DDRG  |= 0x04; PORTG &= ~0x04;
    _delay_ms(100);
}

/* ==== UART ==== */
volatile uint8_t g_pi_cmd   = 'S';
volatile uint8_t g_cmd_new  = 0;

ISR(USART2_RX_vect) {
    uint8_t b = UDR2;
    if(b=='F'||b=='L'||b=='R'||b=='l'||b=='r'||b=='S'){
        g_pi_cmd  = b;
        g_cmd_new = 1;
    }
    wdt_reset();
}
void uart2_init(void) {
    UCSR2B=0; UCSR2A=0; UCSR2C=0x06;
    UBRR2H = (uint8_t)(UBRR_VAL>>8); UBRR2L = (uint8_t)(UBRR_VAL);
    UCSR2B = (1<<RXCIE2)|(1<<RXEN2)|(1<<TXEN2);
}

/* ==== MAIN ==== */
static uint32_t g_ms = 0;

int main(void) {
    MCUSR = 0; wdt_disable();
    cli();
    lcd_port_config(); motion_pin_config();
    adc_pin_config(); adc_init(); uart2_init();
    sei();

    wdt_enable(WDTO_1S);
    sensors_power_on();
    lcd_init(); lcd_clear();

    uint8_t  turning = 0, in_pause = 0;
    uint32_t turn_start_ms = 0, pause_start_ms = 0;
    uint8_t  committed_cmd = 'S';

    uint8_t  obst_state = ST_IDLE;
    uint32_t obst_t0 = 0;
    
    uint8_t lcd_div = 0;
    char lcd1[17], lcd2[17];

    while (1) {
        wdt_reset();
        _delay_ms(1);
        g_ms++;

        if(g_cmd_new){
            committed_cmd = g_pi_cmd;
            g_cmd_new = 0;
        }

        unsigned int front_cm = Sharp_GP2D12_Estimation(ADC_Conversion(SHARP_FRONT_CH)) / 10;
        unsigned char right_prox = ADC_Conversion(PROX_CH_4);

        if (committed_cmd == 'S') {
            motor_stop();
            obst_state = ST_IDLE; 
            turning = 0; 
            in_pause = 0;
        } 
        else {
            if (obst_state == ST_IDLE) {
                if (front_cm <= SHARP_STOP_CM) {
                    motor_stop();
                    turning = 0; in_pause = 0;
                    obst_state = ST_REVERSE;
                    obst_t0 = g_ms;
                } else {
                    if(committed_cmd == 'L' || committed_cmd == 'R' || committed_cmd == 'l' || committed_cmd == 'r') {
                        if(in_pause) {
                            motor_stop(); set_full_speed();
                            if(g_ms - pause_start_ms >= PAUSE_MS) in_pause = 0;
                        } else if(turning) {
                            if(g_ms - turn_start_ms >= TURN_BURST_MS) {
                                motor_stop(); set_full_speed();
                                turning = 0; in_pause = 1; pause_start_ms = g_ms;
                            } else {
                                set_turn_speed();
                                if(committed_cmd == 'L' || committed_cmd == 'l') motor_left(); else motor_right();
                            }
                        } else {
                            turning = 1; turn_start_ms = g_ms;
                            set_turn_speed();
                            if(committed_cmd == 'L' || committed_cmd == 'l') motor_left(); else motor_right();
                        }
                    } else if(committed_cmd == 'F') {
                        turning = 0; in_pause = 0;
                        set_full_speed(); motor_forward();
                    }
                }
            } 
            else {
                switch (obst_state) {
                    
                    case ST_REVERSE:
                        set_full_speed();
                        motor_reverse();
                        if(g_ms - obst_t0 >= AVOID_REV_MS) {
                            motor_stop();
                            obst_state = ST_TURN_LEFT;
                            obst_t0 = g_ms;
                        }
                        break;

                    case ST_TURN_LEFT:
                        set_turn_speed();
                        motor_left();
                        if(g_ms - obst_t0 >= AVOID_TURN90_MS) {
                            motor_stop();
                            obst_state = ST_WALL_FOLLOW;
                        }
                        break;

                    case ST_WALL_FOLLOW:
                        set_full_speed();
                        motor_forward();
                        if(right_prox > RIGHT_WALL_THRES) {
                            motor_stop();
                            obst_state = ST_SAFETY_FWD;
                            obst_t0 = g_ms;
                        }
                        break;

                    case ST_SAFETY_FWD:
                        set_full_speed();
                        motor_forward();
                        if(g_ms - obst_t0 >= AVOID_SAFETY_MS) {
                            motor_stop(); 
                            obst_state = ST_TURN_RIGHT;
                            obst_t0 = g_ms;
                        }
                        break;
                        
                    case ST_TURN_RIGHT:
                        set_turn_speed();
                        motor_right();
                        if(g_ms - obst_t0 >= AVOID_TURN_RIGHT_MS) {
                            motor_stop(); 
                            obst_state = ST_IDLE;
                        }
                        break;
                }
            }
        }

        lcd_div++;
        if(lcd_div >= 200) {
            lcd_div = 0;
            snprintf(lcd1, 16, "F:%2u R:%3u C:%c ", front_cm, right_prox, (char)committed_cmd);
            
            if(committed_cmd == 'S') {
                snprintf(lcd2,16,"*** HALTED *** ");
            }
            else if(obst_state != ST_IDLE) {
                switch(obst_state) {
                    case ST_REVERSE:     snprintf(lcd2,16,"AVOID REVERSE   "); break;
                    case ST_TURN_LEFT:   snprintf(lcd2,16,"AVOID TURN L    "); break;
                    case ST_WALL_FOLLOW: snprintf(lcd2,16,"AVOID WALL FLW  "); break;
                    case ST_SAFETY_FWD:  snprintf(lcd2,16,"AVOID SFTY FWD  "); break;
                    case ST_TURN_RIGHT:  snprintf(lcd2,16,"AVOID TURN R    "); break;
                }
            } else if(in_pause) {
                snprintf(lcd2, 16, "ANTI SPIN PAUSE ");
            } else {
                switch(committed_cmd) {
                    case 'F': snprintf(lcd2,16,"TRACKING FWD    "); break;
                    case 'L': snprintf(lcd2,16,"TRACKING LEFT   "); break;
                    case 'R': snprintf(lcd2,16,"TRACKING RIGHT  "); break;
                    case 'l': snprintf(lcd2,16,"TRACKING SOFT L "); break;
                    case 'r': snprintf(lcd2,16,"TRACKING SOFT R "); break;
                }
            }
            lcd_pos(1,1); lcd_str(lcd1);
            lcd_pos(2,1); lcd_str(lcd2);
        }
    }
    return 0;
}