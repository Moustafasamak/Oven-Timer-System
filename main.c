
 // ========================= PIC16F877A Oven Timer (4MHz) =========================
//moustafa samak (62200073)
//asma hasan (62220078)
#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 4000000UL

#pragma config FOSC = HS
#pragma config WDTE = OFF
#pragma config PWRTE = ON
#pragma config BOREN = ON
#pragma config LVP = OFF
#pragma config CPD = OFF
#pragma config WRT = OFF
#pragma config CP = OFF

// ================= LCD PINS (PORTC) =================
#define LCD_RS  PORTCbits.RC0
#define LCD_EN  PORTCbits.RC1
#define LCD_D4  PORTCbits.RC2
#define LCD_D5  PORTCbits.RC3
#define LCD_D6  PORTCbits.RC4
#define LCD_D7  PORTCbits.RC5

// ================= BUTTONS (PORTA) =================
#define START_MASK  0x01   // RA0
#define PAUSE_MASK  0x02   // RA1
#define RESET_MASK  0x04   // RA2

// ================= KEYPAD MAP =================
static const char keymap[4][3] = {
    {'1','2','3'},
    {'4','5','6'},
    {'7','8','9'},
    {'*','0','#'}
};

// ================= STATE / TIMER =================
typedef enum { SETTING=0, RUNNING=1, PAUSED=2, FINISHED=3 } state_t;

volatile state_t  g_state = SETTING;
volatile uint16_t g_ms = 0;
volatile uint32_t g_time_ms = 0;
volatile uint16_t g_remaining_sec = 0;
volatile uint16_t g_total_sec = 0;

volatile uint32_t finish_ms = 0;
volatile uint8_t  blink_state = 0;
volatile uint8_t  g_child_lock = 0;

// ================= TIME ENTRY (MM then SS) =================
static uint8_t mm_val = 0;      // 0..99
static uint8_t ss_val = 0;      // 0..59
static uint8_t mm_digits = 0;   // 0..2
static uint8_t ss_digits = 0;   // 0..2

static inline void entry_clear(void){
    mm_val = 0; ss_val = 0;
    mm_digits = 0; ss_digits = 0;
}

// ================= PORTB SAFE WRITE (NO RMW) =================
static volatile uint8_t g_buzzer = 0; // RB7 latch state

static inline void portb_write(uint8_t lowNibble){
    // RB0..RB3 = lowNibble (rows)
    // RB4..RB6 kept HIGH (pullups), RB7 = g_buzzer
    uint8_t v = 0x70;                 // RB4..RB6 = 1
    if(g_buzzer) v |= 0x80;           // RB7
    v |= (lowNibble & 0x0F);          // rows
    PORTB = v;                        // write full byte (no RMW)
}

static inline void rows_high(void){
    portb_write(0x0F);
}

static inline void buzzer_set(uint8_t on){
    g_buzzer = on ? 1 : 0;
    rows_high();
}

// ================= LCD =================
static void lcd_pulse(void){
    LCD_EN = 1; __delay_us(5);
    LCD_EN = 0; __delay_us(80);
}
static void lcd_send4(uint8_t nib){
    LCD_D4 = (nib >> 0) & 1;
    LCD_D5 = (nib >> 1) & 1;
    LCD_D6 = (nib >> 2) & 1;
    LCD_D7 = (nib >> 3) & 1;
    lcd_pulse();
}
static void lcd_cmd(uint8_t cmd){
    LCD_RS = 0;
    lcd_send4(cmd >> 4);
    lcd_send4(cmd & 0x0F);
    __delay_ms(2);
}
static void lcd_data(uint8_t d){
    LCD_RS = 1;
    lcd_send4(d >> 4);
    lcd_send4(d & 0x0F);
    __delay_us(60);
}
static void lcd_clear(void){
    lcd_cmd(0x01);
    __delay_ms(2);
}
static void lcd_goto(uint8_t row, uint8_t col){
    uint8_t addr = (row == 0) ? 0x80 : 0xC0;
    lcd_cmd(addr + col);
}
static void lcd_print(const char *s){
    while(*s) lcd_data((uint8_t)*s++);
}
static void lcd_init(void){
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC4 = 0;
    TRISCbits.TRISC5 = 0;

    LCD_RS = 0; LCD_EN = 0;
    __delay_ms(20);

    lcd_send4(0x03); __delay_ms(5);
    lcd_send4(0x03); __delay_us(150);
    lcd_send4(0x03); __delay_us(150);
    lcd_send4(0x02);

    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_clear();
}

static void format_mmss(uint16_t sec, char *out){
    uint16_t mm = sec / 60;
    uint16_t ss = sec % 60;
    if(mm > 99) mm = 99;

    out[0] = (mm/10) + '0';
    out[1] = (mm%10) + '0';
    out[2] = ':';
    out[3] = (ss/10) + '0';
    out[4] = (ss%10) + '0';
    out[5] = '\0';
}

static void lcd_update_screen(void){
    char t[6];
    format_mmss(g_remaining_sec, t);

    uint8_t done_show = 1;
    if(g_state == FINISHED){
        done_show = ((g_time_ms / 500UL) % 2UL) ? 1 : 0;
    }

    lcd_goto(0,0);

    if(g_state == SETTING)  lcd_print("SET   ");
    if(g_state == RUNNING)  lcd_print("RUN   ");
    if(g_state == PAUSED)   lcd_print("PAUSE ");
    if(g_state == FINISHED){
        if(done_show) lcd_print("DONE  ");
        else          lcd_print("      ");
    }

    lcd_print(t);
    lcd_print("   ");

    lcd_goto(1,0);
    if(g_child_lock){
        lcd_print("LOCKED  Hold:*  ");
    }else{
        lcd_print("Keys:set  Btn:SPR ");
    }
}

static void lcd_temp_msg(const char *l1, const char *l2, uint16_t ms){
    lcd_clear();
    lcd_goto(0,0); lcd_print(l1);
    lcd_goto(1,0); lcd_print(l2);
    __delay_ms(ms);
    lcd_clear();
    lcd_update_screen();
}

// ================= KEYPAD (FIXED) =================
static char keypad_scan(void){
    for(uint8_t r=0; r<4; r++){
        uint8_t rowMask = 0x0F;
        rowMask &= (uint8_t)~(1u << r);   // selected row LOW
        portb_write(rowMask);
        __delay_us(80);

        // COL 1 (RB4)
        if(PORTBbits.RB4 == 0){
            __delay_ms(15);
            if(PORTBbits.RB4 == 0){
                char key = keymap[r][0];

                if(key == '*'){
                    uint32_t t0 = g_time_ms;
                    while(PORTBbits.RB4 == 0){
                        if((g_time_ms - t0) >= 1200UL){
                            if(g_state == SETTING || g_state == PAUSED){
                                g_child_lock ^= 1;
                                lcd_clear(); lcd_update_screen();
                            }else{
                                lcd_temp_msg("NO LOCK", "Only SET/PAUSE", 700);
                            }
                            uint32_t tr = g_time_ms;
                            while(PORTBbits.RB4 == 0 && (g_time_ms - tr) < 1500UL);
                            rows_high();
                            return 0;
                        }
                    }
                    rows_high();
                    return '*';
                }

                uint32_t tr = g_time_ms;
                while(PORTBbits.RB4 == 0 && (g_time_ms - tr) < 1500UL);
                rows_high();
                return key;
            }
        }

        // COL 2 (RB5)
        if(PORTBbits.RB5 == 0){
            __delay_ms(15);
            if(PORTBbits.RB5 == 0){
                char key = keymap[r][1];

                if(key == '*'){
                    uint32_t t0 = g_time_ms;
                    while(PORTBbits.RB5 == 0){
                        if((g_time_ms - t0) >= 1200UL){
                            if(g_state == SETTING || g_state == PAUSED){
                                g_child_lock ^= 1;
                                lcd_clear(); lcd_update_screen();
                            }else{
                                lcd_temp_msg("NO LOCK", "Only SET/PAUSE", 700);
                            }
                            uint32_t tr = g_time_ms;
                            while(PORTBbits.RB5 == 0 && (g_time_ms - tr) < 1500UL);
                            rows_high();
                            return 0;
                        }
                    }
                    rows_high();
                    return '*';
                }

                uint32_t tr = g_time_ms;
                while(PORTBbits.RB5 == 0 && (g_time_ms - tr) < 1500UL);
                rows_high();
                return key;
            }
        }

        // COL 3 (RB6)
        if(PORTBbits.RB6 == 0){
            __delay_ms(15);
            if(PORTBbits.RB6 == 0){
                char key = keymap[r][2];

                if(key == '*'){
                    uint32_t t0 = g_time_ms;
                    while(PORTBbits.RB6 == 0){
                        if((g_time_ms - t0) >= 1200UL){
                            if(g_state == SETTING || g_state == PAUSED){
                                g_child_lock ^= 1;
                                lcd_clear(); lcd_update_screen();
                            }else{
                                lcd_temp_msg("NO LOCK", "Only SET/PAUSE", 700);
                            }
                            uint32_t tr = g_time_ms;
                            while(PORTBbits.RB6 == 0 && (g_time_ms - tr) < 1500UL);
                            rows_high();
                            return 0;
                        }
                    }
                    rows_high();
                    return '*';
                }

                uint32_t tr = g_time_ms;
                while(PORTBbits.RB6 == 0 && (g_time_ms - tr) < 1500UL);
                rows_high();
                return key;
            }
        }
    }
    rows_high();
    return 0;
}

// ================= BARGRAPH UPDATE =================
static void bargraph_update(void){
    static uint32_t last_blink = 0;

    if(g_state == FINISHED){
        if((g_time_ms - last_blink) >= 500UL){
            last_blink = g_time_ms;
            blink_state ^= 1;
        }
        if(blink_state){
            PORTCbits.RC6 = 1;
            PORTCbits.RC7 = 1;
            PORTD = 0xFF;
        }else{
            PORTCbits.RC6 = 0;
            PORTCbits.RC7 = 0;
            PORTD = 0x00;
        }
        return;
    }

    if(g_total_sec == 0){
        PORTCbits.RC6 = 0;
        PORTCbits.RC7 = 0;
        PORTD = 0x00;
        return;
    }

    uint32_t on = (uint32_t)g_remaining_sec * 10UL / (uint32_t)g_total_sec;

    PORTCbits.RC6 = (on > 0) ? 1 : 0;
    PORTCbits.RC7 = (on > 1) ? 1 : 0;

    uint8_t mask = 0;
    for(uint8_t i = 0; i < 8; i++){
        if(on > (i + 2)) mask |= (1u << i);
    }
    PORTD = mask;
}

// ================= STATUS LEDs (PORTE) =================
static void status_leds_update(void){
    static uint32_t last_step = 0;
    static uint8_t  seq = 0;

    PORTEbits.RE0 = 0;
    PORTEbits.RE1 = 0;
    PORTEbits.RE2 = 0;

    if(g_state == RUNNING){
        PORTEbits.RE0 = 1;
        return;
    }

    if(g_state == PAUSED){
        PORTEbits.RE1 = 1;
        return;
    }

    if(g_state == FINISHED){
        uint32_t elapsed = g_time_ms - finish_ms;

        if(elapsed < 5000UL){
            if((g_time_ms - last_step) >= 500UL){
                last_step = g_time_ms;
                seq = (seq + 1) % 3;
            }
            if(seq == 0) PORTEbits.RE0 = 1; // Green
            if(seq == 1) PORTEbits.RE2 = 1; // Red
            if(seq == 2) PORTEbits.RE1 = 1; // Yellow
        }else{
            PORTEbits.RE1 = 1; // Yellow steady
        }
    }
}

// ================= BUZZER UPDATE (RB7) =================
static void buzzer_update(void){
    static uint32_t last_blink = 0;
    static uint8_t  buzz_state = 0;

    if(g_state != FINISHED){
        buzzer_set(0);
        buzz_state = 0;
        return;
    }

    uint32_t elapsed = g_time_ms - finish_ms;

    if(elapsed < 5000UL){
        if((g_time_ms - last_blink) >= 500UL){
            last_blink = g_time_ms;
            buzz_state ^= 1;
        }
        buzzer_set(buzz_state);
    }else{
        buzzer_set(1);
    }
}

// ================= TIMER0 1ms (4MHz) =================
static void timer0_init_1ms(void){
    OPTION_REG = 0x04; // RBPU=0 pullups ON, T0CS=0, PSA=0, PS=1:32
    TMR0 = 225;
    INTCONbits.T0IF = 0;
    INTCONbits.T0IE = 1;
    INTCONbits.GIE  = 1;
}

void __interrupt() isr(void){
    if(INTCONbits.T0IF){
        INTCONbits.T0IF = 0;
        TMR0 = 225;

        g_ms++;
        g_time_ms++;

        if(g_ms >= 1000){
            g_ms = 0;

            if(g_state == RUNNING){
                if(g_remaining_sec > 0){
                    g_remaining_sec--;
                    if(g_remaining_sec == 0){
                        g_state = FINISHED;
                        finish_ms = g_time_ms;
                        blink_state = 0;
                    }
                }
            }
        }
    }
}

// ================= BUTTON DEBOUNCE =================
typedef struct {
    uint8_t  stable;
    uint8_t  last_raw;
    uint32_t last_change_ms;
    uint8_t  fired;
} btn_t;

static btn_t bStart = {1,1,0,0};
static btn_t bPause = {1,1,0,0};
static btn_t bReset = {1,1,0,0};

static uint8_t debounce_event(btn_t *b, uint8_t raw){
    if(raw != b->last_raw){
        b->last_raw = raw;
        b->last_change_ms = g_time_ms;
    }
    if((g_time_ms - b->last_change_ms) >= 40UL){
        if(b->stable != raw){
            b->stable = raw;
            if(b->stable == 1){
                b->fired = 0;
            }
        }
    }
    if(b->stable == 0 && b->fired == 0){
        b->fired = 1;
        return 1;
    }
    return 0;
}

static void read_buttons(uint8_t *evStart, uint8_t *evPause, uint8_t *evReset){
    uint8_t pa = PORTA;

    uint8_t rawStart = (pa & START_MASK) ? 1 : 0;
    uint8_t rawPause = (pa & PAUSE_MASK) ? 1 : 0;
    uint8_t rawReset = (pa & RESET_MASK) ? 1 : 0;

    *evStart = debounce_event(&bStart, rawStart);
    *evPause = debounce_event(&bPause, rawPause);
    *evReset = debounce_event(&bReset, rawReset);
}

// ================= MAIN =================
int main(void){
    ADCON1 = 0x07; // all digital (PORTA/PORTE)

    // Buttons inputs
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
    TRISAbits.TRISA2 = 1;

    // PORTB: RB0..RB3 out (rows), RB4..RB6 in (cols), RB7 out (buzzer)
    TRISB = 0b01110000;
    OPTION_REGbits.nRBPU = 0; // enable PORTB pullups
    g_buzzer = 0;
    rows_high();
    INTCONbits.RBIE = 0;

    // Bargraph outputs
    TRISCbits.TRISC6 = 0;
    TRISCbits.TRISC7 = 0;
    PORTCbits.RC6 = 0;
    PORTCbits.RC7 = 0;

    TRISD = 0x00;
    PORTD = 0x00;

    // Status LEDs on PORTE
    TRISEbits.TRISE0 = 0;
    TRISEbits.TRISE1 = 0;
    TRISEbits.TRISE2 = 0;
    PORTEbits.RE0 = 0;
    PORTEbits.RE1 = 0;
    PORTEbits.RE2 = 0;

    lcd_init();
    timer0_init_1ms();

    lcd_clear();
    lcd_goto(0,0); lcd_print("OVEN TIMER");
    lcd_goto(1,0); lcd_print("READY 4MHz...");
    __delay_ms(700);
    lcd_clear();

    g_remaining_sec = 0;
    g_total_sec = 0;
    g_state = SETTING;
    g_child_lock = 0;
    entry_clear();

    lcd_update_screen();

    while(1){
        // Buttons
        uint8_t evS=0, evP=0, evR=0;
        read_buttons(&evS, &evP, &evR);

        if(!g_child_lock){
            if(evR){
                g_remaining_sec = 0;
                g_total_sec = 0;
                g_state = SETTING;

                blink_state = 0;
                finish_ms = 0;
                buzzer_set(0);

                entry_clear();

                lcd_clear();
                lcd_update_screen();
            }

            if(evS){
                if(g_remaining_sec > 0){
                    g_state = RUNNING;
                    lcd_clear();
                    lcd_update_screen();
                }
            }

            if(evP){
                if(g_state == RUNNING) g_state = PAUSED;
                else if(g_state == PAUSED) g_state = RUNNING;

                lcd_clear();
                lcd_update_screen();
            }
        }

        // Keypad
        char k = keypad_scan();

        // Ignore short '*' and '#'
        if(k == '*' || k == '#') k = 0;

        // Digits only (blocked when locked)
        if(k && !g_child_lock){
            if(k >= '0' && k <= '9'){
                if(g_state == SETTING || g_state == PAUSED){

                    uint8_t d = (uint8_t)(k - '0');

                    // If already full MMSS, start a NEW entry
                    if(mm_digits == 2 && ss_digits == 2){
                        entry_clear();
                    }

                    // Enter MM first
                    if(mm_digits == 0){
                        mm_val = (uint8_t)(d * 10u);   // tens of minutes
                        mm_digits = 1;
                    }else if(mm_digits == 1){
                        mm_val = (uint8_t)(mm_val + d); // ones of minutes
                        mm_digits = 2;
                    }
                    // Then enter SS
                    else if(ss_digits == 0){
                        if(d > 5u){
                            lcd_temp_msg("Invalid Time", "SS 00..59", 400);
                        }else{
                            ss_val = (uint8_t)(d * 10u); // tens of seconds
                            ss_digits = 1;
                        }
                    }else if(ss_digits == 1){
                        ss_val = (uint8_t)(ss_val + d); // ones of seconds
                        ss_digits = 2;
                    }

                    // Update displayed/working time (even while entering)
                    g_remaining_sec = (uint16_t)((uint16_t)mm_val * 60u + (uint16_t)ss_val);
                    if(mm_digits == 2 && ss_digits == 2){
                        g_total_sec = g_remaining_sec; // lock total after full entry
                    }

                    lcd_clear();
                    lcd_update_screen();
                }
            }
        }

        __delay_ms(100);
        lcd_update_screen();
        bargraph_update();
        status_leds_update();
        buzzer_update();
    }
}

