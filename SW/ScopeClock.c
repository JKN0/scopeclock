/***************************************************************************
 
 ScopeClock.c
 
 Kellon n‰yttˆ oskilloskoopissa. Ei-7-segment fontit.
 
 Kontrolleri C8051F330.
  
 18.2.2010

 15.9.2010 - DS1307 mukaan

 17.10.2010 - DS1307 viel‰ kommentteihin, Nokian kaukos‰‰din Saloran tilalle.

 9.3.2011 - DS1307 taas mukaan.

 13.3.2011 - Lopullinen HW.
 
 ***************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <datatypes.h>
#include <C8051F330.h>
#include "i2c_F330.h"
#include "fonts.h"

/* =====================================================================
------------------------ Vakiot ja makrot --------------------------- */

/* --- Ajastimen vakiot */
/* Kellotaajuus 24.5 MHz = SYSCLK ==> konejakso 41 ns
   Timer 2 laskee SYSCLK = 24.5 MHz  
   Kun juovakeskeytysv‰li 2 ms ==> reload = -49000 = 0x4098
*/
#define TMR2_RELOAD          0x4098 
#define RTC_PRESCALER        250            // 250 * 2 ms = 0.5 s

#define IR_PULSES            13

#define MAX_DIGIT            6              // number of digits to be displayed

#define BLANK                10             // ' '
#define CURSOR               11             // '_'
#define COLON                12             // ':'

#define S_IDLE               0
#define S_START              1
#define S_RECV               2

#define IN0PL                0x08

#define NONE                 0xFF

/* =======================================================================
------------------------ I/O ------------------------------------------ */

#define led1                 P1_1
#define trigger              P1_7
#define IR_DATA              P0_3
#define PIN_1HZ              P0_7

/* =====================================================================
------------------------ Tietotyypit -------------------------------- */

/* =====================================================================
------------------------ Globaalit muuttujat ------------------------ */

#define CLOCK_I2C_ADDR        0xD0

// numeroiden slotit PCA-positioina
const WORD slot_len[] = {
    200,    // sync - 10h
    630,    // 10h - 1h
    700,    // 1h - :
    300,    // : - 10m
    650,    // 10m - 1m
    700,    // 1m - :
    300,    // : - 10s
    650,    // 10s - 1s
    0       // end-of-table
};

// N‰yttˆpuskurin positiot
#define H10     0       // kymmenet tunnit
#define H1      1       // ykkˆstunnit
#define C1      2       // :
#define M10     3       // kymmenet minuutit
#define M1      4       // ykkˆsminuutit
#define C2      5       // :
#define S10     6       // kymmenet sekunnit
#define S1      7       // ykkˆssekunnit

//                      h   h    :    m  m    :    s  s
//                      0   1    2    3  4    5    6  7
BYTE disp_buf[8] = { BLANK, 0, COLON, 0, 0, COLON, 0, 0 }; 

CODE BYTE digit[] = {   0,  1,        3, 4,        6, 7 };

BYTE   ss = 0, mn = 0, hh = 0 ;        // RTC

BYTE cur_line = 0;
BIT setup_mode = FALSE;
WORD next_pos = 0;
BYTE next_slot = 0;

BYTE ir_cmd = NONE;
BIT volatile ir_received = FALSE;

BIT waiting_edge = FALSE;
BYTE bit_ctr = 0;
BYTE ir_state = S_IDLE;
BYTE ir_code = 0;

BYTE offset = 100;
char offs_dir = 1;

const BYTE max_nr[] = { 2,9,5,9,5,9 };      // asetusarvojen validointi

IDATA BYTE i2c_inbuf[5];
IDATA BYTE i2c_outbuf[5];

/* ------------------------------------------------------------------ */
/* --- IR-m‰‰rittelyt --- */

/* IR-bittien pituuksien raja-arvot:
      esipulssi = 3 ms
      bitin kesto = 1 ms
      
    Timer0 laskee SYSCLK/4 = 6.125 MHz joten 1 us = 6.125 cycles */

#define CYC_IN_US            6.125                          // cycles/us
#define IR_TIMEOUT           (-((int)(CYC_IN_US*4000)))     // timeout 4 ms

#define PREPULSE_LEN         ((int)(CYC_IN_US*3000))        // pre-pulse 3 ms
#define PREPULSE_LOLIM       ((int)(1.1*(IR_TIMEOUT+PREPULSE_LEN)))     // +10%
#define PREPULSE_HILIM       ((int)(0.9*(IR_TIMEOUT+PREPULSE_LEN)))     // -10%
// arvot negatiivisia, joten lo- ja hi-arvot ovat todella noin p‰in!

#define BITLEN               ((int)(CYC_IN_US*1000))        // bitin pituus 1 ms
#define BITLEN_5_4           (-((int)(1.25*BITLEN)))        // 5/4*bitin pituus 
#define BITLEN_3_4           (-((int)(0.75*BITLEN)))        // 3/4*bitin pituus 

#define START_STOP          0xFE    // painalluksen aloitus/lopetuskoodi

#define IRCMD_START         0x2D    // painike "info"
#define IRCMD_ENTER         0x30    // painike "ok"
#define IRCMD_INIT          0x55    // painike "opt"
// numerot 0...9 = 0x00 ... 0x09

/* =====================================================================
------------------------ Funktioiden prototyypit -------------------- */

void rtc_task( void );
void display_task( void );
void setup_task( void );
void ir_recv_task( void );
void init_hw( void );
void Init_Device(void);

/* =====================================================================
P‰‰ohjelma
--------------------------------------------------------------------- */

void main( void )
{
    PCA0MD &= ~0x40;                   // WDTE = 0 (clear watchdog timer enable)

    /* Laitteiston alustukset */
    init_hw();
    
    ENABLE(EA);          // Keskeytysten sallinta

    for(;;)                                                 // main loop
    {
        rtc_task();
        display_task();
        setup_task();
        ir_recv_task();

        led1 = PIN_1HZ;
    }
           
}

/* =======================================================================
----------------------------------------------------------------------- */

#define S_START_READ    0
#define S_WAIT_READ     1
#define S_DONE          2

void rtc_task( void )
{
    static BYTE state = S_START_READ;

    switch (state)
    {
    case S_START_READ:
        i2c_read_sub(CLOCK_I2C_ADDR,0,i2c_inbuf,3);
        state = S_WAIT_READ;
        break;
    
    case S_WAIT_READ:
        if (!i2c_busy)
        {
            DISABLE(EX1);
            ss = (10 * (i2c_inbuf[0] >> 4)) + (i2c_inbuf[0] & 0x0F);
            mn = (10 * (i2c_inbuf[1] >> 4)) + (i2c_inbuf[1] & 0x0F);
            hh = (10 * (i2c_inbuf[2] >> 4)) + (i2c_inbuf[2] & 0x0F);
            ENABLE(EX1);

            state = S_DONE;     // tilakone seis
        }
        break;

    }
}

/* =======================================================================
N‰ytˆn yll‰pito
----------------------------------------------------------------------- */

void display_task( void )
{
    if (setup_mode)
    {
         disp_buf[C2] = COLON;
    }
    else
    {
        if (PIN_1HZ)                        // kaksoispisteen vilkutus 1 Hz
            disp_buf[C2] = BLANK;
        else
            disp_buf[C2] = COLON;

        /*
         * prepare display buffer
         */
        DISABLE(EX1);

        disp_buf[S1]  = ss % 10 ;                          // seconds
        disp_buf[S10] = ss / 10 ;

        disp_buf[M1]  = mn % 10 ;                          // minutes
        disp_buf[M10] = mn / 10 ;

        disp_buf[H1]  = hh % 10 ;                          // hours
        disp_buf[H10] = (hh > 9) ? hh / 10 : 10 ;          // blank first digit if zero

        ENABLE(EX1);
    }
}

/* =======================================================================
Kellon asetus IR-kaukos‰‰timell‰.
----------------------------------------------------------------------- */

void setup_task( void )
{
    static BYTE cur_digit = 0;
    BYTE i;

    if (ir_cmd != NONE)
    {
        // Start-komento
        if (ir_cmd == IRCMD_START)
        {
            setup_mode = TRUE;

            cur_digit = 0;

            for(i = 0 ; i < MAX_DIGIT ; i++)                // for each digit
            {
                disp_buf[digit[i]] = CURSOR;
            }
        }

        // Muut komennot hyv‰ksyt‰‰n vain setup-moodissa
        if (setup_mode)
        {
            // Enter-komento
            if (ir_cmd == IRCMD_ENTER)
            {
                for(i = cur_digit ; i < MAX_DIGIT ; i++)    // zero rest of digits
                {
                    disp_buf[digit[i]] = 0;
                }
                
                // Aika muuttujiin
                hh = 10*disp_buf[H10] + disp_buf[H1];
                mn = 10*disp_buf[M10] + disp_buf[M1];
                ss = 10*disp_buf[S10] + disp_buf[S1];

                // Aika RTC-piirille
                i2c_outbuf[0] = (disp_buf[S10] << 4) | disp_buf[S1];
                i2c_outbuf[1] = (disp_buf[M10] << 4) | disp_buf[M1];
                i2c_outbuf[2] = (disp_buf[H10] << 4) | disp_buf[H1];
                i2c_write_sub(CLOCK_I2C_ADDR,0,i2c_outbuf,3);

                setup_mode = FALSE;
            }
            else if (ir_cmd < 10)   // numerot 0...9
            {
                if (cur_digit < MAX_DIGIT)
                {
                    if (ir_cmd <= max_nr[cur_digit])
                    {
                        disp_buf[digit[cur_digit]] = ir_cmd;
                        cur_digit++;
                    }
                }
            }
        }

        // Init-komento: kirjoittaa control-tavun RTC-piirille
        if (ir_cmd == IRCMD_INIT)
        {
            i2c_outbuf[0] = 0x10;                   // enable 1 Hz sq wave
            i2c_write_sub(CLOCK_I2C_ADDR,7,i2c_outbuf,1);
        }

        ir_cmd = NONE;      // komento k‰sitelty
    }
}

/* =======================================================================
Muodostaa keskeytysten vastaanottamasta ir_codesta ir_cmd:n siten, ett‰
ir_cmd asettuu vain kerran per kaukos‰‰timen painallus.
----------------------------------------------------------------------- */

void ir_recv_task( void )
{
    static BYTE prev_code = NONE;

    if (ir_received && ir_cmd == NONE)
    {
        /* Saatiin uusi IR-koodi */
        ir_received = FALSE;
        
        /* Koodi START_STOP (0xFE) on alku/loppumerkki. Se nollaa prev_code:n, 
           joten seuraavaksi hyv‰ksyt‰‰n taas mik‰ tahansa koodi. */
        if (ir_code == START_STOP)
            prev_code = NONE;
        else
        {
            if (ir_code != prev_code)
                ir_cmd = ir_code;

            prev_code = ir_code;
        }
    }
}

/* =======================================================================
Alustaa laitteiston: I/O-portit, laskurit ym.
----------------------------------------------------------------------- */

void init_hw( void )
{
    /* Kutsutaan ConfigWizardin generoimaa alustusrutiinia */
    Init_Device();

    /* Timerien alustukset */
    TMR2      = TMR2_RELOAD;
    TMR2RL    = TMR2_RELOAD;
    
    trigger = 0;
}

/* ======================================================================= */
/* =====================       KESKEYTYSPALVELUT      ==================== */
/* ======================================================================= */

/* =======================================================================
INT1-keskeytys, aiheuttajana reuna kellopiirilt‰ 1 s v‰lein.

Teht‰v‰t:
 - ajan yll‰pito

----------------------------------------------------------------------- */

void rtc_isr(void) __interrupt(IE1_VECTOR) __using (1) 
{
    ss++ ;                          // next second
    if(ss == 60)                    // last second in minute ?
    {
        ss = 0 ;                    // clear second
        mn++ ;                      // next minute
        if(mn == 60)                // last minute in hour ?
        {
            mn = 0 ;                // clear minute
            hh++ ;                  // next hour
            if(hh == 24)            // last hour in day ?
            {
                hh = 0 ;            // clear hour
            }
        }
    }

    offset += offs_dir;

    if (offset > 210)
        offs_dir = -1;
    else if (offset < 10)
        offs_dir = 1;
}

/* =======================================================================
Juovan aloituskeskeytys: Timer2, 2 ms v‰lein. 
- Muodostaa 5 V tahdistuspulssin juovan alkuun.
- Alustaa juovan piirron.
----------------------------------------------------------------------- */

void line_isr(void) __interrupt(TF2_VECTOR) __using (1) 
{
    TF2H = 0;

    /* Tahdistuspiikki juovan alkuun. Asettamalla trigger=1 
       l‰htˆ nousee 5 V:iin. */
    IDA0H = 0;
    trigger = 1;
    
    // Seuraava juova k‰sittelyyn (syklinen kasvatus)
    cur_line = (cur_line + 1) & 0x03;

    // Luetaan PCA-laskuri talteen ja alustetaan PCA:n moduuli 2
    next_pos = PCA0 + slot_len[0] + offset;
    PCA0CP2 = next_pos;
    next_slot = 1;

    CCF2 = 0;
    PCA0CPM2 |= ECCF;               // PCA mod 2 keskeytyksen sallinta

    trigger = 0;
}

/* =======================================================================
PCA:n keskeytyspalvelu. Aiheuttaja ja teht‰v‰t:

  CCF0: ei k‰ytˆss‰.

  CCF1: ei k‰ytˆss‰.

  CCF2: Moduuli 2 on havainnut PCA-laskurin p‰‰sseen kohtaan, jossa pit‰‰
        aloittaa seuraavan merkin piirt‰minen.

----------------------------------------------------------------------- */

void pca_isr(void) __interrupt(PCA0_VECTOR) __using (1) 
{
    CODE BYTE *p;

    if (CCF0)
    {
        /* Moduulin 0 keskeytys: ei k‰ytˆss‰. */
        CCF0 = 0;
    }
    
    if (CCF1)
    {
        /* Moduulin 1 keskeytys: ei k‰ytˆss‰. */
        CCF1 = 0;
    }

    if (CCF2)
    {
        /* Moduulin 2 keskeytys: seuraava merkki piirrett‰v‰ksi */
        CCF2 = 0;

        // Merkin piirto
        p = font_tbl[cur_line][disp_buf[next_slot-1]];
        do 
        {
            IDA0H = *p;
        } 
        while (*p++ != 0);

        if (slot_len[next_slot] == 0)
        {
            // rivin kaikki slotit k‰yty l‰pi
            PCA0CPM2 &= ~ECCF;  // PCA mod 2 keskeytyksen kielto
        }
        else
        {   // alustetaan seuraava keskeytys
            next_pos += slot_len[next_slot];
            PCA0CP2 = next_pos;
            next_slot++;
        }
    }
}

/* =======================================================================
INT0-keskeytys, aiheuttajana reuna IR-vastaanottimelta.

Teht‰v‰t:
 - IR-koodin vastaanotto

----------------------------------------------------------------------- */

void ir_edge_isr(void) __interrupt(IE0_VECTOR) __using (1) 
{
    switch (ir_state)
    {
    case S_IDLE:
        /* Lepotilassa reuna on esipulssin aloitus. */

        // alustetaan timer 0 vahtimaan, tuleeko start-pulssi ajoissa
        TMR0 = IR_TIMEOUT;
        TR0 = 1;
        ENABLE(ET0);
        waiting_edge = TRUE;
        
        // siirryt‰‰n odottamaan start-pulssia
        ir_state = S_START;
        break;   
        
    case S_START:
        /* Odotetaan start-pulssia, joten reuna on sen aloitus. */

        // tarkistetaan esipulssin pituus
        TR0 = 0;
        if (TMR0 > PREPULSE_LOLIM && TMR0 < PREPULSE_HILIM)
        {
            // esipulssin kesto ok, aletaan ottaa vastaan databittej‰
            // timer 0 keskeytt‰m‰‰n 5/4 bitin kuluttua, jolloin ollaan
            // keskell‰ ensimm‰isen databitin alkupuoliskoa (ohitetaan 
            // siis start-bitti)
            TMR0 = BITLEN_5_4;
            TR0 = 1;            // timer0 k‰yntiin
            ENABLE(ET0);        // timer0-keskeytys sallittu
            DISABLE(EX0);       // INT0 kielletty (reunat ei nyt kiinnosta)
            waiting_edge = FALSE;   // ei olla odottamassa reunaa
            
            bit_ctr = 0;    // vastaanotettujen bittien laskuri
            ir_code = 0;    // vastaanotetut bitit
            
            // siirryt‰‰n odottamaan bitin keskell‰ olevaa reunaa.
            ir_state = S_RECV;
        }
        else
        {   // kelvoton esipulssi, palataan lepotilaan
            IT01CF &= ~IN0PL;   // INT0 reagoimaan laskevaan reunaan
            IE0 = 0;            // mahdollinen odottava INT0 pois
            ENABLE(EX0);        // INT0 sallittu
            DISABLE(ET0);       // timer0-keskeytys kielletty
            TR0 = 0;            // timer0 seis
            ir_state = S_IDLE;
        }
        break;   
        
    case S_RECV:
        /* Vastaanottotilassa reuna on bitin keskell‰ oleva reuna. 
           Synkronoidaan timer 0 siihen. */

        // timer 0 keskeytt‰m‰‰n 3/4 bitin kuluttua, jolloin ollaan
        // keskell‰ seuraavan databitin alkupuoliskoa 
        TMR0 = BITLEN_3_4;
        TR0 = 1;            // timer0 k‰yntiin
        ENABLE(ET0);        // timer0-keskeytys sallittu
        DISABLE(EX0);       // INT0 kielletty (reunat ei nyt kiinnosta)
        waiting_edge = FALSE;   // ei olla odottamassa reunaa
        break;   
    }
}

/* =======================================================================
Timer0-keskeytys.

Teht‰v‰t:
 - IR-koodin ajoitukset

----------------------------------------------------------------------- */

void ir_timer_isr(void) __interrupt(TF0_VECTOR) __using (1) 
{
    BIT ir_bit;
    
    TF0 = 0;
    
    if (waiting_edge)
    {
        /* Odotetaan reunaa, jota ei n‰kˆj‰‰n tullut, koska timer ehti laueta.
           => Pulssijono keskeytynyt, joten palataan lepotilaan. */

        IT01CF &= ~IN0PL;   // INT0 reagoimaan laskevaan reunaan
        IE0 = 0;            // mahdollinen odottava INT0 pois
        ENABLE(EX0);        // INT0 sallittu
        DISABLE(ET0);       // timer0 kielletty
        TR0 = 0;            // timer0 seis
        ir_state = S_IDLE;
    }
    else
    {   /* Ollaan bitin alkupuoliskon keskell‰, luetaan bitti talteen
           ja aletaan odottaa bitin puoliv‰liss‰ olevaa reunaa. */

        // bitin arvo talteen ir_code:n ylimp‰‰n bittiin
        ir_bit = !IR_DATA;

        if (bit_ctr < 8)    // vain 8 alinta bitti‰ talteen, ylemm‰t hukataan
            ir_code = (ir_bit << 7) | (ir_code >> 1);

        bit_ctr++;
        if (bit_ctr > 15)
        {
            // koko bittijono vastaanotettu, lepotilaan
            ir_received = TRUE;
            
            IT01CF &= ~IN0PL;   // INT0 reagoimaan laskevaan reunaan
            IE0 = 0;            // mahdollinen odottava INT0 pois
            ENABLE(EX0);        // INT0 sallittu
            DISABLE(ET0);       // timer0 kielletty
            TR0 = 0;            // timer0 seis
            ir_state = S_IDLE;
        }
        else
        {   // vastaanotto jatkuu, aletaan odottaa bitin keskell‰ olevaa reunaa
            // (bitin keskell‰ on aina jonkun suuntainen reuna)
            if (ir_bit)
                IT01CF |= IN0PL;    // INT0 reagoimaan nousevaan reunaan
            else
                IT01CF &= ~IN0PL;   // INT0 reagoimaan laskevaan reunaan

            IE0 = 0;                // mahdollinen odottava INT0 pois
            ENABLE(EX0);            // INT0 sallittu
            
            // alustetaan timer 0 vahtimaan, tuleeko reuna ajoissa
            TMR0 = IR_TIMEOUT;
            TR0 = 1;
            ENABLE(ET0);
            waiting_edge = TRUE;
        }
    }
}

// ConfigWizardilla generoitu initialisointitiedosto mukaan
#include "sc_init.c"

/* ============================ EOF ====================================== */
