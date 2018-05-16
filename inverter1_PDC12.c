// caries frequancy 2300 hz

#include <30F2010.h>

#FUSES NOWDT                    //No Watch Dog Timer
//#FUSES XT_PLL8                 //XT Crystal Oscillator mode with 16X PLL
#FUSES HS
#FUSES PR                       //Primary Oscillator
#FUSES NOCKSFSM                 //Clock Switching is disabled, fail Safe clock monitor is disabled
#FUSES WPSB16                   //Watch Dog Timer PreScalar B 1:16
#FUSES WPSA512                  //Watch Dog Timer PreScalar A 1:512
#FUSES PUT64                    //Power On Reset Timer value 64ms
#FUSES NOBROWNOUT               //No brownout reset
//#FUSES BORV47                   //Brownout reset at 4.7V
#FUSES LPOL_HIGH                //Low-Side Transistors Polarity is Active-High (PWM 0,2,4 and 6)
   //PWM module low side output pins have active high output polar
#FUSES HPOL_HIGH                //High-Side Transistors Polarity is Active-High (PWM 1,3,5 and 7)
   //PWM module high side output pins have active high output polarity
#FUSES NOPWMPIN                 //PWM outputs drive active state upon Reset
#FUSES MCLR                     //Master Clear pin enabled
#FUSES NOPROTECT                //Code not protected from reading
#FUSES NOWRT                    //Program memory not write protected
#FUSES NODEBUG                  //No Debug mode for ICD
//#FUSES NOCOE                    //Device will reset into operational mode
#FUSES ICS0                     //ICD communication channel 0
#FUSES RESERVED                 //Used to set the reserved FUSE bits

#use delay(clock=10000000)


#use rs232(UART1,baud=9600,parity=N,bits=8)

int16 trisB;
#locate trisB=0x02C6



#bit    trisb3=trisb.3
#bit    trisb4=trisb.4
#bit    trisb5=trisb.5
struct sensor{
    int  non  :3;    
    int  en_data :3;
}hall_data;

#locate hall_data=0x02C8
#LOCATE TRISE =0X02D8
#BIT    TRIS_E8=TRISE.8 

#LOCATE PORTE = 0X2DA
#BIT    E8 = PORTE.8
int16 PTCON; 
#locate PTCON = 0x1C0 
#bit PTEN     = PTCON.15
#bit PTSIDL   = PTCON.13

#bit PTOPS3 = PTCON.7
#bit PTOPS2 = PTCON.6
#bit PTOPS1 = PTCON.5
#bit PTOPS0 = PTCON.4

#bit PTCKPS1 = PTCON.3 
#bit PTCKPS0 = PTCON.2
#BIT PTMOD1  =PTCON.1
#BIT PTMOD0  =PTCON.0





//-----------------------------------
INT16 PTMR;
#LOCATE  PTMR = 0X1C2

//-----------------------------------
INT16 PTPER;
#LOCATE PTPER = 0X1C4

//-----------------------------------
INT16 SEVTCMP;
#LOCATE SEVTCMP = 0X1C6 

//-----------------------------------
INT16 PWMCON1;
#locate PWMCON1 = 0x1c8
#bit PMOD3 = PWMCON1.10 
#bit PMOD2 = PWMCON1.9
#bit PMOD1 = PWMCON1.8
#bit PEN3H = PWMCON1.6
#bit PEN2H = PWMCON1.5
#bit PEN1H = PWMCON1.4

#bit PEN3L = PWMCON1.2
#bit PEN2L = PWMCON1.1
#bit PEN1L = PWMCON1.0
//-----------------------------------
INT16 PWMCON2;
#locate PWMCON2 = 0x1ca
#bit SEVOPS3 = PWMCON2.11 
#bit SEVOPS2 = PWMCON2.10 
#bit SEVOPS1 = PWMCON2.9
#bit SEVOPS0 = PWMCON2.8 
#bit OSYNC   = PWMCON2.1
#bit UDIS    = PWMCON2.0

//----------------------------------
INT16 DTCON1;
#LOCATE DTCON1 = 0X1CC
#BIT DTAPS1 = DTCON1.7
#BIT DTAPS0 = DTCON1.6

#BIT DTIME5 = DTCON1.5
#BIT DTIME4 = DTCON1.4
#BIT DTIME3 = DTCON1.3
#BIT DTIME2 = DTCON1.2
#BIT DTIME1 = DTCON1.1
#BIT DTIME0 = DTCON1.0
//----------------------------------
int16 ifs2;
#locate ifs2=0x088
#bit   fltaif = ifs2.11
INT16 FLTACON;
#LOCATE FLTACON = 0X1D0
#BIT    FAOV3H  = FLTACON.13
#BIT    FAOV3L  = FLTACON.12
#BIT    FAOV2H  = FLTACON.11
#BIT    FAOV2L  = FLTACON.10
#BIT    FAOV1H  = FLTACON.9
#BIT    FAOV1L  = FLTACON.8

#BIT    FLTAM   = FLTACON.7

#BIT    FAEN3   = FLTACON.2
#BIT    FAEN2   = FLTACON.1
#BIT    FAEN1   = FLTACON.0
//---------------------------------
INT16 OVDCON;
#LOCATE OVDCON  =  0X1D4
#BIT    POVD3H  =  OVDCON.13
#BIT    POVD3L  =  OVDCON.12
#BIT    POVD2H  =  OVDCON.11
#BIT    POVD2L  =  OVDCON.10
#BIT    POVD1H  =  OVDCON.9
#BIT    POVD1L  =  OVDCON.8

#BIT    POUT4H  =  OVDCON.7
#BIT    POUT3H  = OVDCON.5
#BIT    POUT3L  = OVDCON.4
#BIT    POUT2H  = OVDCON.3
#BIT    POUT2L  = OVDCON.2
#BIT    POUT1H  = OVDCON.1
#BIT    POUT1L  = OVDCON.0
//-----------------------------------
//#define sw_break pin_e8

INT16 PDC1;
#LOCATE PDC1 = 0X1D6
INT16 PDC2;
#LOCATE PDC2 = 0X1D8
INT16 PDC3;
#LOCATE PDC3 = 0X1DA


INT8 INDEX;
int1 flg_int,flg_t4,FLG_RDA;
int16 n;




int1 flg_bk;

void getspeed(void);
void chk_unbreak(void);
/*INT16 CONST TABLE[]={20,60,100,140,160,200,240,280,320,360,400,440,480
,520,560,600,640,700,740,780,820,860,900,940,980,980
,940,900,860,820,780,740,700,640,600,560,520,480,440,400,360,320,280,
240,200,160,140,100,60,20};*/
int16 const table[]={
130,// 1
259,// 2 
382,// 3
500,// 4
608,// 5
707,// 6
793,// 7
866,// 8
924,// 9
966,// 10
991,// 11
992,// 12 
992,// 13 
992,// 14
991,// 15
966,// 16
924,// 17
866,// 18
793,// 19
707,// 20
608,// 21
500,// 22
382,// 23
259,// 24
130,// 25
//-----------
0,// 1
0,// 2
0,// 3
0,// 4
0,// 5
0,// 6
0,// 7
0,// 8
0,// 9
0,// 10
0,// 11
0,// 12
0,// 13 
0,// 14 
0,// 15 
0,// 16 
0,// 17
0,// 18
0,// 19
0,// 20
0,// 21
0,// 22 
0,// 23
0,// 24
0,// 25
}
;
//===============================
int16 const table2[]={
0,// 1
0,// 2
0,// 3
0,// 4
0,// 5
0,// 6
0,// 7
0,// 8
0,// 9
0,// 10
0,// 11
0,// 12
0,// 13 
0,// 14 
0,// 15 
0,// 16 
0,// 17
0,// 18
0,// 19
0,// 20
0,// 21
0,// 22 
0,// 23
0,// 24
0,// 25
//-----------
130,// 1
259,// 2 
382,// 3
500,// 4
608,// 5
707,// 6
793,// 7
866,// 8
924,// 9
966,// 10
991,// 11
992,// 12 
992,// 13 
992,// 14
991,// 15
966,// 16
924,// 17
866,// 18
793,// 19
707,// 20
608,// 21
500,// 22
382,// 23
259,// 24
130,// 25
}
;

//==============================

#int_PWM1
void  PWM1_isr(void) 
{
  
  INDEX++;
  //pdc1=1000;
  pdc1=table[index];
  pdc2=table2[index];
  pten=1;IF(INDEX>=49)INDEX=0;
 
}



void main()
{
    setup_wdt (WDT_OFF);
  flg_t4=0;
  //trisb=0x000f;
  TRIS_E8=1;
  trisb3=trisb4=trisb5=0;
 // ptper=999;
  ptper=500;
  //SEVTCMP=ptper;
  ptmr=0x0000;
  //===============PMOD3(PWM3 MODE) PMDO2(PWM2MODE) AND PMOD1(PWM1MODE) FOR SELECT COMPLEMENTARY OR Independent mode
  PMOD3=1;//PWM3
  PMOD2=0;//PWM2 
  PMOD1=0;// PWM1
  
 // pten=1;      // ENABLE INPUT CLOCK FOR PWM  
  PTSIDL=0; 
  //===========PTMOD0 AND PTMOD1 FOR SELECT MODE PWM
  ptmod0=1;    // 00 Free Running mode
  ptmod1=0;    // 01  Single-shot mode
               // 10  Continuous Up/Down Counting mode.
               // 11   Continuous Up/Down mode with interrupts for double PWM
  //============PTCKPS0 AND PTCKPS1 BIT FOR PTMR PRESCALE
  ptckps0=1;   // 00 (1:1 prescale)
  ptckps1=0;   // 01 (1:4 prescale) 
               // 10 (1:16 prescale)
               // 11 (1:64 prescale)  
  //============PENxh PENxl for enable pwmoutput===============
  pen3h=1;    //   
  pen2h=1;    //
  pen1h=1;    //
  pen3l=1;    //
  pen2l=1;    //
  pen1l=1;    //
 
//=======================================
  pdc1=0; //
  pdc2=0; //
  pdc3=0; //

//=======================
   OSYNC=1;UDIS=0;
 
   POVD1H =1;
   POVD1L =1;
   POVD2H =1;
   POVD2L =1;

   POVD3H =1;
   povd3l =1;
   Pout3L =1;
//========================
 //PWMCON2=0X0000;

   //FAEN1=1;
   //FAOV1H=1;

   //FLTAM=0;
  INDEX=0;
  SETUP_ADC_PORTS(sAN0|VSS_VDD );
  SETUP_ADC(ADC_CLOCK_INTERNAL);
  enable_interrupts(INTR_GLOBAL);
  enable_interrupts(INT_PWM1);
  //set_timer1(500);
  //=====================SET TIMER23 32 BIT=======

  DTCON1=0x000f;//dead time =  51 usec

  flg_int=0;FLG_RDA=0;flg_bk=0;
  n=0;
  INDEX=0;PTEN =1;
  pdc1=TABLE[INDEX]; pten=1;  
//PDC1=499;
WHILE(TRUE)
{
//OUTPUT_TOGGLE(PIN_D1);
hall_data.en_data =2;
DELAY_MS(200);
hall_data.en_data =4;
DELAY_MS(200);
}

}


  
