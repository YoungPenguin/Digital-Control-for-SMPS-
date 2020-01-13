//----------------------------------------------------------------------------------
//  FILE:           ADC_EPWM_PI.c
//
//  Description:    PI regulation configuration function for flyback stage
//
//  Target:         TMS320F2802x,
//
// The main program file is called:
//
//       ADC_EPWM_PI.c
//
// Functionality and usability:
//-------------------------------
//
//  The negative feedback is on the ADCINA0 (j15)
//  The PWM out is locaed on GPIO6
//
//  The PI control algorithm is implemented as a discrete PI with the formula
//  u(k)=(kp*(error(k)-error(k-1))+ki*Ts*error(k))+u(k-1);
//
//--------------------------------------------------------------------------------

#include "DSP28x_Project.h"               // Device Headerfile and Examples Include File
#include "f2802x_common/include/adc.h"    // Analog to digital converter
#include "f2802x_common/include/clk.h"    // clk
#include "f2802x_common/include/flash.h"  // flash
#include "f2802x_common/include/gpio.h"   // GPIO pins (used for GPIO6)
#include "f2802x_common/include/pie.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/timer.h"
#include "f2802x_common/include/wdog.h"
#include "f2802x_common/include/pwr.h"
#include "f2802x_common/include/pwm.h"

void pwm_Init_();

unsigned int TBPRD = 330; // TBPRD register - the speed of the system - here 150 kHz
unsigned int CMPA  = 0;

// setup all the handles
ADC_Handle   myAdc;
CLK_Handle   myClk;
FLASH_Handle myFlash;
GPIO_Handle  myGpio;
PIE_Handle   myPie;
TIMER_Handle myTimer;
CPU_Handle   myCpu;
PLL_Handle   myPll;
WDOG_Handle  myWDog;
PWM_Handle   myPwm4;
PWR_Handle   myPwr;

uint16_t Digital_Result = 0;

void disable();
void enable();
void ADC_INIT_Fn();
void ADC_SETUP_Fn();
void set_duty(int a);

int adcresult = 0;
int Ref_v     = 165;

// control parameters
double kp   = 1.1;
double ki   = 1000; // ki=kp/ti

int out_max = 330; // 150 kHz - calculated by the TBPRD
int out_min = 0;

double error       = 0;
double PI_output   = 0;
double Ts          = 0.00000666666; // 1/150000  - 150 kHz
double prev_out    = 0;
double  prev_error = 0;

interrupt void adc_isr(void)
{
    Digital_Result = ADC_readResult(myAdc, ADC_ResultNumber_0);

    adcresult = Digital_Result/12.40909091; // ADC 4095 mapped to TBPRD 330

    // PI control algo begin
    error     = Ref_v-adcresult;
    PI_output = (kp*(error-prev_error)+ki*Ts*error)+prev_out;

    // anti-windup
    if(PI_output > out_max){
        PI_output = out_max;
    }
    else if (PI_output < out_min) {
        PI_output = out_min;
    }

    // defining the u(k-1) and e(k-1)
    prev_out   = PI_output;
    prev_error = error;

    set_duty(PI_output);     // set PWM to the PI regulated value

    ADC_clearIntFlag(myAdc, ADC_IntNumber_1);   // Clear ADCINT1 flag
    PIE_clearInt(myPie, PIE_GroupNumber_10);    // Acknowledge interrupt
    return;
}


void main(void)
{
   myAdc   = ADC_init((void *)ADC_BASE_ADDR, sizeof(ADC_Obj));
   myClk   = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
   myCpu   = CPU_init((void *)NULL, sizeof(CPU_Obj));
   myFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
   myGpio  = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
   myPie   = PIE_init((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
   myPll   = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
   myTimer = TIMER_init((void *)TIMER0_BASE_ADDR, sizeof(TIMER_Obj));
   myWDog  = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));
   myPwm4  = PWM_init((void *)PWM_ePWM4_BASE_ADDR, sizeof(PWM_Obj));
   myPwr   = PWR_init((void *)PWR_BASE_ADDR, sizeof(PWR_Obj));

   disable();
    // basic initialization
    WDOG_disable(myWDog);
    CLK_enableAdcClock(myClk);
    CLK_setOscSrc(myClk, CLK_OscSrc_Internal);
    PLL_setup(myPll, PLL_Multiplier_10, PLL_DivideSelect_ClkIn_by_1);

    enable();
    ADC_INIT_Fn();
    ADC_SETUP_Fn();

    //GPIO 6
    GPIO_setMode(myGpio, GPIO_Number_6, GPIO_6_Mode_EPWM4A);

      CLK_disableTbClockSync(myClk);
      pwm_Init_();

      CLK_enableTbClockSync(myClk);

    while(1)
    {
     ADC_forceConversion(myAdc, ADC_SocNumber_0);// Wait for ADC interrupt
    }
}
void disable()
{
  // Disable function
  PIE_disable(myPie);
  PIE_disableAllInts(myPie);
  CPU_disableGlobalInts(myCpu);
  CPU_clearIntFlags(myCpu);
}

void enable()
{
   // Enable function
  PIE_enable(myPie);
  CPU_enableInt(myCpu, CPU_IntNumber_10);
  CPU_enableGlobalInts(myCpu);
  CPU_enableDebugInt(myCpu);

  PIE_enableInt(myPie, PIE_GroupNumber_3, PIE_InterruptSource_XINT_1);
  CPU_enableInt(myCpu, CPU_IntNumber_1);
  GPIO_setExtInt(myGpio, GPIO_Number_12, CPU_ExtIntNumber_1);
  PIE_setExtIntPolarity(myPie, CPU_ExtIntNumber_1, PIE_ExtIntPolarity_RisingEdge);
  PIE_enableExtInt(myPie, CPU_ExtIntNumber_1);
}

void ADC_INIT_Fn()
{
  ADC_enableBandGap(myAdc);
  ADC_enableRefBuffers(myAdc);
  ADC_powerUp(myAdc);
  ADC_enable(myAdc);
  ADC_setVoltRefSrc(myAdc, ADC_VoltageRefSrc_Int);
}

void ADC_SETUP_Fn()
{
 PIE_registerPieIntHandler(myPie, PIE_GroupNumber_10, PIE_SubGroupNumber_1, (intVec_t)&adc_isr);
  PIE_enableAdcInt(myPie, ADC_IntNumber_1);  // Enable ADCINT1 in PIE

  ADC_setIntPulseGenMode(myAdc, ADC_IntPulseGenMode_Prior);
  ADC_enableInt(myAdc, ADC_IntNumber_1);                                  //Enabled ADCINT1
  ADC_setIntMode(myAdc, ADC_IntNumber_1, ADC_IntMode_ClearFlag);
  ADC_setIntSrc(myAdc, ADC_IntNumber_1, ADC_IntSrc_EOC0);
  ADC_setSocChanNumber (myAdc, ADC_SocNumber_0, ADC_SocChanNumber_A4);
  ADC_setSocTrigSrc(myAdc, ADC_SocNumber_0, ADC_SocTrigSrc_Sw);
  ADC_setSocSampleWindow(myAdc, ADC_SocNumber_0, ADC_SocSampleWindow_7_cycles);
}

void pwm_Init_()
{
    CLK_enablePwmClock(myClk, PWM_Number_4);
    // Setup TBCLK
    PWM_setPeriod(myPwm4, TBPRD);   // Set timer period
    PWM_setPhase(myPwm4, 0x0000);   // set the phase
    PWM_setCount(myPwm4, 0x0000);   // set the count

    // Setup counter mode
    PWM_setCounterMode(myPwm4, PWM_CounterMode_UpDown);
    PWM_disableCounterLoad(myPwm4);

    // clk division
    PWM_setHighSpeedClkDiv(myPwm4, PWM_HspClkDiv_by_1);
    PWM_setClkDiv(myPwm4, PWM_ClkDiv_by_1);

    PWM_setShadowMode_CmpA(myPwm4, PWM_ShadowMode_Shadow);
    PWM_setLoadMode_CmpA(myPwm4, PWM_LoadMode_Zero);
    PWM_setActionQual_CntUp_CmpA_PwmA(myPwm4, PWM_ActionQual_Clear);
    PWM_setActionQual_CntDown_CmpA_PwmA(myPwm4, PWM_ActionQual_Set);  // Clear PWM1A on event A, down count

}
void set_duty( int a)
{
 CMPA = a;
 PWM_setCmpA(myPwm4, CMPA);
 PWM_setCmpB(myPwm4, CMPA);// Set compare A value
}
//===========================================================================
// No more. stop pls
