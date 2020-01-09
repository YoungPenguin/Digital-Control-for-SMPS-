#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "f2802x_common/include/adc.h"
#include "f2802x_common/include/clk.h"
#include "f2802x_common/include/flash.h"
#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/pie.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/timer.h"
#include "f2802x_common/include/wdog.h"
#include "f2802x_common/include/pwr.h"
#include "f2802x_common/include/pwm.h"


void pwm_Init_();

unsigned int TBPRD = 4096;// step-size of the PWM (PWM resolution)
unsigned int CMPA=0;

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

uint16_t Digital_Result =2000;

void disable();
void enable();
void ADC_INIT_Fn();
void ADC_SETUP_Fn();
void set_duty(int a);

int adcresult=2048;

int R1 = 1000;
int R2 = 1000;
int Ref_v = 2000;

int vout;
int vin;

// control parameters
int kp = 10;
int ki = 10;

int error;
int pwm;
int integral= 500;

interrupt void adc_isr(void)
{
    Digital_Result = ADC_readResult(myAdc, ADC_ResultNumber_0);

    adcresult = Digital_Result;

// PI Controller section
  /*  vout = (adcresult * 3.3) / 4096; // see text
    vin = vout / (R2/(R1+R2));
    error = Ref_v-vin; */
    error = Ref_v-adcresult;

    integral = integral + error;
    pwm = (kp*error)+(ki*integral);
// PI controller end //


    set_duty(TBPRD-adcresult); // TBPRD is 12 bit, so this just do the inversion of the analog input value

    ADC_clearIntFlag(myAdc, ADC_IntNumber_1);   // Clear ADCINT1 flag
    PIE_clearInt(myPie, PIE_GroupNumber_10);
    return;
}


void main(void)
{
   myAdc = ADC_init((void *)ADC_BASE_ADDR, sizeof(ADC_Obj));
   myClk = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
   myCpu = CPU_init((void *)NULL, sizeof(CPU_Obj));
   myFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
   myGpio = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
   myPie = PIE_init((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
   myPll = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
   myTimer = TIMER_init((void *)TIMER0_BASE_ADDR, sizeof(TIMER_Obj));
   myWDog = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));
   myPwm4 = PWM_init((void *)PWM_ePWM4_BASE_ADDR, sizeof(PWM_Obj));
   myPwr = PWR_init((void *)PWR_BASE_ADDR, sizeof(PWR_Obj));

   disable();
    // basic initialization
    WDOG_disable(myWDog);
    CLK_enableAdcClock(myClk);
    CLK_setOscSrc(myClk, CLK_OscSrc_Internal);
    PLL_setup(myPll, PLL_Multiplier_10, PLL_DivideSelect_ClkIn_by_2);


    enable();
    ADC_INIT_Fn();
    ADC_SETUP_Fn();

    //GPIO 6 & 7
    GPIO_setMode(myGpio, GPIO_Number_6, GPIO_6_Mode_EPWM4A);
    GPIO_setMode(myGpio, GPIO_Number_7, GPIO_7_Mode_EPWM4B);


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
  PIE_disable(myPie);
  PIE_disableAllInts(myPie);
  CPU_disableGlobalInts(myCpu);
  CPU_clearIntFlags(myCpu);
}

void enable()
{
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
    PWM_setPeriod(myPwm4, TBPRD);   // Set timer period 801 TBCLKs
    PWM_setPhase(myPwm4, 0x0000);
    PWM_setCount(myPwm4, 0x0000);

    // Setup counter mode
    PWM_setCounterMode(myPwm4, PWM_CounterMode_UpDown);
    PWM_disableCounterLoad(myPwm4);
    PWM_setHighSpeedClkDiv(myPwm4, PWM_HspClkDiv_by_10);
    PWM_setClkDiv(myPwm4, PWM_ClkDiv_by_1);

    PWM_setShadowMode_CmpA(myPwm4, PWM_ShadowMode_Shadow);
    PWM_setLoadMode_CmpA(myPwm4, PWM_LoadMode_Zero);
    PWM_setActionQual_CntUp_CmpA_PwmA(myPwm4, PWM_ActionQual_Clear);
    PWM_setActionQual_CntDown_CmpA_PwmA(myPwm4, PWM_ActionQual_Set);  // Clear PWM1A on event A, down count

    //SETUP for GPIO 7 PWM
    PWM_setShadowMode_CmpB(myPwm4, PWM_ShadowMode_Shadow);
    PWM_setLoadMode_CmpB(myPwm4, PWM_LoadMode_Zero);
    PWM_setActionQual_CntUp_CmpB_PwmB(myPwm4, PWM_ActionQual_Clear);
    PWM_setActionQual_CntDown_CmpB_PwmB(myPwm4, PWM_ActionQual_Set);
}
void set_duty( int a)
{
 CMPA = a;
 PWM_setCmpA(myPwm4, CMPA);
 PWM_setCmpB(myPwm4, CMPA);// Set compare A value
}

//===========================================================================
// No more. stop pls
