/**************************************************************************//**
 * @main_gg11_tg11.c
 * @brief This project demonstrates multiple frequency generation using
 * TIMER modules. TIMER2 clock is fed by the TIMER1 overflow.
 *
 * Clock Input: PC14, TIM1_CC1 #0
 * Output 1   : PC13, TIM1_CC0 #0
 * Output 2   : PA8 , TIM2_CC0 #0
 *
 * TODO: Given 6 MHz input, why don't the output edges align? They're off by ~150 ns,
 *       i.e. output 2 edge occurs midway through an output 1 pulse.
 * @version 0.0.1 *modified
 ******************************************************************************
 * @section License
 * <b>Copyright 2018 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_chip.h"
#include "em_timer.h"

#include <assert.h>

#define NUM_CLKINS_PER_CLKOUT1   (4)
#define NUM_CLKOUT1S_PER_CLKOUT2 (6)

#define TIMER_PRESCALE timerPrescale1

/**************************************************************************//**
 * @brief
 *    GPIO initialization
 *****************************************************************************/
void initGpio(void)
{
  // Enable GPIO clock
  CMU_ClockEnable(cmuClock_GPIO, true);

  GPIO_PinModeSet(gpioPortC, 13, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortC, 14, gpioModeInput   , 0); // DOUT == 0 means filter disabled
  GPIO_PinModeSet(gpioPortA,  8, gpioModePushPull, 0);
}

/**************************************************************************//**
 * @brief
 *    TIMER initialization
 *****************************************************************************/
void initTimer(void)
{
  // Enable clock for TIMER modules
  CMU_ClockEnable(cmuClock_TIMER1, true);
  CMU_ClockEnable(cmuClock_TIMER2, true);

  // Configure TIMERs Compare/Capture for output compare
  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
  timerCCInit.edge = timerEdgeFalling;
  timerCCInit.mode = timerCCModeCompare;
  timerCCInit.cmoa = timerOutputActionToggle;
  TIMER_InitCC(TIMER1, 0, &timerCCInit);
  TIMER_InitCC(TIMER2, 0, &timerCCInit);

  // Set route locations and enable
  // TIM1_CC0 #0 is PC13
  // TIM1_CC1 #0 is PC14
  TIMER1->ROUTELOC0 |=  (TIMER_ROUTELOC0_CC0LOC_LOC0 | TIMER_ROUTELOC0_CC1LOC_LOC0);
  TIMER1->ROUTEPEN  |=  (TIMER_ROUTEPEN_CC0PEN       | TIMER_ROUTEPEN_CC1PEN      );

  // TIM2_CC0 #0 is PA8
  TIMER2->ROUTELOC0 |=  (TIMER_ROUTELOC0_CC0LOC_LOC0);
  TIMER2->ROUTEPEN  |=  (TIMER_ROUTEPEN_CC0PEN      );

  // Set Top value
  // Note each overflow event constitutes 1/2 the signal period
  TIMER_TopSet(TIMER1, ((NUM_CLKINS_PER_CLKOUT1   / (2 * (1 << TIMER_PRESCALE))) - 1));
  TIMER_TopSet(TIMER2, ((NUM_CLKOUT1S_PER_CLKOUT2 /      (1 << TIMER_PRESCALE) ) - 1));

  // Initialize and start timers with defined prescales
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  timerInit.enable   = true;
  timerInit.prescale = TIMER_PRESCALE;
  timerInit.clkSel   = timerClkSelCascade;
  timerInit.sync     = false;
  TIMER_Init(TIMER2, &timerInit);
  timerInit.clkSel   = timerClkSelCC1;
  TIMER_Init(TIMER1, &timerInit);
}

/**************************************************************************//**
 * @brief
 *    Main function
 *****************************************************************************/
int main(void)
{
  // Chip errata
  CHIP_Init();

  assert(NUM_CLKINS_PER_CLKOUT1   >= (2 * (1 << TIMER_PRESCALE)));
  assert(NUM_CLKOUT1S_PER_CLKOUT2 >= (2 * (1 << TIMER_PRESCALE)));

  CMU_HFRCOBandSet(cmuHFRCOFreq_72M0Hz);

  // Init DCDC regulator with kit specific parameters
  EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_DEFAULT;
  EMU_DCDCInit(&dcdcInit);

  // Initialization
  initGpio();
  initTimer();

  while (1) {
    EMU_EnterEM1(); // Enter EM1 (won't exit)
  }
}


