/*** main.c ***/

// TI-RTOS includes
#include <xdc/std.h>  						//mandatory - have to include first, for BIOS types
#include <ti/sysbios/BIOS.h> 				//mandatory - if you call APIs like BIOS_start()
#include <xdc/runtime/Log.h>				//for Log_info() calls when UIA is added
#include <xdc/cfg/global.h> 				//header file for statically defined objects/handles

// motorware platforms
#include <math.h>
#include "main.h"

#pragma CODE_SECTION(isr_ADC1,"ramfuncs");

//-----------------------------------------
// Prototypes
//-----------------------------------------
void clock_100msec(void);
void clock_1msec(void);

#define LED_BLINK_FREQ_Hz   5

//-----------------------------------------
// Globals
//-----------------------------------------
volatile int16_t i16ToggleCount = 0;

extern uint16_t RamfuncsLoadStart; // External symbols created by the linker cmd file
extern uint16_t RamfuncsLoadEnd;
extern uint16_t RamfuncsRunStart;
extern uint16_t RamfuncsLoadSize;

//TODO: Move important params to separate global.c file, and try to remove others if possible
uint_least16_t gCounter_updateGlobals = 0;
bool Flag_Latch_softwareUpdate = true;
CTRL_Handle ctrlHandle;
HAL_Handle halHandle;
USER_Params gUserParams;
HAL_PwmData_t gPwmData = { _IQ(0.0), _IQ(0.0), _IQ(0.0) };
HAL_AdcData_t gAdcData;
_iq gMaxCurrentSlope = _IQ(0.0);
CTRL_Obj *controller_obj;
uint16_t gLEDcnt = 0;
volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT;
DRV_SPI_8301_Vars_t gDrvSpi8301Vars;
_iq gFlux_pu_to_Wb_sf;
_iq gFlux_pu_to_VpHz_sf;
_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;
_iq gTorque_Flux_Iq_pu_to_Nm_sf;

/****************************************************************************
 * main
 ***************************************************************************/
void main(void)
{
	uint_least8_t estNumber = 0;
	uint_least8_t ctrlNumber = 0;

	memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (unsigned long) &RamfuncsLoadSize);	// Copy InitFlash fxn to RAM and run it

	// initialize the hardware abstraction layer
	halHandle = HAL_init(&hal, sizeof(hal));

	// check for errors in user parameters
	USER_checkForErrors(&gUserParams);

	// store user parameter error in global variable
	gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);

	// do not allow code execution if there is a user parameter error
	if (gMotorVars.UserErrorCode != USER_ErrorCode_NoError)
	{
		for (;;)
		{
			gMotorVars.Flag_enableSys = false;
		}
	}

	// initialize the user parameters
	USER_setParams(&gUserParams);

	// set the hardware abstraction layer parameters
	HAL_setParams(halHandle, &gUserParams);

	ctrlHandle = CTRL_initCtrl(ctrlNumber, estNumber); //v1p6 format (06xF and 06xM devices)
	controller_obj = (CTRL_Obj *) ctrlHandle;

	{
		CTRL_Version version;
		CTRL_getVersion(ctrlHandle, &version);	// get the version number
		gMotorVars.CtrlVersion = version;
	}

	// set the default controller parameters
	CTRL_setParams(ctrlHandle, &gUserParams);

	// setup faults
	HAL_setupFaults(halHandle);

	// enable the ADC interrupts
	HAL_enableAdcInts(halHandle);

	// enable global interrupts
	HAL_enableGlobalInts(halHandle);

	// disable the PWM
	HAL_disablePwm(halHandle);

	// turn on the DRV8301 if present
	HAL_enableDrv(halHandle);

	// initialize the DRV8301 interface
	HAL_setupDrvSpi(halHandle, &gDrvSpi8301Vars);

	// enable DC bus compensation
	CTRL_setFlag_enableDcBusComp(ctrlHandle, true);

	// compute scaling factors for flux and torque calculations
	gFlux_pu_to_Wb_sf = USER_computeFlux_pu_to_Wb_sf();
	gFlux_pu_to_VpHz_sf = USER_computeFlux_pu_to_VpHz_sf();
	gTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
	gTorque_Flux_Iq_pu_to_Nm_sf = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();

	BIOS_start();	// Start BIOS Scheduler
}

/****************************************************************************
 * Test task
 ***************************************************************************/
void task_led_toggle(void)
{
	while (1)
	{
		Semaphore_pend(sem_led_blink, BIOS_WAIT_FOREVER);// wait for LEDSem to be posted by ISR
		HAL_toggleGpio(halHandle, GPIO_Number_34);
		i16ToggleCount += 1;						// keep track of #toggles
		Log_info1("LED TOGGLED [%u] TIMES",i16ToggleCount);	// store #toggles to Log display
	}
}

/****************************************************************************
 * ADC ISR
 ***************************************************************************/
void isr_ADC1(void)
{
	// acknowledge the ADC interrupt
	ADC_clearIntFlag(halHandle->adcHandle, ADC_IntNumber_1);

	// convert the ADC data
	HAL_readAdcData(halHandle, &gAdcData);

	// run the controller
	CTRL_run(ctrlHandle, halHandle, &gAdcData, &gPwmData);

	// write the PWM compare values
	HAL_writePwmData(halHandle, &gPwmData);

	// setup the controller
	CTRL_setup(ctrlHandle);

	return;
}

/****************************************************************************
 * 100 msec software timer
 ***************************************************************************/
void clock_100msec(void)
{
	Semaphore_post(sem_led_blink);
}

/****************************************************************************
 * motorware main stuff
 ***************************************************************************/
void task_motorware()
{
	for (;;)
	{
		// Waiting for enable system flag to be set
		while (!(gMotorVars.Flag_enableSys))
		{
			Task_yield();
		}

		// loop while the enable system flag is true
		while (gMotorVars.Flag_enableSys)
		{
			Task_yield();	//TODO: change this so that an outside timer activates the motorware task when needed

			CTRL_Obj *obj = (CTRL_Obj *) ctrlHandle;

			// increment counters
			gCounter_updateGlobals++;

			// enable/disable the use of motor parameters being loaded from user.h
			CTRL_setFlag_enableUserMotorParams(ctrlHandle, gMotorVars.Flag_enableUserParams);

			if (CTRL_isError(ctrlHandle))
			{
				// set the enable controller flag to false
				CTRL_setFlag_enableCtrl(ctrlHandle, false);

				// set the enable system flag to false
				gMotorVars.Flag_enableSys = false;

				// disable the PWM
				HAL_disablePwm(halHandle);
			}
			else
			{
				// update the controller state
				bool flag_ctrlStateChanged = CTRL_updateState(ctrlHandle);

				// enable or disable the control
				CTRL_setFlag_enableCtrl(ctrlHandle,
						gMotorVars.Flag_Run_Identify);

				if (flag_ctrlStateChanged)
				{
					CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);

					if (ctrlState == CTRL_State_OffLine)
					{
						// enable the PWM
						HAL_enablePwm(halHandle);
					}
					else if (ctrlState == CTRL_State_OnLine)
					{
						// update the ADC bias values
						HAL_updateAdcBias(halHandle);

						// Return the bias value for currents
						gMotorVars.I_bias.value[0] = HAL_getBias(halHandle, HAL_SensorType_Current, 0);
						gMotorVars.I_bias.value[1] = HAL_getBias(halHandle, HAL_SensorType_Current, 1);
						gMotorVars.I_bias.value[2] = HAL_getBias(halHandle, HAL_SensorType_Current, 2);

						// Return the bias value for voltages
						gMotorVars.V_bias.value[0] = HAL_getBias(halHandle, HAL_SensorType_Voltage, 0);
						gMotorVars.V_bias.value[1] = HAL_getBias(halHandle, HAL_SensorType_Voltage, 1);
						gMotorVars.V_bias.value[2] = HAL_getBias(halHandle, HAL_SensorType_Voltage, 2);

						// enable the PWM
						HAL_enablePwm(halHandle);
					}
					else if (ctrlState == CTRL_State_Idle)
					{
						// disable the PWM
						HAL_disablePwm(halHandle);
						gMotorVars.Flag_Run_Identify = false;
					}

					if ((CTRL_getFlag_enableUserMotorParams(ctrlHandle) == true)
							&& (ctrlState > CTRL_State_Idle)
							&& (gMotorVars.CtrlVersion.minor == 6))
					{
						// call this function to fix 1p6
						USER_softwareUpdate1p6(ctrlHandle);
					}
				}
			}

			if (EST_isMotorIdentified(obj->estHandle))
			{
				// set the current ramp
				EST_setMaxCurrentSlope_pu(obj->estHandle, gMaxCurrentSlope);
				gMotorVars.Flag_MotorIdentified = true;

				// set the speed reference
				CTRL_setSpd_ref_krpm(ctrlHandle, gMotorVars.SpeedRef_krpm);

				// set the speed acceleration
				CTRL_setMaxAccel_pu(ctrlHandle, _IQmpy(MAX_ACCEL_KRPMPS_SF, gMotorVars.MaxAccel_krpmps));

				if (Flag_Latch_softwareUpdate)
				{
					Flag_Latch_softwareUpdate = false;

					USER_calcPIgains(ctrlHandle);
				}

			}
			else
			{
				Flag_Latch_softwareUpdate = true;

				// the estimator sets the maximum current slope during identification
				gMaxCurrentSlope = EST_getMaxCurrentSlope_pu(obj->estHandle);
			}

			// when appropriate, update the global variables
			if (gCounter_updateGlobals
					>= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE)
			{
				// reset the counter
				gCounter_updateGlobals = 0;

				updateGlobalVariables_motor(ctrlHandle);
			}

			// enable/disable the forced angle
			EST_setFlag_enableForceAngle(obj->estHandle, gMotorVars.Flag_enableForceAngle);

			// enable or disable power warp
			CTRL_setFlag_enablePowerWarp(ctrlHandle, gMotorVars.Flag_enablePowerWarp);

#ifdef DRV8301_SPI
			HAL_writeDrvData(halHandle, &gDrvSpi8301Vars);

			HAL_readDrvData(halHandle, &gDrvSpi8301Vars);
#endif

		} // end of while(gFlag_enableSys) loop

		// disable the PWM
		HAL_disablePwm(halHandle);

		// set the default controller parameters (Reset the control to re-identify the motor)
		CTRL_setParams(ctrlHandle, &gUserParams);
		gMotorVars.Flag_Run_Identify = false;

	} // end of for(;;) loop

} // end of task_motorware() function



/****************************************************************************
 * update global variables
 ***************************************************************************/

 
// define a macro to fix the return value.  A union would likely be cleaner but this is the TI recommended method.
#ifdef __TMS320C28XX_FPU32__
#warning: FP: assigning temp to instaspin->real world.
#define fpu_assign(x,y) 	tmp =y;						\
							x = *((float_t *) &tmp)
#else
#warning: Fixed pt: direct assigning instaspin->real world.
#define		fpu_assign(x,y)   x =y

#endif

 
 
void updateGlobalVariables_motor(CTRL_Handle handle)
{
	CTRL_Obj *obj = (CTRL_Obj *) handle;


#ifdef __TMS320C28XX_FPU32__
  int32_t tmp;
#endif



	// get the speed estimate
	gMotorVars.Speed_krpm = EST_getSpeed_krpm(obj->estHandle);

	// get the real time speed reference coming out of the speed trajectory generator
	gMotorVars.SpeedTraj_krpm = _IQmpy(CTRL_getSpd_int_ref_pu(handle), EST_get_pu_to_krpm_sf(obj->estHandle));

	// get the torque estimate
	gMotorVars.Torque_Nm = USER_computeTorque_Nm(handle, gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf);

	// get the magnetizing current
   fpu_assign(gMotorVars.MagnCurr_A ,EST_getIdRated( obj->estHandle ));

	// get the rotor resistance
   fpu_assign(gMotorVars.Rr_Ohm, EST_getRr_Ohm( obj->estHandle ));

	// get the stator resistance
   fpu_assign(gMotorVars.Rs_Ohm	  , EST_getRs_Ohm( obj->estHandle ));

	// get the stator inductance in the direct coordinate direction
   fpu_assign(gMotorVars.Lsd_H,   EST_getLs_d_H( obj->estHandle ));

	// get the stator inductance in the quadrature coordinate direction
   fpu_assign(gMotorVars.Lsq_H	  , EST_getLs_q_H( obj->estHandle ));

	// get the flux in V/Hz in floating point
   fpu_assign(gMotorVars.Flux_VpHz , EST_getFlux_VpHz( obj->estHandle ));



	// get the flux in Wb in fixed point
	gMotorVars.Flux_Wb = USER_computeFlux(handle, gFlux_pu_to_Wb_sf);

	// get the controller state
	gMotorVars.CtrlState = CTRL_getState(handle);

	// get the estimator state
	gMotorVars.EstState = EST_getState(obj->estHandle);

	// Get the DC buss voltage
	gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus, _IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));

	return;
} // end of updateGlobalVariables_motor() function

