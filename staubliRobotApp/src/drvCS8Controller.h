/*
 * drvCS8Controller.h
 *
 *  Created on: Sep 5, 2017
 *      Author: hwchen
 */

#ifndef STAUBLIROBOT_STAUBLIROBOTAPP_SRC_DRVCS8CONTROLLER_H_
#define STAUBLIROBOT_STAUBLIROBOTAPP_SRC_DRVCS8CONTROLLER_H_

#include <algorithm>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <asynPortDriver.h>
#include <epicsString.h>
#include "staubli_robot.h"

/*parameters in CS8 controller level*/
#define EsStopString           "ES_STOP"
#define ArmPowerString         "ARM_POWER"
#define AutoModeString         "AUTO_MODE"
#define ManualModeString       "MANUAL_MODE"
#define AtHomeString           "AT_HOME"
#define IsSettledString        "IS_SETTLED"
#define JobDoneString          "JOB_DONE"
#define RobotStartString       "ROBOT_START"
#define RobotStopString        "ROBOT_STOP"
#define ProdModeString         "PROD_MODE"
#define StepModeString         "STEP_MODE"
#define RobotPauseString       "ROBOT_PAUSE"
#define GetFromTrayString      "GET_FROM_TRAY"
#define GetFromInspecString    "GET_FROM_INSPEC"
#define RobotErrorString       "ROBOT_ERROR"
#define RobotResetErrorString  "RESET_ERROR"
#define RecipeNowString       "RECIPE_NOW"
#define RecipeInspecString    "RECIPE_INSPEC"

/*parameters in IOC level*/
#define SampleInString         "SAMPLE_IN"
#define SampleOutString        "SAMPLE_OUT"
#define SampleSpinString       "SAMPLE_SPIN"
#define SampleUnsafeSpinString "SAMPLE_UNSAFE_SPIN"
#define RecipeSelectString    "RECIPE_SELECT"
#define RobotBusyString        "ROBOT_BUSY"

/*asyn definitions*/
#define MAX_SIGNALS         16
#define DEFAULT_POLL_TIME   0.1
#define CONSECUTIVE_CMDS_DELAY_TIME   0.1

/*handle max physical link number is 10*/
#define MAX_ACCESS_IOS      10  /* max physical links while issue read_ios/write_ios*/
#define POLLER_IOS_SET_NUMBER 3

/*CS8 mbDO index map*/
#define ES_STOP_PHYSICAL_LINK           "ModbusSrv-0\\Modbus-Bit\\mbDO[0]"
#define ARM_POWER_PHYSICAL_LINK         "ModbusSrv-0\\Modbus-Bit\\mbDO[1]"
#define AUTO_MODE_PHYSICAL_LINK         "ModbusSrv-0\\Modbus-Bit\\mbDO[2]"
#define MANUAL_MODE_PHYSICAL_LINK       "ModbusSrv-0\\Modbus-Bit\\mbDO[3]"
#define AT_HOME_PHYSICAL_LINK           "ModbusSrv-0\\Modbus-Bit\\mbDO[4]"
#define IS_SETTLED_PHYSICAL_LINK        "ModbusSrv-0\\Modbus-Bit\\mbDO[5]"
#define JOB_DONE_PHYSICAL_LINK          "ModbusSrv-0\\Modbus-Bit\\mbDO[7]"
#define ROBOT_START_PHYSICAL_LINK       "ModbusSrv-0\\Modbus-Bit\\mbDO[20]"
#define ROBOT_STOP_PHYSICAL_LINK        "ModbusSrv-0\\Modbus-Bit\\mbDO[21]"
#define PROD_MODE_PHYSICAL_LINK         "ModbusSrv-0\\Modbus-Bit\\mbDO[22]"
#define STEP_MODE_PHYSICAL_LINK         "ModbusSrv-0\\Modbus-Bit\\mbDO[23]"
#define ROBOT_PAUSE_PHYSICAL_LINK       "ModbusSrv-0\\Modbus-Bit\\mbDO[24]"
#define GET_FROM_TRAY_PHYSICAL_LINK     "ModbusSrv-0\\Modbus-Bit\\mbDO[25]"
#define GET_FROM_INSPEC_PHYSICAL_LINK   "ModbusSrv-0\\Modbus-Bit\\mbDO[26]"
#define ROBOT_EROR_PHYSICAL_LINK        "ModbusSrv-0\\Modbus-Bit\\mbDO[30]"
#define ROBOT_RESET_ERROR_PHYSICAL_LINK "ModbusSrv-0\\Modbus-Bit\\mbDO[31]"
#define SAMPLE_SPIN_PHYSICAL_LINK       "BasicIO-1\\%Q0"

#define RECIPE_mbDO_BASE_INDEX     100
#define NUM_RECIPES  100
#define LAST_RECIPE_INDEX  (NUM_RECIPES-1)

/*CS8 mbAO index map*/
#define RECIPE_NOW_PHYSICAL_LINK       "ModbusSrv-0\\Modbus-Word\\mbAO[1]"
#define RECIPE_INSPEC_PHYSICAL_LINK    "ModbusSrv-0\\Modbus-Word\\mbAO[4]" // former DelayInterval

class CS8Controller : public asynPortDriver{
public:
  CS8Controller(const char *portName, const char *ipAddress, const char* username, const char* password);
  ~CS8Controller();
  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  virtual asynStatus setTargetRecipeNo(epicsInt32 value);
  virtual asynStatus getTargetRecipeNo(epicsInt32 *value);
  virtual asynStatus sampleIn();
  virtual asynStatus sampleOut();
  virtual asynStatus EnterStepMode();

  // These should be private but are called from C
  CStaubli_Robot* robot;
  virtual void pollerThread(void);

protected:
  /* parameters in CS8 server level*/
  int EsStop_;
  #define FIRST_CS8_PARAM EsStop_
  int ArmPower_;
  int AutoMode_;
  int ManualMode_;
  int AtHome_;
  int IsSettled_;
  int JobDone_;
  int RobotStart_;
  int RobotStop_;
  int ProdMode_;
  int StepMode_;
  int RobotPause_;
  int GetFromTray_;
  int GetFromInspec_;
  int RobotError_;
  int RobotResetError_;
  int RecipeNow_;
  int SampleSpin_;
  int SampleUnsafeSpin_;
  int RecipeInspec_;
  /*parameters in IOC level*/
  int RecipeSelect_;
  int SampleIn_;
  int SampleOut_;
  int RobotBusy_;
  #define LAST_CS8_PARAM RobotBusy_

private:

  double pollTime_;
  int forceCallback_;
  double consDelayTime_;
};

#define NUM_CS8_PARAMS  ((int)(&LAST_CS8_PARAM - &FIRST_CS8_PARAM + 1))

#endif /* STAUBLIROBOT_STAUBLIROBOTAPP_SRC_DRVCS8CONTROLLER_H_ */
