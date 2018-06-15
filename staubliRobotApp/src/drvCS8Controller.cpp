/*
 * drvCS8Controller.cpp
 *
 *  Created on: Sep 5, 2017
 *      Author: hwchen
 */

#include "drvCS8Controller.h"


static const char *driverName = "StaubliRobot";

static void pollerThreadC(void * pPvt)
{
    CS8Controller *pCS8Controller = (CS8Controller *)pPvt;
    pCS8Controller->pollerThread();
}

CS8Controller::CS8Controller(const char *portName, const char *ipAddress, const char* username, const char* password)
    : asynPortDriver(portName,
                     1,
                     NUM_CS8_PARAMS,
                     asynInt32Mask | asynDrvUserMask,  // Interfaces that we implement
                     asynInt32Mask,                    // Interfaces that do callbacks
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, /* ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=1, autoConnect=1 */
                     0,
                     0),  /* Default priority and stack size */
                     pollTime_(DEFAULT_POLL_TIME),
                     forceCallback_(1),
                     consDelayTime_(CONSECUTIVE_CMDS_DELAY_TIME)
{

  /*parameters in CS8 controller level*/
  createParam(EsStopString          , asynParamInt32,      &EsStop_);
  createParam(ArmPowerString        , asynParamInt32,      &ArmPower_);
  createParam(AutoModeString        , asynParamInt32,      &AutoMode_);
  createParam(ManualModeString      , asynParamInt32,      &ManualMode_);
  createParam(AtHomeString          , asynParamInt32,      &AtHome_);
  createParam(IsSettledString       , asynParamInt32,      &IsSettled_);
  createParam(JobDoneString         , asynParamInt32,      &JobDone_);
  createParam(RobotStartString      , asynParamInt32,      &RobotStart_);
  createParam(RobotStopString       , asynParamInt32,      &RobotStop_);
  createParam(ProdModeString        , asynParamInt32,      &ProdMode_);
  createParam(StepModeString        , asynParamInt32,      &StepMode_);
  createParam(RobotPauseString      , asynParamInt32,      &RobotPause_);
  createParam(GetFromTrayString     , asynParamInt32,      &GetFromTray_);
  createParam(GetFromInspecString   , asynParamInt32,      &GetFromInspec_);
  createParam(RobotErrorString      , asynParamInt32,      &RobotError_);
  createParam(RobotResetErrorString , asynParamInt32,      &RobotResetError_);
  createParam(RecipeNowString      , asynParamInt32,      &RecipeNow_);
  createParam(RecipeInspecString   , asynParamInt32,      &RecipeInspec_);

  /*parameters in IOC level*/
  createParam(SampleInString        , asynParamInt32,      &SampleIn_);
  createParam(SampleOutString       , asynParamInt32,      &SampleOut_);
  createParam(SampleSpinString      , asynParamInt32,      &SampleSpin_);
  createParam(SampleUnsafeSpinString, asynParamInt32,      &SampleUnsafeSpin_);
  createParam(RecipeSelectString   , asynParamInt32,      &RecipeSelect_);
  createParam(RobotBusyString       , asynParamInt32,      &RobotBusy_);

  std::string ssPortName(portName);
  std::string ssIpAddress(ipAddress);
  std::string ssUsername(username);
  std::string ssPassword(password);

  fprintf(stderr, "%s:%s, port %s, IP address %s, username %s, password %s, NUM_PARAMS %d\n",
           driverName, __func__, this->portName, ipAddress , username, password, NUM_CS8_PARAMS);

  this->robot = new CStaubli_Robot(ssPortName, ssIpAddress, ssUsername, ssPassword);

  /* Start the thread to poll digital inputs and do callbacks to
   * device support */
  epicsThreadCreate("CS8ControllerPoller",
                    epicsThreadPriorityLow,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)pollerThreadC,
                    this);
}

asynStatus CS8Controller::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
  int addr;
  int function = pasynUser->reason;
  int status=0;
  int val;
  int range;

//  this->getAddress(pasynUser, &addr);


  if (function == RecipeSelect_) {
    getIntegerParam(function, value);
  }
  else
  {
    status = asynPortDriver::readInt32(pasynUser, value);
  }

  callParamCallbacks();
  return (status==0) ? asynSuccess : asynError;
}

asynStatus CS8Controller::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int addr;
  int function = pasynUser->reason;
  int status=0;
  static const char *functionName = "writeInt32";

  this->getAddress(pasynUser, &addr);
  setIntegerParam(addr, function, value);

  if (function == RecipeSelect_) {
    /* value: 1-based receipe no. , 0 means select none*/
    if (value >0 && value <= NUM_RECIPES)
      this->setTargetRecipeNo(value);
    else
      this->setTargetRecipeNo(0);
  }
  else if (function == StepMode_)
  {
    std::vector<std::string> physicalLink;
    std::vector<double> val;
    physicalLink.push_back(STEP_MODE_PHYSICAL_LINK        ); val.push_back((double)value);
    status = this->robot->write_ios_value(physicalLink, val);
  }
  else if (function == SampleSpin_)
  {
    int isAtHome = 0;
    int isRecipeInspec = 0;
    getIntegerParam(AtHome_, &isAtHome);
    getIntegerParam(RecipeInspec_, &isRecipeInspec);

    if (isAtHome && isRecipeInspec )
    {
      std::vector<std::string> physicalLink;
      std::vector<double> val;
      physicalLink.push_back(SAMPLE_SPIN_PHYSICAL_LINK        ); val.push_back((double)(value?1.0:0.0));
      status = this->robot->write_ios_value(physicalLink, val);
    }
    else
    {
      printf("Do not permit sample spin when sample is not in.\n");
      setIntegerParam(addr, function, 0);
    }

  }
  else if (function == SampleUnsafeSpin_)
  {
    int isAtHome = 0;
    getIntegerParam(AtHome_, &isAtHome);

    if (isAtHome )
    {
      std::vector<std::string> physicalLink;
      std::vector<double> val;
      physicalLink.push_back(SAMPLE_SPIN_PHYSICAL_LINK        ); val.push_back((double)(value?1.0:0.0));
      status = this->robot->write_ios_value(physicalLink, val);
    }
    else
    {
      printf("Do not permit sample spin when sample is at home.\n");
      setIntegerParam(addr, function, 0);
    }

  }
  else if (function == RobotPause_)
  {
    std::vector<std::string> physicalLink;
    std::vector<double> val;
    physicalLink.push_back(ROBOT_PAUSE_PHYSICAL_LINK        ); val.push_back((double)value);
    status = this->robot->write_ios_value(physicalLink, val);
  }
  else if (function == SampleIn_)
  {
    sampleIn();
  }
  else if (function == SampleOut_)
  {
    sampleOut();
  }



  /*the following should not be set directly*/
  else if (function == ProdMode_)
  {
    std::vector<std::string> physicalLink;
    std::vector<double> val;
    physicalLink.push_back(PROD_MODE_PHYSICAL_LINK        ); val.push_back((double)value);
    status = this->robot->write_ios_value(physicalLink, val);
  }
  else if (function == GetFromTray_)
  {
    std::vector<std::string> physicalLink;
    std::vector<double> val;
    physicalLink.push_back(GET_FROM_TRAY_PHYSICAL_LINK        ); val.push_back((double)value);
    status = this->robot->write_ios_value(physicalLink, val);
  }
  else if (function == GetFromInspec_)
  {
    std::vector<std::string> physicalLink;
    std::vector<double> val;
    physicalLink.push_back(GET_FROM_INSPEC_PHYSICAL_LINK        ); val.push_back((double)value);
    status = this->robot->write_ios_value(physicalLink, val);
  }
  else if (function == RobotStart_)
  {
    std::vector<std::string> physicalLink;
    std::vector<double> val;
    physicalLink.push_back(ROBOT_START_PHYSICAL_LINK        ); val.push_back((double)value);
    status = this->robot->write_ios_value(physicalLink, val);
  }
  else if (function == RobotStop_)
  {
    std::vector<std::string> physicalLink;
    std::vector<double> val;
    physicalLink.push_back(ROBOT_STOP_PHYSICAL_LINK        ); val.push_back((double)value);
    status = this->robot->write_ios_value(physicalLink, val);
  }
  else
  {

  }

  callParamCallbacks(addr);

  if (status == 0) {
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
             "%s:%s, port %s, wrote %d to address %d\n",
             driverName, functionName, this->portName, value, addr);
  } else {
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
             "%s:%s, port %s, ERROR writing %d to address %d, status=%d\n",
             driverName, functionName, this->portName, value, addr, status);
  }
  return (status==0) ? asynSuccess : asynError;
}

asynStatus CS8Controller::setTargetRecipeNo(epicsInt32 receipeNo) {
  int status = 0;
  std::vector<std::string> physicalLink;
  std::vector<double> value;
  char intStr[8];
  const int receipeIndex = receipeNo -1;

  for (int i=0, j=0; i<NUM_RECIPES; i++)
  {
    sprintf(intStr,"%d",RECIPE_mbDO_BASE_INDEX+i);
    physicalLink.push_back("ModbusSrv-0\\Modbus-Bit\\mbDO["+std::string(intStr)+"]");
    value.push_back(receipeIndex==i?1:0);
    if (++j == MAX_ACCESS_IOS || i == LAST_RECIPE_INDEX )
    {
      status += this->robot->write_ios_value(physicalLink, value);
      //printf("writes_ios_value, while j=%d, i=%d\n", j, i);

      /*clear*/
      physicalLink.clear();
      value.clear();
      j=0;
    }
  }

  return (status==0) ? asynSuccess : asynError;

}

asynStatus CS8Controller::getTargetRecipeNo(epicsInt32* targetRecipeNo) {
  int status = 0;
  std::vector<std::string> physicalLink;
  std::vector<double> value;
  char intStr[8];

  for (int i=0, j=0; i<NUM_RECIPES; i++)
  {
    sprintf(intStr,"%d",RECIPE_mbDO_BASE_INDEX+i);
    physicalLink.push_back("ModbusSrv-0\\Modbus-Bit\\mbDO["+std::string(intStr)+"]");
    if (++j == MAX_ACCESS_IOS || i == LAST_RECIPE_INDEX )
    {
      this->robot->read_ios_value(physicalLink, value);
      //printf("%s: read_ios_value, while j=%d, i=%d\n", __func__, j, i);

      for (std::vector<std::string>::size_type k=0; k<physicalLink.size(); k++)
      {
        //std::cout << __func__ << physicalLink[k] << '=' << value[k] << std::endl;
        if(value[k])
        {
          *targetRecipeNo = i/MAX_ACCESS_IOS*MAX_ACCESS_IOS+k+1;
          return asynSuccess;
        }
      }

      /*clear*/
      physicalLink.clear();
      value.clear();
      j=0;
    }
  }
  /*select none*/
  *targetRecipeNo = 0;

  return asynSuccess;
}

asynStatus CS8Controller::sampleIn() {
  std::vector<std::string> physicalLink;
  std::vector<double> newValue;

  int isSampleSpin = 0;
  int isRobotPause = 1;
  int isSettled = 0;
  int isAtHome = 0;
  int receipeSelectNumber = 0;
  int receipeNowNumber = 0;
  int receipeInspecNumber = 0;

  /*verify conditions*/
  getIntegerParam(SampleSpin_,     &isSampleSpin);
  getIntegerParam(RobotPause_,     &isRobotPause);
  getIntegerParam(IsSettled_,      &isSettled);
  getIntegerParam(AtHome_,         &isAtHome);
  getIntegerParam(RecipeSelect_,  &receipeSelectNumber);
  getIntegerParam(RecipeNow_,     &receipeNowNumber);
  getIntegerParam(RecipeInspec_,  &receipeInspecNumber);
  if(isSampleSpin || isRobotPause || !isSettled || !isAtHome ||
     receipeSelectNumber==0 || receipeNowNumber !=0 || receipeInspecNumber !=0)
  {
    printf("Do not permit sample in: expected values are shown in parentheses\n"
           "isSampleSpin=%d (0), isRobotPause=%d (0), isSettled=%d (1), isAtHome=%d (1),"
           "receipeSelectNumber=%d (!=0), receipeNowNumber=%d (0), receipeInspecNumber=%d (0)\n",
           isSampleSpin, isRobotPause, isSettled, isAtHome,
           receipeSelectNumber, receipeNowNumber, receipeInspecNumber);
    return asynError;
  }

  setIntegerParam(RobotBusy_, 1); /* set back to 0 in pollerThread() */

  /*prepare for sample in*/
  physicalLink.push_back(PROD_MODE_PHYSICAL_LINK        ); newValue.push_back(0);
  physicalLink.push_back(STEP_MODE_PHYSICAL_LINK        ); newValue.push_back(1);
  physicalLink.push_back(GET_FROM_TRAY_PHYSICAL_LINK    ); newValue.push_back(1);
  physicalLink.push_back(GET_FROM_INSPEC_PHYSICAL_LINK  ); newValue.push_back(0);
  this->robot->write_ios_value(physicalLink, newValue);
  epicsThreadSleep(consDelayTime_);

  /* clear ios */
  physicalLink.clear();
  newValue.clear();

  /* Two steps to make rising edge 'start' signal */
  /* Step 1. set 'start' signal low */
  physicalLink.push_back(ROBOT_START_PHYSICAL_LINK  ); newValue.push_back(0);
  physicalLink.push_back(ROBOT_STOP_PHYSICAL_LINK   ); newValue.push_back(0);
  this->robot->write_ios_value(physicalLink, newValue);

  epicsThreadSleep(consDelayTime_);

  /* Step 2. set 'start' signal high */
  newValue[0]=1;
  this->robot->write_ios_value(physicalLink, newValue);

  return asynSuccess;
}

asynStatus CS8Controller::sampleOut() {
  std::vector<std::string> physicalLink;
  std::vector<double> newValue;

  int isSampleSpin = 0;
  int isRobotPause = 1;
  int isSettled = 0;
  int isAtHome = 0;
  int receipeSelectNumber = 0;
  int receipeNowNumber = 0;
  int receipeInspecNumber = 0;

  /*verify conditions*/
  getIntegerParam(SampleSpin_,     &isSampleSpin);
  getIntegerParam(RobotPause_,     &isRobotPause);
  getIntegerParam(IsSettled_,      &isSettled);
  getIntegerParam(AtHome_,         &isAtHome);
  getIntegerParam(RecipeSelect_,  &receipeSelectNumber);
  getIntegerParam(RecipeNow_,     &receipeNowNumber);
  getIntegerParam(RecipeInspec_,  &receipeInspecNumber);
  if(isSampleSpin || isRobotPause || !isSettled || !isAtHome ||
     receipeSelectNumber!=receipeInspecNumber || receipeNowNumber !=0 || receipeInspecNumber ==0)
  {
    printf("Do not permit sample out: expected values are shown in parentheses\n"
           "isSampleSpin=%d (0), isRobotPause=%d (0), isSettled=%d (1), isAtHome=%d (1),"
           "receipeSelectNumber=%d (receipeInspecNumber=%d), receipeNowNumber=%d (0), receipeInspecNumber=%d (!=0)\n",
           isSampleSpin, isRobotPause, isSettled, isAtHome,
           receipeSelectNumber, receipeInspecNumber, receipeNowNumber, receipeInspecNumber);
    return asynError;
  }

  setIntegerParam(RobotBusy_, 1); /* set back to 0 in pollerThread() */

  /*prepare for sample out*/
  physicalLink.push_back(PROD_MODE_PHYSICAL_LINK        ); newValue.push_back(0);
  physicalLink.push_back(STEP_MODE_PHYSICAL_LINK        ); newValue.push_back(1);
  physicalLink.push_back(GET_FROM_TRAY_PHYSICAL_LINK    ); newValue.push_back(0);
  physicalLink.push_back(GET_FROM_INSPEC_PHYSICAL_LINK  ); newValue.push_back(1);
  this->robot->write_ios_value(physicalLink, newValue);
  epicsThreadSleep(consDelayTime_);

  /* clear ios */
  physicalLink.clear();
  newValue.clear();

  /* Two steps to make rising edge 'start' signal */
  /* Step 1: set 'start' signal low */
  physicalLink.push_back(ROBOT_START_PHYSICAL_LINK  ); newValue.push_back(0);
  physicalLink.push_back(ROBOT_STOP_PHYSICAL_LINK   ); newValue.push_back(0);
  this->robot->write_ios_value(physicalLink, newValue);

  epicsThreadSleep(consDelayTime_);

  /* Step 2: set 'start' signal high */
  newValue[0]=1;
  this->robot->write_ios_value(physicalLink, newValue);

  return asynSuccess;
}

asynStatus CS8Controller::EnterStepMode() {
  std::vector<std::string> physicalLink;
  std::vector<double> newValue;
  physicalLink.push_back(PROD_MODE_PHYSICAL_LINK        ); newValue.push_back(0);
  physicalLink.push_back(STEP_MODE_PHYSICAL_LINK        ); newValue.push_back(1);
  this->robot->write_ios_value(physicalLink, newValue);
  return asynSuccess;
}

void CS8Controller::pollerThread()
{
  /* This function runs in a separate thread.  It waits for the poll time */
  static const char *functionName = "pollerThread";
  int status = 0;
  int count=0;
  std::vector<std::string> paramString[POLLER_IOS_SET_NUMBER];
  std::vector<std::string> physicalLink[POLLER_IOS_SET_NUMBER];
  std::vector<double> newValue[POLLER_IOS_SET_NUMBER];
  std::vector<double> prevValue[POLLER_IOS_SET_NUMBER];
  std::vector<int> function[POLLER_IOS_SET_NUMBER];
  int prevRecipe=-1, newRecipe;

  /*for update receipe inspec */
  int iosSetIndexRecipeNow=-1, linkIndexRecipeNow=-1, prevRecipeNow=-1;
  int iosSetIndexRecipeInspec=-1, linkIndexRecipeInspec=-1;
  int iosSetIndexGetFromInspec=-1, linkIndexGetFromInspec=-1;
  std::vector<std::string> physicalLinkRecipeInspec;
  std::vector<double> valueRecipeInspec;
  physicalLinkRecipeInspec.push_back(RECIPE_INSPEC_PHYSICAL_LINK); valueRecipeInspec.push_back(0);

  physicalLink[0].push_back(ES_STOP_PHYSICAL_LINK          ); function[0].push_back(EsStop_         ); paramString[0].push_back(EsStopString         );
  physicalLink[0].push_back(ARM_POWER_PHYSICAL_LINK        ); function[0].push_back(ArmPower_       ); paramString[0].push_back(ArmPowerString       );
  physicalLink[0].push_back(AUTO_MODE_PHYSICAL_LINK        ); function[0].push_back(AutoMode_       ); paramString[0].push_back(AutoModeString       );
  physicalLink[0].push_back(MANUAL_MODE_PHYSICAL_LINK      ); function[0].push_back(ManualMode_     ); paramString[0].push_back(ManualModeString     );
  physicalLink[0].push_back(AT_HOME_PHYSICAL_LINK          ); function[0].push_back(AtHome_         ); paramString[0].push_back(AtHomeString         );
  physicalLink[0].push_back(IS_SETTLED_PHYSICAL_LINK       ); function[0].push_back(IsSettled_      ); paramString[0].push_back(IsSettledString      );
  physicalLink[0].push_back(JOB_DONE_PHYSICAL_LINK         ); function[0].push_back(JobDone_        ); paramString[0].push_back(JobDoneString        );
  physicalLink[0].push_back(ROBOT_START_PHYSICAL_LINK      ); function[0].push_back(RobotStart_     ); paramString[0].push_back(RobotStartString     );
  physicalLink[0].push_back(ROBOT_STOP_PHYSICAL_LINK       ); function[0].push_back(RobotStop_      ); paramString[0].push_back(RobotStopString      );

  physicalLink[1].push_back(PROD_MODE_PHYSICAL_LINK        ); function[1].push_back(ProdMode_       ); paramString[1].push_back(ProdModeString       );
  physicalLink[1].push_back(STEP_MODE_PHYSICAL_LINK        ); function[1].push_back(StepMode_       ); paramString[1].push_back(StepModeString       );
  physicalLink[1].push_back(ROBOT_PAUSE_PHYSICAL_LINK      ); function[1].push_back(RobotPause_     ); paramString[1].push_back(RobotPauseString     );
  physicalLink[1].push_back(GET_FROM_TRAY_PHYSICAL_LINK    ); function[1].push_back(GetFromTray_    ); paramString[1].push_back(GetFromTrayString    );
  physicalLink[1].push_back(GET_FROM_INSPEC_PHYSICAL_LINK  ); function[1].push_back(GetFromInspec_  ); paramString[1].push_back(GetFromInspecString  );
  physicalLink[1].push_back(ROBOT_EROR_PHYSICAL_LINK       ); function[1].push_back(RobotError_     ); paramString[1].push_back(RobotErrorString     );
  physicalLink[1].push_back(ROBOT_RESET_ERROR_PHYSICAL_LINK); function[1].push_back(RobotResetError_); paramString[1].push_back(RobotResetErrorString);
  physicalLink[1].push_back(RECIPE_NOW_PHYSICAL_LINK      ); function[1].push_back(RecipeNow_     ); paramString[1].push_back(RecipeNowString     );
  physicalLink[1].push_back(SAMPLE_SPIN_PHYSICAL_LINK      ); function[1].push_back(SampleSpin_     ); paramString[1].push_back(SampleSpinString     );
  physicalLink[1].push_back(RECIPE_INSPEC_PHYSICAL_LINK   ); function[1].push_back(RecipeInspec_  ); paramString[1].push_back(RecipeInspecString     );

  /*initialize*/
  lock();

  robot->read_ios_value(physicalLink[0], newValue[0]);
  prevValue[0]=newValue[0];

  robot->read_ios_value(physicalLink[1], newValue[1]);
  prevValue[1]=newValue[1];

  unlock();

  /* find index of RECIPE_NOW_PHYSICAL_LINK*/
  for(int i=0;i<POLLER_IOS_SET_NUMBER;i++)
  {
    size_t pos = std::find(physicalLink[i].begin(),physicalLink[i].end(), std::string(RECIPE_NOW_PHYSICAL_LINK)) - physicalLink[i].begin();
    if(pos < physicalLink[i].size())
    {
      iosSetIndexRecipeNow = i;
      linkIndexRecipeNow = pos;
      printf("RecipeNow index [%d][%d]\n", iosSetIndexRecipeNow, linkIndexRecipeNow);
    }
  }

  /* find index of RECIPE_INSPEC_PHYSICAL_LINK*/
  for(int i=0;i<POLLER_IOS_SET_NUMBER;i++)
  {
    size_t pos = std::find(physicalLink[i].begin(),physicalLink[i].end(), std::string(RECIPE_INSPEC_PHYSICAL_LINK)) - physicalLink[i].begin();
    if(pos < physicalLink[i].size())
    {
      iosSetIndexRecipeInspec = i;
      linkIndexRecipeInspec = pos;
      printf("RecipeInspec index [%d][%d]\n", iosSetIndexRecipeInspec, linkIndexRecipeInspec);
    }
  }

  /* find index of GET_FROM__INSPEC_PHYSICAL_LINK*/
  for(int i=0;i<POLLER_IOS_SET_NUMBER;i++)
  {
    size_t pos = std::find(physicalLink[i].begin(),physicalLink[i].end(), std::string(GET_FROM_INSPEC_PHYSICAL_LINK)) - physicalLink[i].begin();
    if(pos < physicalLink[i].size())
    {
      iosSetIndexGetFromInspec = i;
      linkIndexGetFromInspec = pos;
      printf("RecipeInspec index [%d][%d]\n", iosSetIndexGetFromInspec, linkIndexGetFromInspec);
    }
  }

  while(1)
  {
    lock();

    /*CS8 controller level parameters */
    for(int i=0;i<POLLER_IOS_SET_NUMBER;i++)
    {
      if(physicalLink[i].empty()) continue;

      robot->read_ios_value(physicalLink[i], newValue[i]);

      if(newValue[i].size() != physicalLink[i].size() ) continue;

      for (std::vector<std::string>::size_type k=0; k<physicalLink[i].size(); k++)
      {
        //printf("%s: %40s value = %d %s\n",__func__, physicalLink[i][k].c_str(), (int)newValue[i][k], paramString[i][k].c_str());
        if(newValue[i][k] != prevValue[i][k] || forceCallback_)
        {
          setIntegerParam(function[i][k], newValue[i][k]);
          printf("%s: %s value update: new=%d, prev=%d\n", __func__, paramString[i][k].c_str(), (int)newValue[i][k], (int)prevValue[i][k]);
          prevValue[i][k] = newValue[i][k];
        }
      }
    }

    /* update receipt inspec*/
    if(prevRecipeNow != newValue[iosSetIndexRecipeNow][linkIndexRecipeNow])
    { /* RecipeNow either 0 to N or N to 0*/

      prevRecipeNow = newValue[iosSetIndexRecipeNow][linkIndexRecipeNow];
      printf("%s: RecipeNow=%d\n",__func__, prevRecipeNow);

      /* at sample in  begin: RecipeNow goes to N, RecipeInspec is 0, GetFromInspec is 0 */
      if(newValue[iosSetIndexRecipeNow][linkIndexRecipeNow] !=0 &&
         newValue[iosSetIndexRecipeInspec][linkIndexRecipeInspec] == 0)
      {
        printf("%s: at sample in begin\n",__func__);
        valueRecipeInspec[0] = newValue[iosSetIndexRecipeNow][linkIndexRecipeNow];
        /* write value to CS8 controller */
        this->robot->write_ios_value(physicalLinkRecipeInspec, valueRecipeInspec);
        setIntegerParam(function[iosSetIndexRecipeInspec][linkIndexRecipeInspec],
                        newValue[iosSetIndexRecipeInspec][linkIndexRecipeInspec]);
      }
      /* at sample in  end:   RecipeNow goes to 0, RecipeInspec is N, GetFromInspec is 0 */
      else if(newValue[iosSetIndexRecipeNow][linkIndexRecipeNow] == 0 &&
                    newValue[iosSetIndexRecipeInspec][linkIndexRecipeInspec] != 0 &&
                    newValue[iosSetIndexGetFromInspec][linkIndexGetFromInspec] == 0)
      {
        setIntegerParam(RobotBusy_, 0);
        setIntegerParam(SampleIn_, 0);
      }
      /* at sample out begin: RecipeNow goes to N, RecipeInspec is N, GetFromInspec is 1
       * do nothing here */

      /* at sample out end:   RecipeNow goes to 0, RecipeInspec is N, GetFromInspec is 1 */
      else if(newValue[iosSetIndexRecipeNow][linkIndexRecipeNow] == 0 &&
              newValue[iosSetIndexRecipeInspec][linkIndexRecipeInspec] != 0 &&
              newValue[iosSetIndexGetFromInspec][linkIndexGetFromInspec] == 1)
      {
        printf("%s: at sample out end\n",__func__);
        valueRecipeInspec[0] = 0;
        /* write value to CS8 controller */
        this->robot->write_ios_value(physicalLinkRecipeInspec, valueRecipeInspec);

        setIntegerParam(RobotBusy_, 0);
        setIntegerParam(SampleOut_, 0);
      }

    }

    /*IOC level parameters*/
    this->getTargetRecipeNo(&newRecipe);
    if (newRecipe != prevRecipe || forceCallback_)
    {
      setIntegerParam(RecipeSelect_, newRecipe);
      printf("%s: Receipt select update: new=%d, prev=%d\n", __func__, newRecipe, prevRecipe);
      prevRecipe = newRecipe;
    }

    forceCallback_ = 0;
    callParamCallbacks();

    unlock();
    epicsThreadSleep(pollTime_);
  }
}

CS8Controller::~CS8Controller()
{
  delete this->robot;
}

/** Configuration command, called directly or from iocsh */
extern "C" int CS8Config(const char *portName, const char *ipAddress, const char* username, const char* password)
{
  CS8Controller *pCS8 = new CS8Controller(portName, ipAddress, username, password);
  pCS8 = NULL;  /* This is just to avoid compiler warnings */
  return(asynSuccess);
}


static const iocshArg configArg0 = { "Port name",      iocshArgString};
static const iocshArg configArg1 = { "IP address",     iocshArgString};
static const iocshArg configArg2 = { "Username",       iocshArgString};
static const iocshArg configArg3 = { "Password",       iocshArgString};
static const iocshArg * const configArgs[] = {&configArg0,
                                              &configArg1,
                                              &configArg2,
                                              &configArg3};
static const iocshFuncDef configFuncDef = {"CS8Config", 4, configArgs};
static void configCallFunc(const iocshArgBuf *args)
{
  CS8Config(args[0].sval, args[1].sval, args[2].sval, args[3].sval);
}

void drvCS8Register(void)
{
  iocshRegister(&configFuncDef,configCallFunc);
}

extern "C" {
epicsExportRegistrar(drvCS8Register);
}
