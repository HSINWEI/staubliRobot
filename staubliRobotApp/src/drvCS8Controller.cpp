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
                     forceCallback_(1)
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
  createParam(ReceipeNowString      , asynParamInt32,      &ReceipeNow_);

  /*parameters in IOC level*/
  createParam(SampleInString        , asynParamInt32,      &SampleIn_);
  createParam(SampleOutString       , asynParamInt32,      &SampleOut_);
  createParam(SampleSpinString      , asynParamInt32,      &SampleSpin_);
  createParam(ReceipeSelectString   , asynParamInt32,      &ReceipeSelect_);

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


  if (function == ReceipeSelect_) {
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

  if (function == ReceipeSelect_) {
    /* value: 1-based receipe no. , 0 means select none*/
    if (value >0 && value <= NUM_RECEIPES)
      this->setTargetReceipeNo(value);
    else
      this->setTargetReceipeNo(0);
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
    std::vector<std::string> physicalLink;
    std::vector<double> val;
    physicalLink.push_back(SAMPLE_SPIN_PHYSICAL_LINK        ); val.push_back((double)value);
    status = this->robot->write_ios_value(physicalLink, val);
  }
  else if (function == RobotPause_)
  {
    std::vector<std::string> physicalLink;
    std::vector<double> val;
    physicalLink.push_back(ROBOT_PAUSE_PHYSICAL_LINK        ); val.push_back((double)value);
    status = this->robot->write_ios_value(physicalLink, val);
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

asynStatus CS8Controller::setTargetReceipeNo(epicsInt32 receipeNo) {
  int status = 0;
  std::vector<std::string> physicalLink;
  std::vector<double> value;
  char intStr[8];
  const int receipeIndex = receipeNo -1;

  for (int i=0, j=0; i<NUM_RECEIPES; i++)
  {
    sprintf(intStr,"%d",RECEIPE_mbDO_BASE_INDEX+i);
    physicalLink.push_back("ModbusSrv-0\\Modbus-Bit\\mbDO["+std::string(intStr)+"]");
    value.push_back(receipeIndex==i?1:0);
    if (++j == MAX_ACCESS_IOS || i == LAST_RECEIPE_INDEX )
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

asynStatus CS8Controller::getTargetReceipeNo(epicsInt32* targetReceipeNo) {
  int status = 0;
  std::vector<std::string> physicalLink;
  std::vector<double> value;
  char intStr[8];

  for (int i=0, j=0; i<NUM_RECEIPES; i++)
  {
    sprintf(intStr,"%d",RECEIPE_mbDO_BASE_INDEX+i);
    physicalLink.push_back("ModbusSrv-0\\Modbus-Bit\\mbDO["+std::string(intStr)+"]");
    if (++j == MAX_ACCESS_IOS || i == LAST_RECEIPE_INDEX )
    {
      this->robot->read_ios_value(physicalLink, value);
      //printf("%s: read_ios_value, while j=%d, i=%d\n", __func__, j, i);

      for (std::vector<std::string>::size_type k=0; k<physicalLink.size(); k++)
      {
        //std::cout << __func__ << physicalLink[k] << '=' << value[k] << std::endl;
        if(value[k])
        {
          *targetReceipeNo = i/MAX_ACCESS_IOS*MAX_ACCESS_IOS+k+1;
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
  *targetReceipeNo = 0;

  return asynSuccess;
}

asynStatus CS8Controller::sampleIn() {
  return asynSuccess;
}

asynStatus CS8Controller::sampleOut() {
  return asynSuccess;
}

asynStatus CS8Controller::sampleSpin(epicsInt32 value) {
  return asynSuccess;
}

asynStatus CS8Controller::robotPause(epicsInt32 value) {
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
  int prevReceipe=-1, newReceipe;

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
  physicalLink[1].push_back(RECEIPE_NOW_PHYSICAL_LINK      ); function[1].push_back(ReceipeNow_     ); paramString[1].push_back(ReceipeNowString     );
  physicalLink[1].push_back(SAMPLE_SPIN_PHYSICAL_LINK      ); function[1].push_back(SampleSpin_     ); paramString[1].push_back(SampleSpinString     );

  /*initialize*/
  lock();

  robot->read_ios_value(physicalLink[0], newValue[0]);
  prevValue[0]=newValue[0];

  robot->read_ios_value(physicalLink[1], newValue[1]);
  prevValue[1]=newValue[1];

  unlock();

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
          printf("%s: %s value update: new=%d, prev=%d\n", __func__, physicalLink[i][k].c_str(), (int)newValue[i][k], (int)prevValue[i][k]);
          prevValue[i][k] = newValue[i][k];
        }
      }
    }

    /*IOC level parameters*/
    this->getTargetReceipeNo(&newReceipe);
    if (newReceipe != prevReceipe || forceCallback_)
    {
      setIntegerParam(ReceipeSelect_, newReceipe);
      printf("%s: Receipt select update: new=%d, prev=%d\n", __func__, newReceipe, prevReceipe);
      prevReceipe = newReceipe;
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
