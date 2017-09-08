#include "staubli_robot.h"

#include "CS8ServerV0.nsmap"
#include "staubli_exceptions.h"

const char *error_messages[]={"Everything is okay",
                              "Impossible to log in into to the robot",
                              "Invalid robot identifier",
                              "No feedback information received from the robot",
                              "The initial position of the trajectory does not coincide with the current position of the robot",
                              "The current step is too small and can not be executed successfully",
                              "Impossible to get the joint ranges",
                              "Power can not be tuned on while the robot is in motion",
                              "A timeout has elapsed while waiting for the power to turn on",
                              "A timeout has elapsed while waiting for the power to turn off",
                              "Power can only be turned on or off in remote mode",
                              "Invalid shoulder configuration",
                              "Invalid elbow configuration",
                              "Invalid wrist configuration",
                              "Leave and reach values must be positive",
                              "Invalid feedback rate",
                              "Invalid pose vector length",
                              "A trajectory is being executed",
                              "Invalid cartesian velocity",
                              "The robot is not ready to move",
                              "One or more parameters are invalid",
                              "One or more parameters are misused",
                              "Unexpected error",
                              "One or more joint angles are out of range",
                              "One or more jointi velocities are too high"
                              "No joint angles found for the given cartesian configuration",
                              "Solution joints out of range",
                              "Solution joints out of the workspace",
                              "Invalid robot configuration",
                              "Invalid robot orientation",
                              "Unsupported kinematics",
                              "Unconstrained frame",
                              "No Cartesian configuration found for the given joint angles",
                              "Trying to set an invalid value for the maximum cartesian speed"};

CStaubli_Robot::CStaubli_Robot(std::string &robot_id, std::string &ip_address, std::string &username, std::string &password)
{
  _ns1__logout logout;
  _ns1__logoutResponse logout_response;
  
  // try to log in into the robot
  this->end_point_v0 = "http://" + ip_address + ":5653/";
  this->end_point_v2 = this->end_point_v0 + "CS8ServerV2";
  this->CS8_server_v0=NULL;
  this->CS8_server_v2=NULL;
  try{
    this->CS8_server_v0 = new CS8ServerV0Proxy();
    this->CS8_server_v0->soap_endpoint = this->end_point_v0.c_str();
    this->user_name=username;
    this->password=password;
    this->login.user=&this->user_name;
    this->login.pwd=&this->password;
    this->CS8_server_v0->login(&this->login,&this->login_response);
    this->sid = login_response.sid;
    if(login_response.sid==0) // impossible to connect
    {
      this->logged_in=false;
      /* handle exceptions */
      throw CStaubliException(_HERE_,error_messages[staubli_login_error],staubli_login_error);
    }
    std::cout << "Session Id =" << login_response.sid << std::endl;
    this->logged_in=true;
    this->CS8_server_v2=new CS8ServerV2Proxy();
    this->CS8_server_v2->soap_header(&(this->login_response.sid));
    this->CS8_server_v2->soap_endpoint = this->end_point_v2.c_str();
  }catch(...){
    this->clear();
    throw;
  }
}
 
void CStaubli_Robot::clear(void)
{
  _ns1__logout logout;
  _ns1__logoutResponse logout_response;
  int status;

  if(this->CS8_server_v2!=NULL)
  {
    delete this->CS8_server_v2;
    this->CS8_server_v2=NULL;
  }
  if(this->logged_in)
  {
    status = this->CS8_server_v0->logout(&logout, &logout_response);
    std::cout << "Successfully logged out!!. status=" << status << std::endl;
  }
  if(this->CS8_server_v0!=NULL)
  {
    delete this->CS8_server_v0;
    this->CS8_server_v0=NULL;
  }
}

void CStaubli_Robot::read_ios_value(std::vector<std::string> &physicalLink, std::vector<double> &value)
{
  //the max size of physicalLink is 10
  _ns7__readIos readIos;
  _ns7__readIosResponse readIosResponse;
  ns7__SoapPhysicalIoLinks soapPhysicalIoLinks;
  soapPhysicalIoLinks.PhysicalPath = physicalLink;
  readIos.ios = &soapPhysicalIoLinks;
  readIos.x_getDescription = false;
  this->CS8_server_v2->readIos(&readIos, &readIosResponse);
  if(!value.empty()) value.clear();
  for (unsigned i=0; i<readIosResponse.state->PhysicalIoState.size(); i++)
  {
    //std::cout << __func__<< ": " << physicalLink[i] << " value = " << readIosResponse.state->PhysicalIoState[i]->value<< std::endl;
    value.push_back(readIosResponse.state->PhysicalIoState[i]->value);
  }
}

int CStaubli_Robot::write_ios_value(std::vector<std::string> &physicalLink, std::vector<double> &value)
{
  int status=0;
  _ns7__writeIos writeIos;
  _ns7__writeIosResponse writeIosResponse;
  ns7__SoapPhysicalIoLinks soapPhysicalIoLinks;
  ns7__SoapPhysicalIoValues soapPhysicalIoValues;
  soapPhysicalIoLinks.PhysicalPath = physicalLink;
  soapPhysicalIoValues.PhysicalIoValue = value;
  writeIos.ios = &soapPhysicalIoLinks;
  writeIos.values = &soapPhysicalIoValues;

  status = this->CS8_server_v2->writeIos(&writeIos, &writeIosResponse);
  for (unsigned i=0; i<writeIosResponse.out->PhysicalIoResponse.size(); i++)
  {
    //std::cout << __func__ << ": " << physicalLink[i] << " success = " << writeIosResponse.out->PhysicalIoResponse[i]->success << std::endl;
    if(!(writeIosResponse.out->PhysicalIoResponse[i]->success))
      status++;
  }
  return status;
}

CStaubli_Robot::~CStaubli_Robot()
{
  this->clear();
}

int CStaubli_Robot::get_sid() {
  return this->sid;
}
