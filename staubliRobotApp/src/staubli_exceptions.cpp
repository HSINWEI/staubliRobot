#include "staubli_exceptions.h"
#include <sstream>
#include <string.h>
#include <stdio.h>

const std::string staubli_error_message="[CStaubli class] - ";

CStaubliException::CStaubliException(const std::string& where,const std::string& error_msg,int error_code):CException(where,staubli_error_message)
{
  std::stringstream text;

  this->error_msg+=error_msg;
  this->error_msg+=" - error_code: ";
  this->error_code=error_code;
  text << error_code;
  this->error_msg+=text.str();
}

int CStaubliException::get_error_code(void)
{
  return this->error_code;
}
