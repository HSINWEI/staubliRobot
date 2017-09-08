#ifndef _STAUBLI_EXCEPTIONS
#define _STAUBLI_EXCEPTIONS

#include "exceptions.h"

class CStaubliException : public CException
{
  private:
    int error_code;
  public:
    CStaubliException(const std::string& where,const std::string& error_msg,int error_code);
    int get_error_code(void);
};

#endif
