TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += staubliRobot

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install staubliRobot.dbd into <top>/dbd
DBD += staubliRobot.dbd

# specify all source files to be compiled and added to the library
staubliRobot_SRCS += stdsoap2.cpp soapC.cpp soapCS8ServerV0Proxy.cpp soapCS8ServerV2Proxy.cpp
staubliRobot_SRCS += exceptions.cpp staubli_exceptions.cpp staubli_robot.cpp
staubliRobot_SRCS += drvCS8Controller.cpp

staubliRobot_LIBS += $(EPICS_BASE_IOC_LIBS)

#staubliRobot_USR_CXX_FLAGS += -lgsoap+

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

