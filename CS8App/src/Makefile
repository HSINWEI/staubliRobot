TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = CS8
# CS8.dbd will be created and installed
DBD += CS8.dbd

# CS8.dbd will be made up from these files:
CS8_DBD += base.dbd
CS8_DBD += asyn.dbd
CS8_DBD += staubliRobot.dbd

# Include dbd files from all support applications:
#CS8_DBD += xxx.dbd

# Add all the support libraries needed by this IOC
CS8_LIBS += asyn
CS8_LIBS += staubliRobot

# CS8_registerRecordDeviceDriver.cpp derives from CS8.dbd
CS8_SRCS += CS8_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
CS8_SRCS_DEFAULT += CS8Main.cpp
CS8_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#CS8_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
CS8_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

