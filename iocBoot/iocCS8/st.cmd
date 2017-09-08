#!../../bin/linux-x86_64/CS8

## You may have to change CS8 to something else
## everywhere it appears in this file

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/CS8.dbd"
CS8_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

dbLoadTemplate("CS8.substitutions")

CS8Config("CS8", "172.18.6.220", "***REMOVED***", "***REMOVED***")

iocInit

## Start any sequence programs
#seq sncxxx,"user=hwchenHost"
