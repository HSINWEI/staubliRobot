/* soapCS8ServerV2Proxy.h
   Generated by gSOAP 2.8.52 for CS8Server.h

gSOAP XML Web services tools
Copyright (C) 2000-2017, Robert van Engelen, Genivia Inc. All Rights Reserved.
The soapcpp2 tool and its generated software are released under the GPL.
This program is released under the GPL with the additional exemption that
compiling, linking, and/or using OpenSSL is allowed.
--------------------------------------------------------------------------------
A commercial use license is available from Genivia Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

#ifndef soapCS8ServerV2Proxy_H
#define soapCS8ServerV2Proxy_H
#include "../soapH.h"

    class SOAP_CMAC CS8ServerV2Proxy {
      public:
        /// Context to manage proxy IO and data
        struct soap *soap;
        bool soap_own; ///< flag indicating that this context is owned by this proxy when context is shared
        /// Endpoint URL of service 'CS8ServerV2Proxy' (change as needed)
        const char *soap_endpoint;
        /// Variables globally declared in CS8Server.h, if any
        /// Construct a proxy with new managing context
        CS8ServerV2Proxy();
        /// Copy constructor
        CS8ServerV2Proxy(const CS8ServerV2Proxy& rhs);
        /// Construct proxy given a shared managing context
        CS8ServerV2Proxy(struct soap*);
        /// Constructor taking an endpoint URL
        CS8ServerV2Proxy(const char *endpoint);
        /// Constructor taking input and output mode flags for the new managing context
        CS8ServerV2Proxy(soap_mode iomode);
        /// Constructor taking endpoint URL and input and output mode flags for the new managing context
        CS8ServerV2Proxy(const char *endpoint, soap_mode iomode);
        /// Constructor taking input and output mode flags for the new managing context
        CS8ServerV2Proxy(soap_mode imode, soap_mode omode);
        /// Destructor deletes non-shared managing context only (use destroy() to delete deserialized data)
        virtual ~CS8ServerV2Proxy();
        /// Initializer used by constructors
        virtual void CS8ServerV2Proxy_init(soap_mode imode, soap_mode omode);
        /// Return a copy that has a new managing context with the same engine state
        virtual CS8ServerV2Proxy *copy();
        /// Copy assignment
        CS8ServerV2Proxy& operator=(const CS8ServerV2Proxy&);
        /// Delete all deserialized data (uses soap_destroy() and soap_end())
        virtual void destroy();
        /// Delete all deserialized data and reset to default
        virtual void reset();
        /// Disables and removes SOAP Header from message by setting soap->header = NULL
        virtual void soap_noheader();
        /// Add SOAP Header to message
        virtual void soap_header(int *ns1__sessionId);
        /// Get SOAP Header structure (i.e. soap->header, which is NULL when absent)
        virtual ::SOAP_ENV__Header *soap_header();
        /// Get SOAP Fault structure (i.e. soap->fault, which is NULL when absent)
        virtual ::SOAP_ENV__Fault *soap_fault();
        /// Get SOAP Fault string (NULL when absent)
        virtual const char *soap_fault_string();
        /// Get SOAP Fault detail as string (NULL when absent)
        virtual const char *soap_fault_detail();
        /// Close connection (normally automatic, except for send_X ops)
        virtual int soap_close_socket();
        /// Force close connection (can kill a thread blocked on IO)
        virtual int soap_force_close_socket();
        /// Print fault
        virtual void soap_print_fault(FILE*);
    #ifndef WITH_LEAN
    #ifndef WITH_COMPAT
        /// Print fault to stream
        virtual void soap_stream_fault(std::ostream&);
    #endif
        /// Write fault to buffer
        virtual char *soap_sprint_fault(char *buf, size_t len);
    #endif
        /// Web service operation 'setBreakpoints' (returns SOAP_OK or error code)
        virtual int setBreakpoints(_ns7__setBreakpoints *ns7__setBreakpoints, _ns7__setBreakpointsResponse *ns7__setBreakpointsResponse)
        { return this->setBreakpoints(NULL, NULL, ns7__setBreakpoints, ns7__setBreakpointsResponse); }
        virtual int setBreakpoints(const char *soap_endpoint, const char *soap_action, _ns7__setBreakpoints *ns7__setBreakpoints, _ns7__setBreakpointsResponse *ns7__setBreakpointsResponse);
        /// Web service operation 'getBreakpoints' (returns SOAP_OK or error code)
        virtual int getBreakpoints(_ns7__getBreakpoints *ns7__getBreakpoints, _ns7__getBreakpointsResponse *ns7__getBreakpointsResponse)
        { return this->getBreakpoints(NULL, NULL, ns7__getBreakpoints, ns7__getBreakpointsResponse); }
        virtual int getBreakpoints(const char *soap_endpoint, const char *soap_action, _ns7__getBreakpoints *ns7__getBreakpoints, _ns7__getBreakpointsResponse *ns7__getBreakpointsResponse);
        /// Web service operation 'clearBreakpoints' (returns SOAP_OK or error code)
        virtual int clearBreakpoints(_ns7__clearBreakpoints *ns7__clearBreakpoints, _ns7__clearBreakpointsResponse *ns7__clearBreakpointsResponse)
        { return this->clearBreakpoints(NULL, NULL, ns7__clearBreakpoints, ns7__clearBreakpointsResponse); }
        virtual int clearBreakpoints(const char *soap_endpoint, const char *soap_action, _ns7__clearBreakpoints *ns7__clearBreakpoints, _ns7__clearBreakpointsResponse *ns7__clearBreakpointsResponse);
        /// Web service operation 'clearAllBreakpoints' (returns SOAP_OK or error code)
        virtual int clearAllBreakpoints(_ns7__clearAllBreakpoints *ns7__clearAllBreakpoints, _ns7__clearAllBreakpointsResponse *ns7__clearAllBreakpointsResponse)
        { return this->clearAllBreakpoints(NULL, NULL, ns7__clearAllBreakpoints, ns7__clearAllBreakpointsResponse); }
        virtual int clearAllBreakpoints(const char *soap_endpoint, const char *soap_action, _ns7__clearAllBreakpoints *ns7__clearAllBreakpoints, _ns7__clearAllBreakpointsResponse *ns7__clearAllBreakpointsResponse);
        /// Web service operation 'getTasks' (returns SOAP_OK or error code)
        virtual int getTasks(_ns7__getTasks *ns7__getTasks, _ns7__getTasksResponse *ns7__getTasksResponse)
        { return this->getTasks(NULL, NULL, ns7__getTasks, ns7__getTasksResponse); }
        virtual int getTasks(const char *soap_endpoint, const char *soap_action, _ns7__getTasks *ns7__getTasks, _ns7__getTasksResponse *ns7__getTasksResponse);
        /// Web service operation 'taskSuspend' (returns SOAP_OK or error code)
        virtual int taskSuspend(_ns7__taskSuspend *ns7__taskSuspend, _ns7__taskSuspendResponse *ns7__taskSuspendResponse)
        { return this->taskSuspend(NULL, NULL, ns7__taskSuspend, ns7__taskSuspendResponse); }
        virtual int taskSuspend(const char *soap_endpoint, const char *soap_action, _ns7__taskSuspend *ns7__taskSuspend, _ns7__taskSuspendResponse *ns7__taskSuspendResponse);
        /// Web service operation 'taskResume' (returns SOAP_OK or error code)
        virtual int taskResume(_ns7__taskResume *ns7__taskResume, _ns7__taskResumeResponse *ns7__taskResumeResponse)
        { return this->taskResume(NULL, NULL, ns7__taskResume, ns7__taskResumeResponse); }
        virtual int taskResume(const char *soap_endpoint, const char *soap_action, _ns7__taskResume *ns7__taskResume, _ns7__taskResumeResponse *ns7__taskResumeResponse);
        /// Web service operation 'taskKill' (returns SOAP_OK or error code)
        virtual int taskKill(_ns7__taskKill *ns7__taskKill, _ns7__taskKillResponse *ns7__taskKillResponse)
        { return this->taskKill(NULL, NULL, ns7__taskKill, ns7__taskKillResponse); }
        virtual int taskKill(const char *soap_endpoint, const char *soap_action, _ns7__taskKill *ns7__taskKill, _ns7__taskKillResponse *ns7__taskKillResponse);
        /// Web service operation 'taskStep' (returns SOAP_OK or error code)
        virtual int taskStep(_ns7__taskStep *ns7__taskStep, _ns7__taskStepResponse *ns7__taskStepResponse)
        { return this->taskStep(NULL, NULL, ns7__taskStep, ns7__taskStepResponse); }
        virtual int taskStep(const char *soap_endpoint, const char *soap_action, _ns7__taskStep *ns7__taskStep, _ns7__taskStepResponse *ns7__taskStepResponse);
        /// Web service operation 'subscribeToControllerEvents' (returns SOAP_OK or error code)
        virtual int subscribeToControllerEvents(_ns7__subscribeToControllerEvents *ns7__subscribeToControllerEvents, _ns7__subscribeResponse *ns7__subscribeResponse)
        { return this->subscribeToControllerEvents(NULL, NULL, ns7__subscribeToControllerEvents, ns7__subscribeResponse); }
        virtual int subscribeToControllerEvents(const char *soap_endpoint, const char *soap_action, _ns7__subscribeToControllerEvents *ns7__subscribeToControllerEvents, _ns7__subscribeResponse *ns7__subscribeResponse);
        /// Web service operation 'unsubscribeToControllerEvents' (returns SOAP_OK or error code)
        virtual int unsubscribeToControllerEvents(_ns7__unsubscribeToControllerEvents *ns7__unsubscribeToControllerEvents, _ns7__unsubscribeResponse *ns7__unsubscribeResponse)
        { return this->unsubscribeToControllerEvents(NULL, NULL, ns7__unsubscribeToControllerEvents, ns7__unsubscribeResponse); }
        virtual int unsubscribeToControllerEvents(const char *soap_endpoint, const char *soap_action, _ns7__unsubscribeToControllerEvents *ns7__unsubscribeToControllerEvents, _ns7__unsubscribeResponse *ns7__unsubscribeResponse);
        /// Web service operation 'getCallStack' (returns SOAP_OK or error code)
        virtual int getCallStack(_ns7__getCallStack *ns7__getCallStack, _ns7__getCallStackResponse *ns7__getCallStackResponse)
        { return this->getCallStack(NULL, NULL, ns7__getCallStack, ns7__getCallStackResponse); }
        virtual int getCallStack(const char *soap_endpoint, const char *soap_action, _ns7__getCallStack *ns7__getCallStack, _ns7__getCallStackResponse *ns7__getCallStackResponse);
        /// Web service operation 'getStackFrame' (returns SOAP_OK or error code)
        virtual int getStackFrame(_ns7__getStackFrame *ns7__getStackFrame, _ns7__getStackFrameResponse *ns7__getStackFrameResponse)
        { return this->getStackFrame(NULL, NULL, ns7__getStackFrame, ns7__getStackFrameResponse); }
        virtual int getStackFrame(const char *soap_endpoint, const char *soap_action, _ns7__getStackFrame *ns7__getStackFrame, _ns7__getStackFrameResponse *ns7__getStackFrameResponse);
        /// Web service operation 'getWatches' (returns SOAP_OK or error code)
        virtual int getWatches(_ns7__getWatches *ns7__getWatches, _ns7__getWatchResponse *ns7__getWatchResponse)
        { return this->getWatches(NULL, NULL, ns7__getWatches, ns7__getWatchResponse); }
        virtual int getWatches(const char *soap_endpoint, const char *soap_action, _ns7__getWatches *ns7__getWatches, _ns7__getWatchResponse *ns7__getWatchResponse);
        /// Web service operation 'execVal3' (returns SOAP_OK or error code)
        virtual int execVal3(_ns7__execVal3 *ns7__execVal3, _ns7__execVal3Response *ns7__execVal3Response)
        { return this->execVal3(NULL, NULL, ns7__execVal3, ns7__execVal3Response); }
        virtual int execVal3(const char *soap_endpoint, const char *soap_action, _ns7__execVal3 *ns7__execVal3, _ns7__execVal3Response *ns7__execVal3Response);
        /// Web service operation 'setProjectAsModified' (returns SOAP_OK or error code)
        virtual int setProjectAsModified(_ns7__setProjectAsModified *ns7__setProjectAsModified, _ns7__setProjectAsModifiedResponse *ns7__setProjectAsModifiedResponse)
        { return this->setProjectAsModified(NULL, NULL, ns7__setProjectAsModified, ns7__setProjectAsModifiedResponse); }
        virtual int setProjectAsModified(const char *soap_endpoint, const char *soap_action, _ns7__setProjectAsModified *ns7__setProjectAsModified, _ns7__setProjectAsModifiedResponse *ns7__setProjectAsModifiedResponse);
        /// Web service operation 'setCurrentInstruction' (returns SOAP_OK or error code)
        virtual int setCurrentInstruction(_ns7__setCurrentInstruction *ns7__setCurrentInstruction, _ns7__setCurrentInstructionResponse *ns7__setCurrentInstructionResponse)
        { return this->setCurrentInstruction(NULL, NULL, ns7__setCurrentInstruction, ns7__setCurrentInstructionResponse); }
        virtual int setCurrentInstruction(const char *soap_endpoint, const char *soap_action, _ns7__setCurrentInstruction *ns7__setCurrentInstruction, _ns7__setCurrentInstructionResponse *ns7__setCurrentInstructionResponse);
        /// Web service operation 'replaceLine' (returns SOAP_OK or error code)
        virtual int replaceLine(_ns7__replaceLine *ns7__replaceLine, _ns7__replaceLineResponse *ns7__replaceLineResponse)
        { return this->replaceLine(NULL, NULL, ns7__replaceLine, ns7__replaceLineResponse); }
        virtual int replaceLine(const char *soap_endpoint, const char *soap_action, _ns7__replaceLine *ns7__replaceLine, _ns7__replaceLineResponse *ns7__replaceLineResponse);
        /// Web service operation 'readIos' (returns SOAP_OK or error code)
        virtual int readIos(_ns7__readIos *ns7__readIos, _ns7__readIosResponse *ns7__readIosResponse)
        { return this->readIos(NULL, NULL, ns7__readIos, ns7__readIosResponse); }
        virtual int readIos(const char *soap_endpoint, const char *soap_action, _ns7__readIos *ns7__readIos, _ns7__readIosResponse *ns7__readIosResponse);
        /// Web service operation 'writeIos' (returns SOAP_OK or error code)
        virtual int writeIos(_ns7__writeIos *ns7__writeIos, _ns7__writeIosResponse *ns7__writeIosResponse)
        { return this->writeIos(NULL, NULL, ns7__writeIos, ns7__writeIosResponse); }
        virtual int writeIos(const char *soap_endpoint, const char *soap_action, _ns7__writeIos *ns7__writeIos, _ns7__writeIosResponse *ns7__writeIosResponse);
        /// Web service operation 'lockIos' (returns SOAP_OK or error code)
        virtual int lockIos(_ns7__lockIos *ns7__lockIos, _ns7__lockIosResponse *ns7__lockIosResponse)
        { return this->lockIos(NULL, NULL, ns7__lockIos, ns7__lockIosResponse); }
        virtual int lockIos(const char *soap_endpoint, const char *soap_action, _ns7__lockIos *ns7__lockIos, _ns7__lockIosResponse *ns7__lockIosResponse);
        /// Web service operation 'unlockIos' (returns SOAP_OK or error code)
        virtual int unlockIos(_ns7__unlockIos *ns7__unlockIos, _ns7__unlockIosResponse *ns7__unlockIosResponse)
        { return this->unlockIos(NULL, NULL, ns7__unlockIos, ns7__unlockIosResponse); }
        virtual int unlockIos(const char *soap_endpoint, const char *soap_action, _ns7__unlockIos *ns7__unlockIos, _ns7__unlockIosResponse *ns7__unlockIosResponse);
        /// Web service operation 'lockAllIos' (returns SOAP_OK or error code)
        virtual int lockAllIos(_ns7__lockAllIos *ns7__lockAllIos, _ns7__SoapAllPhysicalIoResponse *ns7__SoapAllPhysicalIoResponse)
        { return this->lockAllIos(NULL, NULL, ns7__lockAllIos, ns7__SoapAllPhysicalIoResponse); }
        virtual int lockAllIos(const char *soap_endpoint, const char *soap_action, _ns7__lockAllIos *ns7__lockAllIos, _ns7__SoapAllPhysicalIoResponse *ns7__SoapAllPhysicalIoResponse);
        /// Web service operation 'unlockAllIos' (returns SOAP_OK or error code)
        virtual int unlockAllIos(_ns7__unlockAllIos *ns7__unlockAllIos, _ns7__SoapAllPhysicalIoResponse *ns7__SoapAllPhysicalIoResponse)
        { return this->unlockAllIos(NULL, NULL, ns7__unlockAllIos, ns7__SoapAllPhysicalIoResponse); }
        virtual int unlockAllIos(const char *soap_endpoint, const char *soap_action, _ns7__unlockAllIos *ns7__unlockAllIos, _ns7__SoapAllPhysicalIoResponse *ns7__SoapAllPhysicalIoResponse);
        /// Web service operation 'getAllPhysicalIos' (returns SOAP_OK or error code)
        virtual int getAllPhysicalIos(_ns7__getAllPhysicalIos *ns7__getAllPhysicalIos, _ns7__getAllPhysicalIosResponse *ns7__getAllPhysicalIosResponse)
        { return this->getAllPhysicalIos(NULL, NULL, ns7__getAllPhysicalIos, ns7__getAllPhysicalIosResponse); }
        virtual int getAllPhysicalIos(const char *soap_endpoint, const char *soap_action, _ns7__getAllPhysicalIos *ns7__getAllPhysicalIos, _ns7__getAllPhysicalIosResponse *ns7__getAllPhysicalIosResponse);
        /// Web service operation 'getRobotDhParameters' (returns SOAP_OK or error code)
        virtual int getRobotDhParameters(_ns7__getRobotDhParameters *ns7__getRobotDhParameters, _ns7__SoapRobotDhParameters *ns7__SoapRobotDhParameters)
        { return this->getRobotDhParameters(NULL, NULL, ns7__getRobotDhParameters, ns7__SoapRobotDhParameters); }
        virtual int getRobotDhParameters(const char *soap_endpoint, const char *soap_action, _ns7__getRobotDhParameters *ns7__getRobotDhParameters, _ns7__SoapRobotDhParameters *ns7__SoapRobotDhParameters);
        /// Web service operation 'getProject' (returns SOAP_OK or error code)
        virtual int getProject(_ns7__getProject *ns7__getProject, _ns7__getProjectResponse *ns7__getProjectResponse)
        { return this->getProject(NULL, NULL, ns7__getProject, ns7__getProjectResponse); }
        virtual int getProject(const char *soap_endpoint, const char *soap_action, _ns7__getProject *ns7__getProject, _ns7__getProjectResponse *ns7__getProjectResponse);
    };
#endif
