/* soapCS8ServerV0Proxy.cpp
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

#include "../soapCS8ServerV0Proxy.h"

CS8ServerV0Proxy::CS8ServerV0Proxy()
{	this->soap = soap_new();
	this->soap_own = true;
	CS8ServerV0Proxy_init(SOAP_IO_DEFAULT, SOAP_IO_DEFAULT);
}

CS8ServerV0Proxy::CS8ServerV0Proxy(const CS8ServerV0Proxy& rhs)
{	this->soap = rhs.soap;
	this->soap_own = false;
	this->soap_endpoint = rhs.soap_endpoint;
}

CS8ServerV0Proxy::CS8ServerV0Proxy(struct soap *_soap)
{	this->soap = _soap;
	this->soap_own = false;
	CS8ServerV0Proxy_init(_soap->imode, _soap->omode);
}

CS8ServerV0Proxy::CS8ServerV0Proxy(const char *endpoint)
{	this->soap = soap_new();
	this->soap_own = true;
	CS8ServerV0Proxy_init(SOAP_IO_DEFAULT, SOAP_IO_DEFAULT);
	soap_endpoint = endpoint;
}

CS8ServerV0Proxy::CS8ServerV0Proxy(soap_mode iomode)
{	this->soap = soap_new();
	this->soap_own = true;
	CS8ServerV0Proxy_init(iomode, iomode);
}

CS8ServerV0Proxy::CS8ServerV0Proxy(const char *endpoint, soap_mode iomode)
{	this->soap = soap_new();
	this->soap_own = true;
	CS8ServerV0Proxy_init(iomode, iomode);
	soap_endpoint = endpoint;
}

CS8ServerV0Proxy::CS8ServerV0Proxy(soap_mode imode, soap_mode omode)
{	this->soap = soap_new();
	this->soap_own = true;
	CS8ServerV0Proxy_init(imode, omode);
}

CS8ServerV0Proxy::~CS8ServerV0Proxy()
{	if (this->soap_own)
		soap_free(this->soap);
}

void CS8ServerV0Proxy::CS8ServerV0Proxy_init(soap_mode imode, soap_mode omode)
{	soap_imode(this->soap, imode);
	soap_omode(this->soap, omode);
	soap_endpoint = NULL;
	static const struct Namespace namespaces[] = {
        {"SOAP-ENV", "http://www.w3.org/2003/05/soap-envelope", "http://schemas.xmlsoap.org/soap/envelope/", NULL},
        {"SOAP-ENC", "http://www.w3.org/2003/05/soap-encoding", "http://schemas.xmlsoap.org/soap/encoding/", NULL},
        {"xsi", "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*/XMLSchema-instance", NULL},
        {"xsd", "http://www.w3.org/2001/XMLSchema", "http://www.w3.org/*/XMLSchema", NULL},
        {"ns2", "urn:CS8ServerV1", NULL, NULL},
        {"ns5", "http://tempuri.org/xmlmime.xsd", NULL, NULL},
        {"ns3", "http://www.w3.org/2004/08/xop/include", NULL, NULL},
        {"ns4", "http://www.w3.org/2004/11/xmlmime", NULL, NULL},
        {"ns1", "CS8ServerV0", NULL, NULL},
        {"ns7", "urn:CS8ServerV2", NULL, NULL},
        {NULL, NULL, NULL, NULL}
    };
	soap_set_namespaces(this->soap, namespaces);
}

CS8ServerV0Proxy *CS8ServerV0Proxy::copy()
{	CS8ServerV0Proxy *dup = SOAP_NEW_COPY(CS8ServerV0Proxy);
	if (dup)
		soap_copy_context(dup->soap, this->soap);
	return dup;
}

CS8ServerV0Proxy& CS8ServerV0Proxy::operator=(const CS8ServerV0Proxy& rhs)
{	if (this->soap != rhs.soap)
	{	if (this->soap_own)
			soap_free(this->soap);
		this->soap = rhs.soap;
		this->soap_own = false;
		this->soap_endpoint = rhs.soap_endpoint;
	}
	return *this;
}

void CS8ServerV0Proxy::destroy()
{	soap_destroy(this->soap);
	soap_end(this->soap);
}

void CS8ServerV0Proxy::reset()
{	this->destroy();
	soap_done(this->soap);
	soap_initialize(this->soap);
	CS8ServerV0Proxy_init(SOAP_IO_DEFAULT, SOAP_IO_DEFAULT);
}

void CS8ServerV0Proxy::soap_noheader()
{	this->soap->header = NULL;
}

void CS8ServerV0Proxy::soap_header(int *ns1__sessionId)
{	::soap_header(this->soap);
	this->soap->header->ns1__sessionId = ns1__sessionId;
}

::SOAP_ENV__Header *CS8ServerV0Proxy::soap_header()
{	return this->soap->header;
}

::SOAP_ENV__Fault *CS8ServerV0Proxy::soap_fault()
{	return this->soap->fault;
}

const char *CS8ServerV0Proxy::soap_fault_string()
{	return *soap_faultstring(this->soap);
}

const char *CS8ServerV0Proxy::soap_fault_detail()
{	return *soap_faultdetail(this->soap);
}

int CS8ServerV0Proxy::soap_close_socket()
{	return soap_closesock(this->soap);
}

int CS8ServerV0Proxy::soap_force_close_socket()
{	return soap_force_closesock(this->soap);
}

void CS8ServerV0Proxy::soap_print_fault(FILE *fd)
{	::soap_print_fault(this->soap, fd);
}

#ifndef WITH_LEAN
#ifndef WITH_COMPAT
void CS8ServerV0Proxy::soap_stream_fault(std::ostream& os)
{	::soap_stream_fault(this->soap, os);
}
#endif

char *CS8ServerV0Proxy::soap_sprint_fault(char *buf, size_t len)
{	return ::soap_sprint_fault(this->soap, buf, len);
}
#endif

int CS8ServerV0Proxy::getSoapServerVersion(const char *endpoint, const char *soap_action, _ns1__getSoapServerVersion *ns1__getSoapServerVersion, _ns1__getSoapServerVersionResponse *ns1__getSoapServerVersionResponse)
{	struct soap *soap = this->soap;
	struct __ns1__getSoapServerVersion soap_tmp___ns1__getSoapServerVersion;
	if (endpoint)
		soap_endpoint = endpoint;
	if (soap_endpoint == NULL)
		soap_endpoint = "http://localhost:5653/";
	if (soap_action == NULL)
		soap_action = "";
	soap_tmp___ns1__getSoapServerVersion.ns1__getSoapServerVersion = ns1__getSoapServerVersion;
	soap_begin(soap);
	soap_set_version(soap, 2); /* SOAP1.2 */
	soap->encodingStyle = NULL;
	soap_serializeheader(soap);
	soap_serialize___ns1__getSoapServerVersion(soap, &soap_tmp___ns1__getSoapServerVersion);
	if (soap_begin_count(soap))
		return soap->error;
	if (soap->mode & SOAP_IO_LENGTH)
	{	if (soap_envelope_begin_out(soap)
		 || soap_putheader(soap)
		 || soap_body_begin_out(soap)
		 || soap_put___ns1__getSoapServerVersion(soap, &soap_tmp___ns1__getSoapServerVersion, "-ns1:getSoapServerVersion", "")
		 || soap_body_end_out(soap)
		 || soap_envelope_end_out(soap))
			 return soap->error;
	}
	if (soap_end_count(soap))
		return soap->error;
	if (soap_connect(soap, soap_endpoint, soap_action)
	 || soap_envelope_begin_out(soap)
	 || soap_putheader(soap)
	 || soap_body_begin_out(soap)
	 || soap_put___ns1__getSoapServerVersion(soap, &soap_tmp___ns1__getSoapServerVersion, "-ns1:getSoapServerVersion", "")
	 || soap_body_end_out(soap)
	 || soap_envelope_end_out(soap)
	 || soap_end_send(soap))
		return soap_closesock(soap);
	if (!ns1__getSoapServerVersionResponse)
		return soap_closesock(soap);
	ns1__getSoapServerVersionResponse->soap_default(soap);
	if (soap_begin_recv(soap)
	 || soap_envelope_begin_in(soap)
	 || soap_recv_header(soap)
	 || soap_body_begin_in(soap))
		return soap_closesock(soap);
	ns1__getSoapServerVersionResponse->soap_get(soap, "ns1:getSoapServerVersionResponse", NULL);
	if (soap->error)
		return soap_recv_fault(soap, 0);
	if (soap_body_end_in(soap)
	 || soap_envelope_end_in(soap)
	 || soap_end_recv(soap))
		return soap_closesock(soap);
	return soap_closesock(soap);
}

int CS8ServerV0Proxy::findServer(const char *endpoint, const char *soap_action, _ns1__findServer *ns1__findServer, _ns1__findServerResponse *ns1__findServerResponse)
{	struct soap *soap = this->soap;
	struct __ns1__findServer soap_tmp___ns1__findServer;
	if (endpoint)
		soap_endpoint = endpoint;
	if (soap_endpoint == NULL)
		soap_endpoint = "http://localhost:5653/";
	if (soap_action == NULL)
		soap_action = "";
	soap_tmp___ns1__findServer.ns1__findServer = ns1__findServer;
	soap_begin(soap);
	soap_set_version(soap, 2); /* SOAP1.2 */
	soap->encodingStyle = NULL;
	soap_serializeheader(soap);
	soap_serialize___ns1__findServer(soap, &soap_tmp___ns1__findServer);
	if (soap_begin_count(soap))
		return soap->error;
	if (soap->mode & SOAP_IO_LENGTH)
	{	if (soap_envelope_begin_out(soap)
		 || soap_putheader(soap)
		 || soap_body_begin_out(soap)
		 || soap_put___ns1__findServer(soap, &soap_tmp___ns1__findServer, "-ns1:findServer", "")
		 || soap_body_end_out(soap)
		 || soap_envelope_end_out(soap))
			 return soap->error;
	}
	if (soap_end_count(soap))
		return soap->error;
	if (soap_connect(soap, soap_endpoint, soap_action)
	 || soap_envelope_begin_out(soap)
	 || soap_putheader(soap)
	 || soap_body_begin_out(soap)
	 || soap_put___ns1__findServer(soap, &soap_tmp___ns1__findServer, "-ns1:findServer", "")
	 || soap_body_end_out(soap)
	 || soap_envelope_end_out(soap)
	 || soap_end_send(soap))
		return soap_closesock(soap);
	if (!ns1__findServerResponse)
		return soap_closesock(soap);
	ns1__findServerResponse->soap_default(soap);
	if (soap_begin_recv(soap)
	 || soap_envelope_begin_in(soap)
	 || soap_recv_header(soap)
	 || soap_body_begin_in(soap))
		return soap_closesock(soap);
	ns1__findServerResponse->soap_get(soap, "ns1:findServerResponse", NULL);
	if (soap->error)
		return soap_recv_fault(soap, 0);
	if (soap_body_end_in(soap)
	 || soap_envelope_end_in(soap)
	 || soap_end_recv(soap))
		return soap_closesock(soap);
	return soap_closesock(soap);
}

int CS8ServerV0Proxy::ping(const char *endpoint, const char *soap_action, _ns1__ping *ns1__ping, _ns1__pingResponse *ns1__pingResponse)
{	struct soap *soap = this->soap;
	struct __ns1__ping soap_tmp___ns1__ping;
	if (endpoint)
		soap_endpoint = endpoint;
	if (soap_endpoint == NULL)
		soap_endpoint = "http://localhost:5653/";
	if (soap_action == NULL)
		soap_action = "";
	soap_tmp___ns1__ping.ns1__ping = ns1__ping;
	soap_begin(soap);
	soap_set_version(soap, 2); /* SOAP1.2 */
	soap->encodingStyle = NULL;
	soap_serializeheader(soap);
	soap_serialize___ns1__ping(soap, &soap_tmp___ns1__ping);
	if (soap_begin_count(soap))
		return soap->error;
	if (soap->mode & SOAP_IO_LENGTH)
	{	if (soap_envelope_begin_out(soap)
		 || soap_putheader(soap)
		 || soap_body_begin_out(soap)
		 || soap_put___ns1__ping(soap, &soap_tmp___ns1__ping, "-ns1:ping", "")
		 || soap_body_end_out(soap)
		 || soap_envelope_end_out(soap))
			 return soap->error;
	}
	if (soap_end_count(soap))
		return soap->error;
	if (soap_connect(soap, soap_endpoint, soap_action)
	 || soap_envelope_begin_out(soap)
	 || soap_putheader(soap)
	 || soap_body_begin_out(soap)
	 || soap_put___ns1__ping(soap, &soap_tmp___ns1__ping, "-ns1:ping", "")
	 || soap_body_end_out(soap)
	 || soap_envelope_end_out(soap)
	 || soap_end_send(soap))
		return soap_closesock(soap);
	if (!ns1__pingResponse)
		return soap_closesock(soap);
	ns1__pingResponse->soap_default(soap);
	if (soap_begin_recv(soap)
	 || soap_envelope_begin_in(soap)
	 || soap_recv_header(soap)
	 || soap_body_begin_in(soap))
		return soap_closesock(soap);
	ns1__pingResponse->soap_get(soap, "ns1:pingResponse", NULL);
	if (soap->error)
		return soap_recv_fault(soap, 0);
	if (soap_body_end_in(soap)
	 || soap_envelope_end_in(soap)
	 || soap_end_recv(soap))
		return soap_closesock(soap);
	return soap_closesock(soap);
}

int CS8ServerV0Proxy::getCS8Versions(const char *endpoint, const char *soap_action, _ns1__getCS8Versions *ns1__getCS8Versions, _ns1__getCS8VersionsResponse *ns1__getCS8VersionsResponse)
{	struct soap *soap = this->soap;
	struct __ns1__getCS8Versions soap_tmp___ns1__getCS8Versions;
	if (endpoint)
		soap_endpoint = endpoint;
	if (soap_endpoint == NULL)
		soap_endpoint = "http://localhost:5653/";
	if (soap_action == NULL)
		soap_action = "";
	soap_tmp___ns1__getCS8Versions.ns1__getCS8Versions = ns1__getCS8Versions;
	soap_begin(soap);
	soap_set_version(soap, 2); /* SOAP1.2 */
	soap->encodingStyle = NULL;
	soap_serializeheader(soap);
	soap_serialize___ns1__getCS8Versions(soap, &soap_tmp___ns1__getCS8Versions);
	if (soap_begin_count(soap))
		return soap->error;
	if (soap->mode & SOAP_IO_LENGTH)
	{	if (soap_envelope_begin_out(soap)
		 || soap_putheader(soap)
		 || soap_body_begin_out(soap)
		 || soap_put___ns1__getCS8Versions(soap, &soap_tmp___ns1__getCS8Versions, "-ns1:getCS8Versions", "")
		 || soap_body_end_out(soap)
		 || soap_envelope_end_out(soap))
			 return soap->error;
	}
	if (soap_end_count(soap))
		return soap->error;
	if (soap_connect(soap, soap_endpoint, soap_action)
	 || soap_envelope_begin_out(soap)
	 || soap_putheader(soap)
	 || soap_body_begin_out(soap)
	 || soap_put___ns1__getCS8Versions(soap, &soap_tmp___ns1__getCS8Versions, "-ns1:getCS8Versions", "")
	 || soap_body_end_out(soap)
	 || soap_envelope_end_out(soap)
	 || soap_end_send(soap))
		return soap_closesock(soap);
	if (!ns1__getCS8VersionsResponse)
		return soap_closesock(soap);
	ns1__getCS8VersionsResponse->soap_default(soap);
	if (soap_begin_recv(soap)
	 || soap_envelope_begin_in(soap)
	 || soap_recv_header(soap)
	 || soap_body_begin_in(soap))
		return soap_closesock(soap);
	ns1__getCS8VersionsResponse->soap_get(soap, "ns1:getCS8VersionsResponse", NULL);
	if (soap->error)
		return soap_recv_fault(soap, 0);
	if (soap_body_end_in(soap)
	 || soap_envelope_end_in(soap)
	 || soap_end_recv(soap))
		return soap_closesock(soap);
	return soap_closesock(soap);
}

int CS8ServerV0Proxy::getControllerParameters(const char *endpoint, const char *soap_action, _ns1__getControllerParameters *ns1__getControllerParameters, _ns1__getControllerParametersResponse *ns1__getControllerParametersResponse)
{	struct soap *soap = this->soap;
	struct __ns1__getControllerParameters soap_tmp___ns1__getControllerParameters;
	if (endpoint)
		soap_endpoint = endpoint;
	if (soap_endpoint == NULL)
		soap_endpoint = "http://localhost:5653/";
	if (soap_action == NULL)
		soap_action = "";
	soap_tmp___ns1__getControllerParameters.ns1__getControllerParameters = ns1__getControllerParameters;
	soap_begin(soap);
	soap_set_version(soap, 2); /* SOAP1.2 */
	soap->encodingStyle = NULL;
	soap_serializeheader(soap);
	soap_serialize___ns1__getControllerParameters(soap, &soap_tmp___ns1__getControllerParameters);
	if (soap_begin_count(soap))
		return soap->error;
	if (soap->mode & SOAP_IO_LENGTH)
	{	if (soap_envelope_begin_out(soap)
		 || soap_putheader(soap)
		 || soap_body_begin_out(soap)
		 || soap_put___ns1__getControllerParameters(soap, &soap_tmp___ns1__getControllerParameters, "-ns1:getControllerParameters", "")
		 || soap_body_end_out(soap)
		 || soap_envelope_end_out(soap))
			 return soap->error;
	}
	if (soap_end_count(soap))
		return soap->error;
	if (soap_connect(soap, soap_endpoint, soap_action)
	 || soap_envelope_begin_out(soap)
	 || soap_putheader(soap)
	 || soap_body_begin_out(soap)
	 || soap_put___ns1__getControllerParameters(soap, &soap_tmp___ns1__getControllerParameters, "-ns1:getControllerParameters", "")
	 || soap_body_end_out(soap)
	 || soap_envelope_end_out(soap)
	 || soap_end_send(soap))
		return soap_closesock(soap);
	if (!ns1__getControllerParametersResponse)
		return soap_closesock(soap);
	ns1__getControllerParametersResponse->soap_default(soap);
	if (soap_begin_recv(soap)
	 || soap_envelope_begin_in(soap)
	 || soap_recv_header(soap)
	 || soap_body_begin_in(soap))
		return soap_closesock(soap);
	ns1__getControllerParametersResponse->soap_get(soap, "ns1:getControllerParametersResponse", NULL);
	if (soap->error)
		return soap_recv_fault(soap, 0);
	if (soap_body_end_in(soap)
	 || soap_envelope_end_in(soap)
	 || soap_end_recv(soap))
		return soap_closesock(soap);
	return soap_closesock(soap);
}

int CS8ServerV0Proxy::getCS8Compatibility(const char *endpoint, const char *soap_action, _ns1__getCS8Compatibility *ns1__getCS8Compatibility, _ns1__getCS8CompatibilityResponse *ns1__getCS8CompatibilityResponse)
{	struct soap *soap = this->soap;
	struct __ns1__getCS8Compatibility soap_tmp___ns1__getCS8Compatibility;
	if (endpoint)
		soap_endpoint = endpoint;
	if (soap_endpoint == NULL)
		soap_endpoint = "http://localhost:5653/";
	if (soap_action == NULL)
		soap_action = "";
	soap_tmp___ns1__getCS8Compatibility.ns1__getCS8Compatibility = ns1__getCS8Compatibility;
	soap_begin(soap);
	soap_set_version(soap, 2); /* SOAP1.2 */
	soap->encodingStyle = NULL;
	soap_serializeheader(soap);
	soap_serialize___ns1__getCS8Compatibility(soap, &soap_tmp___ns1__getCS8Compatibility);
	if (soap_begin_count(soap))
		return soap->error;
	if (soap->mode & SOAP_IO_LENGTH)
	{	if (soap_envelope_begin_out(soap)
		 || soap_putheader(soap)
		 || soap_body_begin_out(soap)
		 || soap_put___ns1__getCS8Compatibility(soap, &soap_tmp___ns1__getCS8Compatibility, "-ns1:getCS8Compatibility", "")
		 || soap_body_end_out(soap)
		 || soap_envelope_end_out(soap))
			 return soap->error;
	}
	if (soap_end_count(soap))
		return soap->error;
	if (soap_connect(soap, soap_endpoint, soap_action)
	 || soap_envelope_begin_out(soap)
	 || soap_putheader(soap)
	 || soap_body_begin_out(soap)
	 || soap_put___ns1__getCS8Compatibility(soap, &soap_tmp___ns1__getCS8Compatibility, "-ns1:getCS8Compatibility", "")
	 || soap_body_end_out(soap)
	 || soap_envelope_end_out(soap)
	 || soap_end_send(soap))
		return soap_closesock(soap);
	if (!ns1__getCS8CompatibilityResponse)
		return soap_closesock(soap);
	ns1__getCS8CompatibilityResponse->soap_default(soap);
	if (soap_begin_recv(soap)
	 || soap_envelope_begin_in(soap)
	 || soap_recv_header(soap)
	 || soap_body_begin_in(soap))
		return soap_closesock(soap);
	ns1__getCS8CompatibilityResponse->soap_get(soap, "ns1:getCS8CompatibilityResponse", NULL);
	if (soap->error)
		return soap_recv_fault(soap, 0);
	if (soap_body_end_in(soap)
	 || soap_envelope_end_in(soap)
	 || soap_end_recv(soap))
		return soap_closesock(soap);
	return soap_closesock(soap);
}

int CS8ServerV0Proxy::login(const char *endpoint, const char *soap_action, _ns1__login *ns1__login, _ns1__loginResponse *ns1__loginResponse)
{	struct soap *soap = this->soap;
	struct __ns1__login soap_tmp___ns1__login;
	if (endpoint)
		soap_endpoint = endpoint;
	if (soap_endpoint == NULL)
		soap_endpoint = "http://localhost:5653/";
	if (soap_action == NULL)
		soap_action = "";
	soap_tmp___ns1__login.ns1__login = ns1__login;
	soap_begin(soap);
	soap_set_version(soap, 2); /* SOAP1.2 */
	soap->encodingStyle = NULL;
	soap_serializeheader(soap);
	soap_serialize___ns1__login(soap, &soap_tmp___ns1__login);
	if (soap_begin_count(soap))
		return soap->error;
	if (soap->mode & SOAP_IO_LENGTH)
	{	if (soap_envelope_begin_out(soap)
		 || soap_putheader(soap)
		 || soap_body_begin_out(soap)
		 || soap_put___ns1__login(soap, &soap_tmp___ns1__login, "-ns1:login", "")
		 || soap_body_end_out(soap)
		 || soap_envelope_end_out(soap))
			 return soap->error;
	}
	if (soap_end_count(soap))
		return soap->error;
	if (soap_connect(soap, soap_endpoint, soap_action)
	 || soap_envelope_begin_out(soap)
	 || soap_putheader(soap)
	 || soap_body_begin_out(soap)
	 || soap_put___ns1__login(soap, &soap_tmp___ns1__login, "-ns1:login", "")
	 || soap_body_end_out(soap)
	 || soap_envelope_end_out(soap)
	 || soap_end_send(soap))
		return soap_closesock(soap);
	if (!ns1__loginResponse)
		return soap_closesock(soap);
	ns1__loginResponse->soap_default(soap);
	if (soap_begin_recv(soap)
	 || soap_envelope_begin_in(soap)
	 || soap_recv_header(soap)
	 || soap_body_begin_in(soap))
		return soap_closesock(soap);
	ns1__loginResponse->soap_get(soap, "ns1:loginResponse", NULL);
	if (soap->error)
		return soap_recv_fault(soap, 0);
	if (soap_body_end_in(soap)
	 || soap_envelope_end_in(soap)
	 || soap_end_recv(soap))
		return soap_closesock(soap);
	return soap_closesock(soap);
}

int CS8ServerV0Proxy::logout(const char *endpoint, const char *soap_action, _ns1__logout *ns1__logout, _ns1__logoutResponse *ns1__logoutResponse)
{	struct soap *soap = this->soap;
	struct __ns1__logout soap_tmp___ns1__logout;
	if (endpoint)
		soap_endpoint = endpoint;
	if (soap_endpoint == NULL)
		soap_endpoint = "http://localhost:5653/";
	if (soap_action == NULL)
		soap_action = "";
	soap_tmp___ns1__logout.ns1__logout = ns1__logout;
	soap_begin(soap);
	soap_set_version(soap, 2); /* SOAP1.2 */
	soap->encodingStyle = NULL;
	soap_serializeheader(soap);
	soap_serialize___ns1__logout(soap, &soap_tmp___ns1__logout);
	if (soap_begin_count(soap))
		return soap->error;
	if (soap->mode & SOAP_IO_LENGTH)
	{	if (soap_envelope_begin_out(soap)
		 || soap_putheader(soap)
		 || soap_body_begin_out(soap)
		 || soap_put___ns1__logout(soap, &soap_tmp___ns1__logout, "-ns1:logout", "")
		 || soap_body_end_out(soap)
		 || soap_envelope_end_out(soap))
			 return soap->error;
	}
	if (soap_end_count(soap))
		return soap->error;
	if (soap_connect(soap, soap_endpoint, soap_action)
	 || soap_envelope_begin_out(soap)
	 || soap_putheader(soap)
	 || soap_body_begin_out(soap)
	 || soap_put___ns1__logout(soap, &soap_tmp___ns1__logout, "-ns1:logout", "")
	 || soap_body_end_out(soap)
	 || soap_envelope_end_out(soap)
	 || soap_end_send(soap))
		return soap_closesock(soap);
	if (!ns1__logoutResponse)
		return soap_closesock(soap);
	ns1__logoutResponse->soap_default(soap);
	if (soap_begin_recv(soap)
	 || soap_envelope_begin_in(soap)
	 || soap_recv_header(soap)
	 || soap_body_begin_in(soap))
		return soap_closesock(soap);
	ns1__logoutResponse->soap_get(soap, "ns1:logoutResponse", NULL);
	if (soap->error)
		return soap_recv_fault(soap, 0);
	if (soap_body_end_in(soap)
	 || soap_envelope_end_in(soap)
	 || soap_end_recv(soap))
		return soap_closesock(soap);
	return soap_closesock(soap);
}

int CS8ServerV0Proxy::getRobots(const char *endpoint, const char *soap_action, _ns1__getRobots *ns1__getRobots, _ns1__getRobotsResponse *ns1__getRobotsResponse)
{	struct soap *soap = this->soap;
	struct __ns1__getRobots soap_tmp___ns1__getRobots;
	if (endpoint)
		soap_endpoint = endpoint;
	if (soap_endpoint == NULL)
		soap_endpoint = "http://localhost:5653/";
	if (soap_action == NULL)
		soap_action = "";
	soap_tmp___ns1__getRobots.ns1__getRobots = ns1__getRobots;
	soap_begin(soap);
	soap_set_version(soap, 2); /* SOAP1.2 */
	soap->encodingStyle = NULL;
	soap_serializeheader(soap);
	soap_serialize___ns1__getRobots(soap, &soap_tmp___ns1__getRobots);
	if (soap_begin_count(soap))
		return soap->error;
	if (soap->mode & SOAP_IO_LENGTH)
	{	if (soap_envelope_begin_out(soap)
		 || soap_putheader(soap)
		 || soap_body_begin_out(soap)
		 || soap_put___ns1__getRobots(soap, &soap_tmp___ns1__getRobots, "-ns1:getRobots", "")
		 || soap_body_end_out(soap)
		 || soap_envelope_end_out(soap))
			 return soap->error;
	}
	if (soap_end_count(soap))
		return soap->error;
	if (soap_connect(soap, soap_endpoint, soap_action)
	 || soap_envelope_begin_out(soap)
	 || soap_putheader(soap)
	 || soap_body_begin_out(soap)
	 || soap_put___ns1__getRobots(soap, &soap_tmp___ns1__getRobots, "-ns1:getRobots", "")
	 || soap_body_end_out(soap)
	 || soap_envelope_end_out(soap)
	 || soap_end_send(soap))
		return soap_closesock(soap);
	if (!ns1__getRobotsResponse)
		return soap_closesock(soap);
	ns1__getRobotsResponse->soap_default(soap);
	if (soap_begin_recv(soap)
	 || soap_envelope_begin_in(soap)
	 || soap_recv_header(soap)
	 || soap_body_begin_in(soap))
		return soap_closesock(soap);
	ns1__getRobotsResponse->soap_get(soap, "ns1:getRobotsResponse", NULL);
	if (soap->error)
		return soap_recv_fault(soap, 0);
	if (soap_body_end_in(soap)
	 || soap_envelope_end_in(soap)
	 || soap_end_recv(soap))
		return soap_closesock(soap);
	return soap_closesock(soap);
}

int CS8ServerV0Proxy::getRobotJointPos(const char *endpoint, const char *soap_action, _ns1__getRobotJointPos *ns1__getRobotJointPos, _ns1__getRobotJointPosResponse *ns1__getRobotJointPosResponse)
{	struct soap *soap = this->soap;
	struct __ns1__getRobotJointPos soap_tmp___ns1__getRobotJointPos;
	if (endpoint)
		soap_endpoint = endpoint;
	if (soap_endpoint == NULL)
		soap_endpoint = "http://localhost:5653/";
	if (soap_action == NULL)
		soap_action = "";
	soap_tmp___ns1__getRobotJointPos.ns1__getRobotJointPos = ns1__getRobotJointPos;
	soap_begin(soap);
	soap_set_version(soap, 2); /* SOAP1.2 */
	soap->encodingStyle = NULL;
	soap_serializeheader(soap);
	soap_serialize___ns1__getRobotJointPos(soap, &soap_tmp___ns1__getRobotJointPos);
	if (soap_begin_count(soap))
		return soap->error;
	if (soap->mode & SOAP_IO_LENGTH)
	{	if (soap_envelope_begin_out(soap)
		 || soap_putheader(soap)
		 || soap_body_begin_out(soap)
		 || soap_put___ns1__getRobotJointPos(soap, &soap_tmp___ns1__getRobotJointPos, "-ns1:getRobotJointPos", "")
		 || soap_body_end_out(soap)
		 || soap_envelope_end_out(soap))
			 return soap->error;
	}
	if (soap_end_count(soap))
		return soap->error;
	if (soap_connect(soap, soap_endpoint, soap_action)
	 || soap_envelope_begin_out(soap)
	 || soap_putheader(soap)
	 || soap_body_begin_out(soap)
	 || soap_put___ns1__getRobotJointPos(soap, &soap_tmp___ns1__getRobotJointPos, "-ns1:getRobotJointPos", "")
	 || soap_body_end_out(soap)
	 || soap_envelope_end_out(soap)
	 || soap_end_send(soap))
		return soap_closesock(soap);
	if (!ns1__getRobotJointPosResponse)
		return soap_closesock(soap);
	ns1__getRobotJointPosResponse->soap_default(soap);
	if (soap_begin_recv(soap)
	 || soap_envelope_begin_in(soap)
	 || soap_recv_header(soap)
	 || soap_body_begin_in(soap))
		return soap_closesock(soap);
	ns1__getRobotJointPosResponse->soap_get(soap, "ns1:getRobotJointPosResponse", NULL);
	if (soap->error)
		return soap_recv_fault(soap, 0);
	if (soap_body_end_in(soap)
	 || soap_envelope_end_in(soap)
	 || soap_end_recv(soap))
		return soap_closesock(soap);
	return soap_closesock(soap);
}

int CS8ServerV0Proxy::getRobotJntCartPos(const char *endpoint, const char *soap_action, _ns1__getRobotJntCartPos *ns1__getRobotJntCartPos, _ns1__getRobotJntCartPosResponse *ns1__getRobotJntCartPosResponse)
{	struct soap *soap = this->soap;
	struct __ns1__getRobotJntCartPos soap_tmp___ns1__getRobotJntCartPos;
	if (endpoint)
		soap_endpoint = endpoint;
	if (soap_endpoint == NULL)
		soap_endpoint = "http://localhost:5653/";
	if (soap_action == NULL)
		soap_action = "";
	soap_tmp___ns1__getRobotJntCartPos.ns1__getRobotJntCartPos = ns1__getRobotJntCartPos;
	soap_begin(soap);
	soap_set_version(soap, 2); /* SOAP1.2 */
	soap->encodingStyle = NULL;
	soap_serializeheader(soap);
	soap_serialize___ns1__getRobotJntCartPos(soap, &soap_tmp___ns1__getRobotJntCartPos);
	if (soap_begin_count(soap))
		return soap->error;
	if (soap->mode & SOAP_IO_LENGTH)
	{	if (soap_envelope_begin_out(soap)
		 || soap_putheader(soap)
		 || soap_body_begin_out(soap)
		 || soap_put___ns1__getRobotJntCartPos(soap, &soap_tmp___ns1__getRobotJntCartPos, "-ns1:getRobotJntCartPos", "")
		 || soap_body_end_out(soap)
		 || soap_envelope_end_out(soap))
			 return soap->error;
	}
	if (soap_end_count(soap))
		return soap->error;
	if (soap_connect(soap, soap_endpoint, soap_action)
	 || soap_envelope_begin_out(soap)
	 || soap_putheader(soap)
	 || soap_body_begin_out(soap)
	 || soap_put___ns1__getRobotJntCartPos(soap, &soap_tmp___ns1__getRobotJntCartPos, "-ns1:getRobotJntCartPos", "")
	 || soap_body_end_out(soap)
	 || soap_envelope_end_out(soap)
	 || soap_end_send(soap))
		return soap_closesock(soap);
	if (!ns1__getRobotJntCartPosResponse)
		return soap_closesock(soap);
	ns1__getRobotJntCartPosResponse->soap_default(soap);
	if (soap_begin_recv(soap)
	 || soap_envelope_begin_in(soap)
	 || soap_recv_header(soap)
	 || soap_body_begin_in(soap))
		return soap_closesock(soap);
	ns1__getRobotJntCartPosResponse->soap_get(soap, "ns1:getRobotJntCartPosResponse", NULL);
	if (soap->error)
		return soap_recv_fault(soap, 0);
	if (soap_body_end_in(soap)
	 || soap_envelope_end_in(soap)
	 || soap_end_recv(soap))
		return soap_closesock(soap);
	return soap_closesock(soap);
}

int CS8ServerV0Proxy::setRobotJointPos(const char *endpoint, const char *soap_action, _ns1__setRobotJointPos *ns1__setRobotJointPos, _ns1__setRobotPosResponse *ns1__setRobotPosResponse)
{	struct soap *soap = this->soap;
	struct __ns1__setRobotJointPos soap_tmp___ns1__setRobotJointPos;
	if (endpoint)
		soap_endpoint = endpoint;
	if (soap_endpoint == NULL)
		soap_endpoint = "http://localhost:5653/";
	if (soap_action == NULL)
		soap_action = "";
	soap_tmp___ns1__setRobotJointPos.ns1__setRobotJointPos = ns1__setRobotJointPos;
	soap_begin(soap);
	soap_set_version(soap, 2); /* SOAP1.2 */
	soap->encodingStyle = NULL;
	soap_serializeheader(soap);
	soap_serialize___ns1__setRobotJointPos(soap, &soap_tmp___ns1__setRobotJointPos);
	if (soap_begin_count(soap))
		return soap->error;
	if (soap->mode & SOAP_IO_LENGTH)
	{	if (soap_envelope_begin_out(soap)
		 || soap_putheader(soap)
		 || soap_body_begin_out(soap)
		 || soap_put___ns1__setRobotJointPos(soap, &soap_tmp___ns1__setRobotJointPos, "-ns1:setRobotJointPos", "")
		 || soap_body_end_out(soap)
		 || soap_envelope_end_out(soap))
			 return soap->error;
	}
	if (soap_end_count(soap))
		return soap->error;
	if (soap_connect(soap, soap_endpoint, soap_action)
	 || soap_envelope_begin_out(soap)
	 || soap_putheader(soap)
	 || soap_body_begin_out(soap)
	 || soap_put___ns1__setRobotJointPos(soap, &soap_tmp___ns1__setRobotJointPos, "-ns1:setRobotJointPos", "")
	 || soap_body_end_out(soap)
	 || soap_envelope_end_out(soap)
	 || soap_end_send(soap))
		return soap_closesock(soap);
	if (!ns1__setRobotPosResponse)
		return soap_closesock(soap);
	ns1__setRobotPosResponse->soap_default(soap);
	if (soap_begin_recv(soap)
	 || soap_envelope_begin_in(soap)
	 || soap_recv_header(soap)
	 || soap_body_begin_in(soap))
		return soap_closesock(soap);
	ns1__setRobotPosResponse->soap_get(soap, "ns1:setRobotPosResponse", NULL);
	if (soap->error)
		return soap_recv_fault(soap, 0);
	if (soap_body_end_in(soap)
	 || soap_envelope_end_in(soap)
	 || soap_end_recv(soap))
		return soap_closesock(soap);
	return soap_closesock(soap);
}
/* End of client proxy code */
