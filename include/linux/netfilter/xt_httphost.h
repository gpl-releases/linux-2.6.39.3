#ifndef _XT_HTTPHOST_H
#define _XT_HTTPHOST_H

#define XT_HTTPHOST_MAX_HOST_SIZE (128)

struct xt_httphost_info
{
	char 	  host[XT_HTTPHOST_MAX_HOST_SIZE];
	u_int8_t  invert;
};

#endif /*_XT_HTTPHOST_H */
