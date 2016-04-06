/* Kernel module to match a string into a packet.
 *
 * Copyright (C) 2000 Emmanuel Roger  <winfield@freegates.be>
 * Kernel module to match a http header string into a packet.
 *
 * Copyright (C) 2003, CyberTAN Corporation
 * All Rights Reserved.

 */ 
#ifndef _IP6T_WEBSTR_H
#define _IP6T_WEBSTR_H

#define BM6_MAX_NLEN 256
#define BM6_MAX_HLEN 1024

#define BLK6_JAVA		0x01
#define BLK6_ACTIVE		0x02
#define BLK6_COOKIE		0x04
#define BLK6_PROXY		0x08

typedef char *(*proc_ipt_search) (char *, char *, int, int);

struct ip6t_webstr_info {
    char string[BM6_MAX_NLEN];
    u_int16_t invert;
    u_int16_t len;
    u_int8_t type;
};

enum ip6t_webstr_type
{
    IP6T_WEBSTR_HOST,
    IP6T_WEBSTR_URL,
    IP6T_WEBSTR_CONTENT
};

#endif /* _IP6T_WEBSTR_H */
