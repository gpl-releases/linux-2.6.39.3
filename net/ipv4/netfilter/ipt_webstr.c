/* Kernel module to match a string into a packet.
 *
 * Copyright (C) 2000 Emmanuel Roger  <winfield@freegates.be>
 * 
 * ChangeLog
 *	19.02.2002: Gianni Tedesco <gianni@ecsc.co.uk>
 *		Fixed SMP re-entrancy problem using per-cpu data areas
 *		for the skip/shift tables.
 *	02.05.2001: Gianni Tedesco <gianni@ecsc.co.uk>
 *		Fixed kernel panic, due to overrunning boyer moore string
 *		tables. Also slightly tweaked heuristic for deciding what
 * 		search algo to use.
 * 	27.01.2001: Gianni Tedesco <gianni@ecsc.co.uk>
 * 		Implemented Boyer Moore Sublinear search algorithm
 * 		alongside the existing linear search based on memcmp().
 * 		Also a quick check to decide which method to use on a per
 * 		packet basis.
 * Download location
 *               http://svn.dd-wrt.com:8000/browser//src/linux/xscale/linux-2.6.34.6/net/ipv4/netfilter
 */

/* Kernel module to match a http header string into a packet.
 *
 * Copyright (C) 2003, CyberTAN Corporation
 * All Rights Reserved.
 *
 * Description:
 *   This is kernel module for web content inspection. It was derived from 
 *   'string' match module, declared as above.
 *
 *   The module follows the Netfilter framework, called extended packet 
 *   matching modules. 
 * Restriction:
 *            Can not search https headers, higly recomended to use  tinyproxy server to block contents
 ******************************************************************************************************
 Includes Intel Corporation's changes/modifications dated: 02.11.2011
 Changed/modified portions :   
 * -  X-tables API adapted for kernel 2.6.39
 * - added, improved Debug prints
 * - Kernel warning /complains of a too big stack size in the ipt_webstr_match() function fixed
 * - copy from packet sk_buff removed, search is done directly in the tcp payload (optimization)
 * Copyright © 2011, Intel Corporation.
 ******************************************************************************************************

 */



#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <linux/tcp.h>

#include <linux/netfilter/x_tables.h>
#include <linux/netfilter_ipv4/ip_tables.h>
#include <linux/netfilter_ipv4/ipt_webstr.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gianni Tedesco <gianni@ecsc.co.uk>");
//MODULE_DESCRIPTION("Xtables: match --url or --host parameter in http header IPv4");
#define	isdigit(x) ((x) >= '0' && (x) <= '9')
#define	isupper(x) (((unsigned)(x) >= 'A') && ((unsigned)(x) <= 'Z'))
#define	islower(x) (((unsigned)(x) >= 'a') && ((unsigned)(x) <= 'z'))
#define	isalpha(x) (isupper(x) || islower(x))
#define	toupper(x) (isupper(x) ? (x) : (x) - 'a' + 'A')
#define tolower(x) (isupper(x) ? ((x) - 'A' + 'a') : (x))

#define split(word, wordlist, next, delim) \
    for (next = wordlist, \
	strncpy(word, next, sizeof(word)), \
	word[(next=strstr(next, delim)) ? strstr(word, delim) - word : sizeof(word) - 1] = '\0', \
	next = next ? next + sizeof(delim) - 1 : NULL ; \
	strlen(word); \
	next = next ? : "", \
	strncpy(word, next, sizeof(word)), \
	word[(next=strstr(next, delim)) ? strstr(word, delim) - word : sizeof(word) - 1] = '\0', \
	next = next ? next + sizeof(delim) - 1 : NULL)

#define URLSIZE 	1024
#define WORDSIZE 	256
#define HOSTSIZE 	256


/* Flags for get_http_info() */
#define HTTP_HOST	0x01
#define HTTP_URL	0x02
/* Flags for mangle_http_header() */
#define HTTP_COOKIE	0x04

//#define DEBUG
#ifdef DEBUG
#define DEBUGP printk
#define PRINTK_LEVEL KERN_CRIT
#else
#define PRINTK_LEVEL
#define DEBUGP( PRINTK_LEVEL, format, args...)
#endif

typedef struct httpinfo {
    char  host[HOSTSIZE];
    int hostlen;
    char  url[URLSIZE];
    int urllen;
} httpinfo_t;
static httpinfo_t htinfo;

/* Return 1 for match, 0 for accept, -1 for partial. */
static int find_pattern2(const char *data, size_t dlen,
	const char *pattern, size_t plen,
	char term,
	unsigned int *numoff,
	unsigned int *numlen)
{
    size_t i, j, k;
    int state = 0;
    *numoff = *numlen = 0;

    DEBUGP( PRINTK_LEVEL "%s: pattern = '%s', dlen = %u\n",__FUNCTION__, pattern, dlen);
    if (dlen == 0)
	return 0;

    if (dlen <= plen) {	/* Short packet: try for partial? */
	if (strnicmp(data, pattern, dlen) == 0)
	    return -1;
	else 
	    return 0;
    }
    for (i = 0; i <= (dlen - plen); i++) {
	/* DFA : \r\n\r\n :: 1234 */
	if (*(data + i) == '\r') {
	    if (!(state % 2)) state++;	/* forwarding move */
	    else state = 0;		/* reset */
	}
	else if (*(data + i) == '\n') {
	    if (state % 2) state++;
	    else state = 0;
	}
	else state = 0;

	if (state >= 4)
	    break;

	/* pattern compare */
	if (memcmp(data + i, pattern, plen ) != 0)
	    continue;

	/* Here, it means patten match!! */
	*numoff=i + plen;
	for (j = *numoff, k = 0; data[j] != term; j++, k++)
	    if (j > dlen) return -1 ;	/* no terminal char */

	*numlen = k;
	return 1;
    }
    return 0;
}

static int mangle_http_header(const struct sk_buff *skb, int flags)
{
    struct iphdr *iph = ip_hdr(skb);
    struct tcphdr *tcph = (void *)iph + iph->ihl*4;
    unsigned char *data = (void *)tcph + tcph->doff*4;
    unsigned int datalen = (skb)->len - (iph->ihl*4) - (tcph->doff*4);

    int found, offset, len;
    int ret = 0;


    DEBUGP( PRINTK_LEVEL "%s: seq=%u\n", __FUNCTION__, ntohl(tcph->seq));

    /* Basic checking, is it HTTP packet? */
    if (datalen < 10)
	return ret;	/* Not enough length, ignore it */
    if (memcmp(data, "GET ", sizeof("GET ") - 1) != 0 &&
        memcmp(data, "POST ", sizeof("POST ") - 1) != 0)
	return ret;	/* Pass it */

    /* COOKIE modification */
    if (flags & HTTP_COOKIE) {
	found = find_pattern2(data, datalen, "Cookie: ", 
		sizeof("Cookie: ")-1, '\r', &offset, &len);
	if (found) {
	    char c;
	    offset -= (sizeof("Cookie: ") - 1);
	    /* Swap the 2rd and 4th bit */
	    c = *(data + offset + 2) ;
	    *(data + offset + 2) = *(data + offset + 4) ;
	    *(data + offset + 4) = c ;
	    ret++;
	}
    }

    return ret;
}

static int get_http_info(const struct sk_buff *skb, int flags, httpinfo_t *info)
{
    struct iphdr *iph = ip_hdr(skb);
    struct tcphdr *tcph = (void *)iph + iph->ihl*4;
    unsigned char *data = (void *)tcph + tcph->doff*4;
    unsigned int datalen = (skb)->len - (iph->ihl*4) - (tcph->doff*4);

    int found, offset;
    int hostlen, pathlen;
    int ret = 0;


    DEBUGP( PRINTK_LEVEL "%s:, skb->len=%d, datalen=%d seq=%u\n", __FUNCTION__,skb->len, datalen, ntohl(tcph->seq));

    /* Basic checking, is it HTTP packet? */
    if (datalen < 10)
    {
        #ifdef DEBUG
            DEBUGP( PRINTK_LEVEL "%s: Error ?SYN? datalen <10 datalen=%d\n", __FUNCTION__ ,datalen);
        #endif
	return ret;	/* Not enough length, ignore it */
    }
    if (memcmp(data, "GET ", sizeof("GET ") - 1) != 0 &&
        memcmp(data, "POST ", sizeof("POST ") - 1) != 0 &&
        memcmp(data, "HEAD ", sizeof("HEAD ") - 1) != 0)
    {
        #ifdef DEBUG
            DEBUGP( PRINTK_LEVEL "%s: Error no GET/POST clause\n", __FUNCTION__ );
        #endif
	    return ret;	/* Pass it */
    }
    if (!(flags & (HTTP_HOST | HTTP_URL)))
    {
        #ifdef DEBUG
            DEBUGP( PRINTK_LEVEL "%s: Error nor HOST nor URL match\n", __FUNCTION__ );
        #endif
	    return ret;
    }

    if ( flags & HTTP_HOST )  /* it is always true because MATCH parameter parser adds HTTP__HOST when HTTP_URL is specified */ 
    {
        /* find the 'Host: ' value */
         found = find_pattern2(data, datalen, "Host: ", 
        	    sizeof("Host: ") - 1, '\r', &offset, &hostlen);
        #ifdef DEBUG
        if (found )
            DEBUGP( PRINTK_LEVEL "Host found at offset=%d, with hostlen=%d\n", offset, hostlen);
        #endif

        if (!found || !hostlen)
        	return ret;

        ret++;	/* Host found, increase the return value */
		hostlen = (hostlen < HOSTSIZE) ? hostlen : HOSTSIZE;
        strncpy(info->host, data + offset, hostlen);
		 *(info->host + hostlen) = 0;		/* null-terminated */
        info->hostlen = hostlen;
        DEBUGP( PRINTK_LEVEL "HOST=%s, hostlen=%d\n", info->host, info->hostlen);
    }

    if (!(flags & HTTP_URL))
	return ret;

    /* find the ['GET ' | 'POST ' | 'HEAD'] parameter value, i.e find url */
    found = find_pattern2(data, datalen, "GET ",
	    sizeof("GET ") - 1, '\r', &offset, &pathlen);
    DEBUGP( PRINTK_LEVEL "HTTP GET found=%d\n", found);
    if (!found)
	found = find_pattern2(data, datalen, "POST ",
		sizeof("POST ") - 1, '\r', &offset, &pathlen);
    if (!found) 
        found = find_pattern2(data, datalen, "HEAD ", 
                sizeof("HEAD ") - 1, '\r', &offset, &pathlen);      
    DEBUGP( PRINTK_LEVEL "HTTP POST found=%d\n", found);

    if (!found || (pathlen -= (sizeof(" HTTP/x.x") - 1)) <= 0) /* ignore this field */
	   return ret;
    DEBUGP( PRINTK_LEVEL " pathlen - (sizeof(' HTTP/x.x') %d\n", pathlen - (sizeof(" HTTP/x.x") - 1));

    ret++;	/* GET/POST found, increase the return value */
    pathlen = ((pathlen + hostlen) < URLSIZE) ? pathlen : URLSIZE - hostlen;
	DEBUGP( PRINTK_LEVEL " Make url start from host hostlen %d\n", hostlen);
	strncpy(info->url, info->host, hostlen);
	
	
    strncpy(info->url + hostlen, data + offset, pathlen);
    *(info->url + hostlen + pathlen) = 0;	/* null-terminated */
	/* in a regular http reguest, host is after url */
    DEBUGP( PRINTK_LEVEL " Append Url  to host sits at address of %p, offset=%d from tcp payload to \n", data + offset, offset);
    info->urllen = hostlen + pathlen;
    DEBUGP( PRINTK_LEVEL "URL=%s, urllen=%d\n", info->url, info->urllen);

    return ret;
}

/* Linear string search based on memcmp() */
static char *search_linear (char *needle, char *haystack, int needle_len, int haystack_len) 
{
	char *k = haystack + (haystack_len-needle_len);
	char *t = haystack;
	
	DEBUGP( PRINTK_LEVEL "%s: haystack=%s, needle=%s\n", __FUNCTION__, t, needle);
	for(; t <= k; t++) {
		DEBUGP( PRINTK_LEVEL "%s: haystack=%s, needle=%s\n", __FUNCTION__, t, needle);
		if (strnicmp(t, needle, needle_len) == 0) return t;
		//if ( memcmp(t, needle, needle_len) == 0 ) return t;
	}

	return NULL;
}

#define TOKEN   "<&nbsp;>" 

static bool ipt_webstr_match(const struct sk_buff *skb,  struct xt_action_param *par)
{
	const struct ipt_webstr_info *info = par->matchinfo;
	struct iphdr *ip = ip_hdr(skb);

	char *wordlist = (char *)&info->string;
	int flags = 0;
	int found = 0;
	long int opt = 0;


	if (!ip || info->len < 1)
	    return 0;

	DEBUGP( PRINTK_LEVEL "\n************************************************\n"
		"%s: type=%s\n", __FUNCTION__, (info->type == IPT_WEBSTR_URL) 
		? "IPT_WEBSTR_URL"  : (info->type == IPT_WEBSTR_HOST) 
		? "IPT_WEBSTR_HOST" : "IPT_WEBSTR_CONTENT" );
	
	/* Determine the flags value for get_http_info(), and mangle packet 
	 * if needed. */
	switch(info->type)
	{
	    case IPT_WEBSTR_URL:
		flags |= HTTP_URL;
       /* add both flags because we construct url from host name first, then url
       adds HTTP__HOST when HTTP_URL is specified */
	    case IPT_WEBSTR_HOST:
		flags |= HTTP_HOST;
		break;

	    case IPT_WEBSTR_CONTENT:
		opt = simple_strtol(wordlist, (char **)NULL, 10);
		DEBUGP( PRINTK_LEVEL "%s: string=%s, opt=%#lx\n", __FUNCTION__, wordlist, opt);

		if (opt & (BLK_JAVA | BLK_ACTIVE | BLK_PROXY))
		    flags |= HTTP_URL;
		if (opt & BLK_PROXY)
		    flags |= HTTP_HOST;
		if (opt & BLK_COOKIE)
		    mangle_http_header(skb, HTTP_COOKIE);
		break;

	    default:
		printk( KERN_ERR "%s: Sorry! Cannot find  match type.\n", __FILE__);
		return 0;
	}

	/* Get the http header info */
	if (get_http_info(skb, flags, &htinfo) < 1)
	    return 0;

	/* Check if the http header content contains the forbidden keyword */
	if (info->type == IPT_WEBSTR_HOST || info->type == IPT_WEBSTR_URL) {
	    int nlen = 0, hlen = 0;
	    char needle[WORDSIZE], *haystack = NULL;
	    char *next;

	    if (info->type == IPT_WEBSTR_HOST) {
		haystack = htinfo.host;
		hlen = htinfo.hostlen;
	    }
	    else {
		haystack = htinfo.url;
		hlen = htinfo.urllen;
	    }
	    split(needle, wordlist, next, TOKEN) {
		nlen = strlen(needle);
		DEBUGP( PRINTK_LEVEL "keyword=%s, nlen=%d, hlen=%d\n", needle, nlen, hlen);
		if (!nlen || !hlen || nlen > hlen) continue;
		if (search_linear(needle, haystack, nlen, hlen) != NULL) {
		    found = 1;
		    break;
		}
	    }
	}
	else {		/* IPT_WEBSTR_CONTENT */
	    int vicelen;

	    if (opt & BLK_JAVA) {
		vicelen = sizeof(".js") - 1;
		if (strnicmp(htinfo.url + htinfo.urllen - vicelen, ".js", vicelen) == 0) {
		    DEBUGP( PRINTK_LEVEL "%s: MATCH....java\n", __FUNCTION__);
		    found = 1;
		    goto match_ret;
		}
		vicelen = sizeof(".class") - 1;
		if (strnicmp(htinfo.url + htinfo.urllen - vicelen, ".class", vicelen) == 0) {
		    DEBUGP( PRINTK_LEVEL "%s: MATCH....java\n", __FUNCTION__);
		    found = 1;
		    goto match_ret;
		}
	    }
	    if (opt & BLK_ACTIVE){
		vicelen = sizeof(".ocx") - 1;
		if (strnicmp(htinfo.url + htinfo.urllen - vicelen, ".ocx", vicelen) == 0) {
		    DEBUGP( PRINTK_LEVEL "%s: MATCH....activex\n", __FUNCTION__);
		    found = 1;
		    goto match_ret;
		}
		vicelen = sizeof(".cab") - 1;
		if (strnicmp(htinfo.url + htinfo.urllen - vicelen, ".cab", vicelen) == 0) {
		    DEBUGP( PRINTK_LEVEL "%s: MATCH....activex\n", __FUNCTION__);
		    found = 1;
		    goto match_ret;
		}
	    }
	    if (opt & BLK_PROXY){
		if (strnicmp(htinfo.url + htinfo.hostlen, "http://", sizeof("http://") - 1) == 0) {
		    DEBUGP( PRINTK_LEVEL "%s: MATCH....proxy\n", __FUNCTION__);
		    found = 1;
		    goto match_ret;
		}
	    }
	}

match_ret:
	DEBUGP( PRINTK_LEVEL "%s: Verdict =======> %s \n",__FUNCTION__
		, found ? "MATCH FOUND" : "MATCH NOT FOUND");

	return (found ^ info->invert);
}


static struct xt_match webstr_match __read_mostly = {
	.name		= "webstr",
	.family		= NFPROTO_IPV4,
	.match		= ipt_webstr_match,
	.matchsize      = sizeof(struct ipt_webstr_info),
	.me		= THIS_MODULE,
};


static int __init ipt_webstr_init(void)
{
	return xt_register_match(&webstr_match);
}

static void __exit ipt_webstr_fini(void)
{
	xt_unregister_match(&webstr_match);
}

module_init(ipt_webstr_init);
module_exit(ipt_webstr_fini);
