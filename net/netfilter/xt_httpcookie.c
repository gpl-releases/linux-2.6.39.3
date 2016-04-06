#include <linux/init.h>
#include <linux/ip.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/tcp.h>
#include <linux/netfilter/x_tables.h>
#include <linux/netfilter/xt_httpcookie.h>
#include <net/tcp.h>

MODULE_AUTHOR("");
MODULE_DESCRIPTION("");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xt_httpcookie");
MODULE_ALIAS("ipt_httpcookie");
MODULE_ALIAS("ip6t_httpcookie");

#define PATTERN_FOUND       (1)
#define PATTERN_NOT_FOUND   (0)
#define MIN_HTTP_PACKET_SZ  (16)

/* Returns PATTERN_FOUND or PATTERN_NOT_FOUND */
static int find_pattern(const char *needle,
                        size_t needle_len,
                        char terminal,
                        const char *haystack,
                        size_t haystack_len,
                        unsigned int *match_offset,
                        unsigned int *match_len)
{
	size_t i, j, k;
	
	if (needle_len > haystack_len) {
		return PATTERN_NOT_FOUND;
	}
	
	for (i = 0; i < (haystack_len - needle_len); i++) {
		if (memcmp(haystack + i, needle, needle_len) != 0) {
			continue;
		}
		
		/* We found the needle, so now we need to look
		   for the terminal character */
		*match_offset = i + needle_len;
		for (j = *match_offset, k = 0; haystack[j] != terminal; j++, k++) {
			/* Return if the terminal character isn't found */
	    	if (j > haystack_len) {
	    		return PATTERN_NOT_FOUND;
	    	}
	    }
	    
	    /* We also found the terminal character */
	    *match_len = k;
	    return PATTERN_FOUND;
    }
    
    return PATTERN_NOT_FOUND;
}

static int match(const struct sk_buff *skb,
                 const struct xt_action_param *par)
{
	const struct xt_httpcookie_info *info = par->matchinfo;
	struct tcphdr _tcph, *tcp_header;
	unsigned int tcp_header_len;
	unsigned char *data;
	unsigned int data_len;
	unsigned int cookie_offset = 0;
	unsigned int cookie_len = 0;
	
	/* Ignore non-head fragments */
	if (par->fragoff != 0) {
		return 0;
	}
	
	tcp_header = skb_header_pointer(skb, par->thoff, sizeof(_tcph), &_tcph);
	if (tcp_header == NULL) {
		return 0;
	}
	
	tcp_header_len = skb->len - ip_hdr(skb)->ihl * 4;
	data = (unsigned char *)tcp_header + tcp_header->doff * 4;
	data_len = skb->len - ip_hdr(skb)->ihl * 4 - tcp_header->doff * 4;
	
	/* Ignore packets that are too short to be HTTP packets */
	if (data_len < MIN_HTTP_PACKET_SZ) {
		return 0;
	}
	
	/* Ignore packets that don't represent HTTP methods */
	if (memcmp(data, "GET ", sizeof("GET ") - 1) != 0 &&
        memcmp(data, "POST ", sizeof("POST ") - 1) != 0 &&
        memcmp(data, "HEAD ", sizeof("HEAD ") - 1) != 0)
	{
		return 0;
	}
	
	/* Ignore packets without a Cookie header */
	if (find_pattern("Cookie: ",
	                 sizeof("Cookie: ") - 1,
	                 '\r',
	                 data,
	                 data_len,
	                 &cookie_offset,
	                 &cookie_len) == PATTERN_NOT_FOUND)
	{
		return 0;
	}
	
	/* If cookies are to be blocked, */
	if (info->block_cookies) {
		/* Disable the Cookie header by changing the header name */
		unsigned char *c = data + cookie_offset - (sizeof("Cookie: ") - 1);
		*c++ = 'X'; // 'C' -> 'X'
		*c++ = 'x'; // 'o' -> 'x'
		*c++ = 'x'; // 'o' -> 'x'
		*c++ = 'x'; // 'k' -> 'x'
		*c++ = 'x'; // 'i' -> 'x'
		*c++ = 'x'; // 'e' -> 'x'
		
		/*
		 * Recompute the TCP checksum
		 * Note: You must set the checksum value to 0 prior to recomputing it!
		 */
		tcp_header->check = 0;
		tcp_header->check = tcp_v4_check(tcp_header_len,
		                                 ip_hdr(skb)->saddr,
		                                 ip_hdr(skb)->daddr,
		                                 csum_partial((char *)tcp_header, tcp_header_len, 0));
	}
	
	return 1;
}

static int checkentry(const struct xt_mtchk_param *par)
{
	return 0;
}

static void destroy(const struct xt_mtdtor_param *par)
{
}

static struct xt_match xt_httpcookie_match[] = {
	{
		.name 		= "httpcookie",
		.family		= AF_INET,
		.checkentry	= checkentry,
		.match 		= match,
		.destroy 	= destroy,
		.matchsize	= sizeof(struct xt_httpcookie_info),
		.me 		= THIS_MODULE
	},
	{
		.name 		= "httpcookie",
		.family		= AF_INET6,
		.checkentry	= checkentry,
		.match 		= match,
		.destroy 	= destroy,
		.matchsize	= sizeof(struct xt_httpcookie_info),
		.me 		= THIS_MODULE
	},
};

static int __init xt_httpcookie_init(void)
{
	return xt_register_matches(xt_httpcookie_match, ARRAY_SIZE(xt_httpcookie_match));
}

static void __exit xt_httpcookie_exit(void)
{
	xt_unregister_matches(xt_httpcookie_match, ARRAY_SIZE(xt_httpcookie_match));
}

module_init(xt_httpcookie_init);
module_exit(xt_httpcookie_exit);
