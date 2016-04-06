#include <linux/init.h>
#include <linux/ip.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/tcp.h>
#include <linux/netfilter/x_tables.h>
#include <linux/netfilter/xt_httpurl.h>

MODULE_AUTHOR("");
MODULE_DESCRIPTION("");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xt_httpurl");
MODULE_ALIAS("ipt_httpurl");
MODULE_ALIAS("ip6t_httpurl");

#define MAX_URL_SZ          (256)
#define MIN_HTTP_PACKET_SZ  (16)
#define PATTERN_FOUND       (1)
#define PATTERN_NOT_FOUND   (0)

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

static int str_endswith(char *suffix, unsigned int suffix_len,
                        char *str, unsigned int str_len)
{
	if (str_len < suffix_len) {
		return 0;
	}
	
	return (strnicmp(str + str_len - suffix_len, suffix, suffix_len) == 0);
}

static int match(const struct sk_buff *skb,
                 const struct xt_action_param *par)
{
	const struct xt_httpurl_info *info = par->matchinfo;
	struct tcphdr _tcph, *tcp_header;
	unsigned char *data;
	unsigned int data_len;
	unsigned int url_offset = 0;
	unsigned int url_len = 0;
	int found;
	int filter_match = 0;
	int i;
	
	/* Ignore non-head fragments */
	if (par->fragoff != 0) {
		return 0;
	}
	
	tcp_header = skb_header_pointer(skb, par->thoff, sizeof(_tcph), &_tcph);
	if (tcp_header == NULL) {
		return 0;
	}
	
	data = (unsigned char *)tcp_header + tcp_header->doff * 4;
	data_len = skb->len - ip_hdr(skb)->ihl * 4 - tcp_header->doff * 4;
	
	/* Ignore packets that are too short to be HTTP packets */
	if (data_len < MIN_HTTP_PACKET_SZ) {
		return 0;
	}
	
	found = find_pattern("GET ",
	                     sizeof("GET ") - 1,
	                     '\r',
	                     data,
	                     data_len,
	                     &url_offset,
	                     &url_len);
	if (found == PATTERN_NOT_FOUND) {
		found = find_pattern("POST ",
		                     sizeof("POST ") - 1,
		                     '\r',
		                     data,
		                     data_len,
		                     &url_offset,
		                     &url_len);
	}
	if (found == PATTERN_NOT_FOUND) {
		found = find_pattern("HEAD ",
		                     sizeof("HEAD ") - 1,
		                     '\r',
		                     data,
		                     data_len,
		                     &url_offset,
		                     &url_len);
	}
	
	/* Ignore packets that don't represent HTTP methods */
	if (found == PATTERN_NOT_FOUND) {
		return 0;
	}
	
	/* Truncate the HTTP version from the end of the URL */
	url_len -= (sizeof(" HTTP/x.x") - 1);
	
	/* Check the bounds on the length of the host name */
	if (url_len <= 0 || url_len > MAX_URL_SZ) {
		return 0;
	}
	
	/* Truncate any parameters from the URL */
	for (i = 1; i < url_len; i++) {
		if (data[url_offset + i] == '?') {
			url_len = i;
			break;
		}
	}
	
	if (info->match_java) {
		if (str_endswith(".class", sizeof(".class") - 1, data + url_offset, url_len) ||
		      str_endswith(".js", sizeof(".js") - 1, data + url_offset, url_len)) {
			filter_match = 1;
			goto done;
		}
	}
	if (info->match_activex) {
		if (str_endswith(".ocx", sizeof(".ocx") - 1, data + url_offset, url_len)) {
			filter_match = 1;
			goto done;
		}
		if (str_endswith(".cab", sizeof(".cab") - 1, data + url_offset, url_len)) {
			filter_match = 1;
			goto done;
		}
	}
	if (info->match_proxy) {
		unsigned int host_offset = 0;
		unsigned int host_len = 0;
		
		found = find_pattern("Host: ",
		                     sizeof("Host: ") - 1,
		                     '\r',
		                     data,
		                     data_len,
		                     &host_offset,
		                     &host_len);
		
		if (found == PATTERN_NOT_FOUND) {
			return 0;
		}
		
		if (strnicmp(data + url_offset, "http://", sizeof("http://") - 1) == 0) {
			filter_match = 1;
			goto done;
		}
	}

done:
	return filter_match ^ info->invert;
}

static int checkentry(const struct xt_mtchk_param *par)
{
	return 0;
}

static void destroy(const struct xt_mtdtor_param *par)
{
}

static struct xt_match xt_httpurl_match[] = {
	{
		.name 		= "httpurl",
		.family		= AF_INET,
		.checkentry	= checkentry,
		.match 		= match,
		.destroy 	= destroy,
		.matchsize	= sizeof(struct xt_httpurl_info),
		.me 		= THIS_MODULE
	},
	{
		.name 		= "httpurl",
		.family		= AF_INET6,
		.checkentry	= checkentry,
		.match 		= match,
		.destroy 	= destroy,
		.matchsize	= sizeof(struct xt_httpurl_info),
		.me 		= THIS_MODULE
	},
};

static int __init xt_httpurl_init(void)
{
	return xt_register_matches(xt_httpurl_match, ARRAY_SIZE(xt_httpurl_match));
}

static void __exit xt_httpurl_exit(void)
{
	xt_unregister_matches(xt_httpurl_match, ARRAY_SIZE(xt_httpurl_match));
}

module_init(xt_httpurl_init);
module_exit(xt_httpurl_exit);
