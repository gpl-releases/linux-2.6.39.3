#include <linux/init.h>
#include <linux/ip.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/tcp.h>
#include <linux/netfilter/x_tables.h>
#include <linux/netfilter/xt_httphost.h>

MODULE_AUTHOR("");
MODULE_DESCRIPTION("");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xt_httphost");
MODULE_ALIAS("ipt_httphost");
MODULE_ALIAS("ip6t_httphost");

#define MIN_HTTP_PACKET_SZ  (16)
#define PATTERN_FOUND       (1)
#define PATTERN_NOT_FOUND   (0)
#define HOSTS_DO_MATCH      (1)
#define HOSTS_DO_NOT_MATCH  (0)

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

/* Returns HOSTS_DO_MATCH or HOSTS_DO_NOT_MATCH */
static int http_host_cmp(const char *top_level_host,
                         const char *test_host)
{
	unsigned int top_level_host_len;
	unsigned int test_host_len;
	
	if (test_host == NULL || top_level_host == NULL) {
		return HOSTS_DO_NOT_MATCH;
	}
	
	top_level_host_len = strlen(top_level_host);
	test_host_len = strlen(test_host);
	
	/* If the top level host is longer than the test host,
	   we can already conclude that they don't match */
	if (top_level_host_len > test_host_len) {
		return HOSTS_DO_NOT_MATCH;
	}
	
	/* Check whether the hosts are equal */
	if (strnicmp(test_host, top_level_host, top_level_host_len) == 0) {
		return HOSTS_DO_MATCH;
	}
	
	/* Check whether the test host is a subdomain of the top level host
	   (the entire top level host string needs to appear at the end of the
	   test host string, preceded by a '.' */
	if (test_host_len >= top_level_host_len + 1 &&
	    test_host[test_host_len - top_level_host_len - 1] == '.' &&
	    strnicmp(top_level_host,
	             &test_host[test_host_len - top_level_host_len],
	             top_level_host_len) == 0)
	{
		return HOSTS_DO_MATCH;
	}
	
	return HOSTS_DO_NOT_MATCH;
}

static int match(const struct sk_buff *skb,
                 const struct xt_action_param *par)
{
	const struct xt_httphost_info *info = par->matchinfo;
	struct tcphdr _tcph, *tcp_header;
	unsigned char *data;
	unsigned int data_len;
	unsigned int host_offset = 0;
	unsigned int host_len = 0;
	char host_buf[XT_HTTPHOST_MAX_HOST_SIZE];
	int does_host_match = 0;
	
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
	
	/* Ignore packets that don't represent HTTP methods */
	if (memcmp(data, "GET ", sizeof("GET ") - 1) != 0 &&
        memcmp(data, "POST ", sizeof("POST ") - 1) != 0 &&
        memcmp(data, "HEAD ", sizeof("HEAD ") - 1) != 0)
	{
		return 0;
	}
	
	/* Ignore packets without a host header */
	if (find_pattern("Host: ",
                     sizeof("Host: ") - 1,
                     '\r',
                     data,
                     data_len,
                     &host_offset,
                     &host_len) == PATTERN_NOT_FOUND)
	{
		return 0;
	}
	
	/* Check the bounds on the length of the host name */
	if (host_len <= 0 || host_len >= XT_HTTPHOST_MAX_HOST_SIZE - 1) {
		return 0;
	}
	
	strncpy(host_buf, data + host_offset, host_len);
	*(host_buf + host_len) = 0;
	
	does_host_match = (http_host_cmp(info->host, host_buf) == HOSTS_DO_MATCH);
	
	return does_host_match ^ info->invert;
}

static int checkentry(const struct xt_mtchk_param *par)
{
	return 0;
}

static void destroy(const struct xt_mtdtor_param *par)
{
}

static struct xt_match xt_httphost_match[] __read_mostly = {
	{
		.name 		= "httphost",
		.family		= AF_INET,
		.checkentry	= checkentry,
		.match 		= match,
		.destroy 	= destroy,
		.matchsize	= sizeof(struct xt_httphost_info),
		.me 		= THIS_MODULE
	},
	{
		.name 		= "httphost",
		.family		= AF_INET6,
		.checkentry	= checkentry,
		.match 		= match,
		.destroy 	= destroy,
		.matchsize	= sizeof(struct xt_httphost_info),
		.me 		= THIS_MODULE
	},
};

static int __init xt_httphost_init(void)
{
	return xt_register_matches(xt_httphost_match, ARRAY_SIZE(xt_httphost_match));
}

static void __exit xt_httphost_exit(void)
{
	xt_unregister_matches(xt_httphost_match, ARRAY_SIZE(xt_httphost_match));
}

module_init(xt_httphost_init);
module_exit(xt_httphost_exit);
