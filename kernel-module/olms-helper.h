#ifndef _OLMS_HELPER_H_
#define _OLMS_HELPER_H_

struct olms_cmd_args {
	union {
		unsigned long start;
		unsigned long addr1;
		unsigned long src;
		unsigned long long sid;
		unsigned long rtt_vec_addr;
	};
	union {
		unsigned long end;
		unsigned long addr2;
		unsigned long dst;
		unsigned num_nodes;
		unsigned long bw_vec_addr;
	};
	union {
		unsigned long priv;
		unsigned long len;
		unsigned threads_per_loader;
		unsigned long loss_vec_addr;
	};
};

enum OLMS_CMD {
	OLMS_CMD_IMPLEMENTED = 0,
	OLMS_CMD_CLIENT = 1,
	OLMS_CMD_SERVER = 2,
	OLMS_CMD_NUMPATHS = 3,
	OLMS_CMD_GET_RTT = 4,
	OLMS_CMD_GET_BW = 5,
	OLMS_CMD_PREFER = 6,
};

#define OLMS_IOC_MAGIC 0xCA

/* Macros for OLMS */
#define OLMS_IOC_CLIENT                                                        \
	_IOWR(OLMS_IOC_MAGIC, OLMS_CMD_CLIENT, struct olms_cmd_args)
#define OLMS_IOC_SERVER                                                        \
	_IOWR(OLMS_IOC_MAGIC, OLMS_CMD_SERVER, struct olms_cmd_args)
#define OLMS_IOC_NUMPATHS                                                      \
	_IOWR(OLMS_IOC_MAGIC, OLMS_CMD_NUMPATHS, struct olms_cmd_args)
#define OLMS_IOC_GET_RTT                                                       \
	_IOWR(OLMS_IOC_MAGIC, OLMS_CMD_GET_RTT, struct olms_cmd_args)
#define OLMS_IOC_GET_BW                                                        \
	_IOWR(OLMS_IOC_MAGIC, OLMS_CMD_GET_BW, struct olms_cmd_args)
#define OLMS_IOC_PREFER                                                        \
	_IOWR(OLMS_IOC_MAGIC, OLMS_CMD_PREFER, struct olms_cmd_args)
#define OLMS_IOC_MAXNR 6

#endif /* _OLMS_HELPER_H_ */
