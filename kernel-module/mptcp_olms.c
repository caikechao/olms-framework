/* MPTCP Scheduler module selector. Highly inspired by tcp_cong.c */
/* Most of the functions are adpated from the file mptcp_sched.c  */

#include <net/mptcp.h>
#include <trace/events/tcp.h>
#include <linux/vmalloc.h>
#include <uapi/linux/inet_diag.h>
#include <asm/uaccess.h> // copy_to_user()
#include <linux/errno.h> // error codes
#include <uapi/linux/olms-helper.h>
#include <linux/init.h> // module_init and module_exit
#include <linux/module.h> // THIS_MODULE, kernel versioning info
#include <linux/fs.h> // register_chrdev and unregister_chrdev
#include <linux/kernel.h> // printk
#include <linux/cdev.h> // character device stuff

#define MODULE_NAME "olms"

struct olms_dev {
	struct cdev cdev;
} olms_dev;

/* struct subflow_measurement rtt_bw_measurements[]; */
struct olms_measurement_data {
	u32 rtt[10];
	u32 bw[10];
	u32 loss[10];
} measurement;

static u32 user_path_id = 0;
static u32 kernel_path_id = 0;


dev_t devno;

struct class *cl;

struct mptcp_cb *target_mpcb = NULL;
static u32 target_num_subflows = 0;
static unsigned long path_status_bits;
static u32 last_srtt = 0xffffffff;
static bool olms_enabled = false;

static u32 pref[2];
static unsigned long pref_index_bits = 0;
static spinlock_t pref_lock;

struct olmssched_priv {
	u32	last_rbuf_opti;
};

static struct olmssched_priv *olmssched_get_priv(const struct tcp_sock *tp)
{
	return (struct olmssched_priv *)&tp->mptcp->mptcp_sched[0];
}

/* Are we not allowed to reinject this skb on tp? */
static int mptcp_olms_dont_reinject_skb(const struct tcp_sock *tp, const struct sk_buff *skb)
{
	/* If the skb has already been enqueued in this sk, try to find
	 * another one.
	 */
	return skb &&
		/* Has the skb already been enqueued into this subsocket? */
		mptcp_pi_to_flag(tp->mptcp->path_index) & TCP_SKB_CB(skb)->path_mask;
}

static inline int max_num_subflows(struct mptcp_cb *target_mpcb)
{
	return sizeof(target_mpcb->path_index_bits) * 8;
}

static bool mptcp_olms_is_temp_unavailable(struct sock *sk,
					const struct sk_buff *skb,
					bool zero_wnd_test)
{
	const struct tcp_sock *tp = tcp_sk(sk);
	unsigned int mss_now, space, in_flight;

	if (inet_csk(sk)->icsk_ca_state == TCP_CA_Loss) {
		/* If SACK is disabled, and we got a loss, TCP does not exit
		 * the loss-state until something above high_seq has been
		 * acked. (see tcp_try_undo_recovery)
		 *
		 * high_seq is the snd_nxt at the moment of the RTO. As soon
		 * as we have an RTO, we won't push data on the subflow.
		 * Thus, snd_una can never go beyond high_seq.
		 */
		if (!tcp_is_reno(tp))
			return true;
		else if (tp->snd_una != tp->high_seq)
			return true;
	}

	if (!tp->mptcp->fully_established) {
		/* Make sure that we send in-order data */
		if (skb && tp->mptcp->second_packet &&
			tp->mptcp->last_end_data_seq != TCP_SKB_CB(skb)->seq)
			return true;
	}

	in_flight = tcp_packets_in_flight(tp);
	/* Not even a single spot in the cwnd */
	if (in_flight >= tp->snd_cwnd)
		return true;

	/* Now, check if what is queued in the subflow's send-queue
	 * already fills the cwnd.
	 */
	space = (tp->snd_cwnd - in_flight) * tp->mss_cache;

	if (tp->write_seq - tp->snd_nxt > space)
		return true;

	if (zero_wnd_test && !before(tp->write_seq, tcp_wnd_end(tp)))
		return true;

	mss_now = tcp_current_mss(sk);

	/* Don't send on this subflow if we bypass the allowed send-window at
	 * the per-subflow level. Similar to tcp_snd_wnd_test, but manually
	 * calculated end_seq (because here at this point end_seq is still at
	 * the meta-level).
	 */
	if (skb && zero_wnd_test &&
		after(tp->write_seq + min(skb->len, mss_now), tcp_wnd_end(tp)))
		return true;

	return false;
}

static void check_olms_target(struct sock *meta_sk)
{
	struct inet_sock *inet;
	struct mptcp_tcp_sock *mptcp;

	if (unlikely(target_mpcb))
		return;

	inet = inet_sk(meta_sk);
	// Fix the port issue. Fix the port for flow debugging
	if (likely(ntohs(inet->inet_dport) != 8999))
		return;

	target_mpcb = tcp_sk(meta_sk)->mpcb;
	/* FIXME */
	mptcp_for_each_sub(target_mpcb, mptcp)
	{
		if (mptcp->fully_established) {
			target_num_subflows++;
			set_bit(mptcp->path_index, &path_status_bits);
		} else {
			clear_bit(mptcp->path_index, &path_status_bits);
		}
	}
	pr_info("MPTCP %pI4:%u(L) -> %pI4:%u(R)\n", &inet->inet_saddr,
		ntohs(inet->inet_sport), &inet->inet_daddr,
		ntohs(inet->inet_dport));
}

/* Generic function to iterate over used and unused subflows and to select the
 * best one
 */
static struct sock
*olms_get_subflow_from_selectors(struct mptcp_cb *mpcb, struct sk_buff *skb,
				bool (*selector)(const struct tcp_sock *),
				bool zero_wnd_test, bool *force)
{
	struct sock *bestsk = NULL;
	u32 min_srtt = 0xffffffff;
	bool found_unused = false;
	bool found_unused_una = false;
	struct mptcp_tcp_sock *mptcp;

	mptcp_for_each_sub(mpcb, mptcp) {
		struct sock *sk = mptcp_to_sock(mptcp);
		struct tcp_sock *tp = tcp_sk(sk);
		bool unused = false;

		/* First, we choose only the wanted sks */
		if (!(*selector)(tp))
			continue;

		if (!mptcp_olms_dont_reinject_skb(tp, skb))
			unused = true;
		else if (found_unused)
			/* If a unused sk was found previously, we continue -
			 * no need to check used sks anymore.
			 */
			continue;

		if (mptcp_is_def_unavailable(sk))
			continue;

		if (mptcp_olms_is_temp_unavailable(sk, skb, zero_wnd_test)) {
			if (unused)
				found_unused_una = true;
			continue;
		}

		if (unused) {
			if (!found_unused) {
				/* It's the first time we encounter an unused
				 * sk - thus we reset the bestsk (which might
				 * have been set to a used sk).
				 */
				min_srtt = 0xffffffff;
				bestsk = NULL;
			}
			found_unused = true;
		}
		/* read and write driver for the user space program. */

		/* Choose the best subflow according to the user space program. */

		/* only subflows with MPTCP enabled and
		 * part of data transfer will reach this point
		* Call these functions only if the subflow
		* needs to be selected
		*/
		/*
		* qSize = get_queue_size(sk);
		* wait_time = qSize;
		* wait_time = get_wait_time(sk); //send sk and qSize
		* curr_service =
		* 	qSize *
		* 	(tp->srtt_us -
		* 	 (wait_time / 10000000)); //convert wait time to srtt time units
		*/
		/*
		 * if (curr_service < min_service){
		 *    min_service = curr_service;
		 *    bestsk = sk;
		 * }
		 */
		//printk(KERN_INFO "Path Index: %u, SRTT: %u, Wait Time: %li, WDiv10^7:
		//%li, Queue Size: %u, Service Time: %u", tp->mptcp->path_index,
		//tp->srtt_us, wait_time, wait_time/10000000, qSize, curr_service);

		if (tp->srtt_us < min_srtt) {
			min_srtt = tp->srtt_us;
			bestsk = sk;
		}
	}

	if (bestsk) {
		/* The force variable is used to mark the returned sk as
		 * previously used or not-used.
		 */
		if (found_unused)
			*force = true;
		else
			*force = false;
	} else {
		/* The force variable is used to mark if there are temporally
		 * unavailable not-used sks.
		 */
		if (found_unused_una)
			*force = true;
		else
			*force = false;
	}

	if (last_srtt == 0 || last_srtt == 0xffffffff) {
		last_srtt = min_srtt;
	}

	return bestsk;
}

/* This is the scheduler. This function decides on which flow to send
 * a given MSS. If all subflows are found to be busy, NULL is returned
 * The flow is selected based on the shortest RTT.
 * If all paths have full cong windows, we simply return NULL.
 *
 * Additionally, this function is aware of the backup-subflows.
 */
struct sock *olms_get_available_subflow(struct sock *meta_sk, struct sk_buff *skb,
					bool zero_wnd_test)
{
	struct mptcp_cb *mpcb = tcp_sk(meta_sk)->mpcb;
	struct sock *sk;
	bool looping = false, force;

	check_olms_target(meta_sk);

	/* Answer data_fin on same subflow!!! */
	if (meta_sk->sk_shutdown & RCV_SHUTDOWN &&
		skb && mptcp_is_data_fin(skb)) {
		struct mptcp_tcp_sock *mptcp;

		mptcp_for_each_sub(mpcb, mptcp) {
			sk = mptcp_to_sock(mptcp);

			if (tcp_sk(sk)->mptcp->path_index == mpcb->dfin_path_index &&
				mptcp_is_available(sk, skb, zero_wnd_test))
				return sk;
		}
	}

	/* Find the best subflow */
restart:
	sk = olms_get_subflow_from_selectors(mpcb, skb, &subflow_is_active,
					zero_wnd_test, &force);
	if (force)
		/* one unused active sk or one NULL sk when there is at least
		 * one temporally unavailable unused active sk
		 */
		return sk;

	sk = olms_get_subflow_from_selectors(mpcb, skb, &subflow_is_backup,
					zero_wnd_test, &force);
	if (!force && skb) {
		/* one used backup sk or one NULL sk where there is no one
		 * temporally unavailable unused backup sk
		 *
		 * the skb passed through all the available active and backups
		 * sks, so clean the path mask
		 */
		TCP_SKB_CB(skb)->path_mask = 0;

		if (!looping) {
			looping = true;
			goto restart;
		}
	}
	return sk;
}

static struct sk_buff *mptcp_olms_rcv_buf_optimization(struct sock *sk, int penal)
{
	struct sock *meta_sk;
	const struct tcp_sock *tp = tcp_sk(sk);
	struct mptcp_tcp_sock *mptcp;
	struct sk_buff *skb_head;
	struct olmssched_priv *def_p = olmssched_get_priv(tp);

	meta_sk = mptcp_meta_sk(sk);
	skb_head = tcp_rtx_queue_head(meta_sk);

	if (!skb_head)
		return NULL;

	/* If penalization is optional (coming from mptcp_next_segment() and
	 * We are not send-buffer-limited we do not penalize. The retransmission
	 * is just an optimization to fix the idle-time due to the delay before
	 * we wake up the application.
	 */
	if (!penal && sk_stream_memory_free(meta_sk))
		goto retrans;

	/* Only penalize again after an RTT has elapsed */
	if (tcp_jiffies32 - def_p->last_rbuf_opti < usecs_to_jiffies(tp->srtt_us >> 3))
		goto retrans;

	/* Half the cwnd of the slow flows */
	mptcp_for_each_sub(tp->mpcb, mptcp) {
		struct tcp_sock *tp_it = mptcp->tp;

		if (tp_it != tp &&
			TCP_SKB_CB(skb_head)->path_mask & mptcp_pi_to_flag(tp_it->mptcp->path_index)) {
			if (tp->srtt_us < tp_it->srtt_us && inet_csk((struct sock *)tp_it)->icsk_ca_state == TCP_CA_Open) {
				u32 prior_cwnd = tp_it->snd_cwnd;

				tp_it->snd_cwnd = max(tp_it->snd_cwnd >> 1U, 1U);

				/* If in slow start, do not reduce the ssthresh */
				if (prior_cwnd >= tp_it->snd_ssthresh)
					tp_it->snd_ssthresh = max(tp_it->snd_ssthresh >> 1U, 2U);

				def_p->last_rbuf_opti = tcp_jiffies32;
			}
		}
	}

retrans:

	/* Segment not yet injected into this path? Take it!!! */
	if (!(TCP_SKB_CB(skb_head)->path_mask & mptcp_pi_to_flag(tp->mptcp->path_index))) {
		bool do_retrans = false;
		mptcp_for_each_sub(tp->mpcb, mptcp) {
			struct tcp_sock *tp_it = mptcp->tp;

			if (tp_it != tp &&
				TCP_SKB_CB(skb_head)->path_mask & mptcp_pi_to_flag(tp_it->mptcp->path_index)) {
				if (tp_it->snd_cwnd <= 4) {
					do_retrans = true;
					break;
				}

				if (4 * tp->srtt_us >= tp_it->srtt_us) {
					do_retrans = false;
					break;
				} else {
					do_retrans = true;
				}
			}
		}

		if (do_retrans && mptcp_is_available(sk, skb_head, false)) {
			trace_mptcp_retransmit(sk, skb_head);
			return skb_head;
		}
	}
	return NULL;
}

/* Returns the next segment to be sent from the mptcp meta-queue.
 * (chooses the reinject queue if any segment is waiting in it, otherwise,
 * chooses the normal write queue).
 * Sets *@reinject to 1 if the returned segment comes from the
 * reinject queue. Sets it to 0 if it is the regular send-head of the meta-sk,
 * and sets it to -1 if it is a meta-level retransmission to optimize the
 * receive-buffer.
 */
static struct sk_buff *__mptcp_olms_next_segment(struct sock *meta_sk, int *reinject)
{
	const struct mptcp_cb *mpcb = tcp_sk(meta_sk)->mpcb;
	struct sk_buff *skb = NULL;

	*reinject = 0;

	/* If we are in fallback-mode, just take from the meta-send-queue */
	if (mpcb->infinite_mapping_snd || mpcb->send_infinite_mapping)
		return tcp_send_head(meta_sk);

	skb = skb_peek(&mpcb->reinject_queue);

	if (skb) {
		*reinject = 1;
	} else {
		skb = tcp_send_head(meta_sk);

		if (!skb && meta_sk->sk_socket &&
			test_bit(SOCK_NOSPACE, &meta_sk->sk_socket->flags) &&
			sk_stream_wspace(meta_sk) < sk_stream_min_wspace(meta_sk)) {
			struct sock *subsk = olms_get_available_subflow(meta_sk, NULL,
									false);
			if (!subsk)
				return NULL;

			skb = mptcp_olms_rcv_buf_optimization(subsk, 0);
			if (skb)
				*reinject = -1;
		}
	}
	return skb;
}

static struct sk_buff *mptcp_olms_next_segment(struct sock *meta_sk,
					int *reinject,
					struct sock **subsk,
					unsigned int *limit)
{
	struct sk_buff *skb = __mptcp_olms_next_segment(meta_sk, reinject);
	unsigned int mss_now;
	struct tcp_sock *subtp;
	u16 gso_max_segs;
	u32 max_len, max_segs, window, needed;

	/* As we set it, we have to reset it as well. */
	*limit = 0;

	if (!skb)
		return NULL;

	*subsk = olms_get_available_subflow(meta_sk, skb, false);
	if (!*subsk)
		return NULL;

	subtp = tcp_sk(*subsk);
	mss_now = tcp_current_mss(*subsk);

	if (!*reinject && unlikely(!tcp_snd_wnd_test(tcp_sk(meta_sk), skb, mss_now))) {
		skb = mptcp_olms_rcv_buf_optimization(*subsk, 1);
		if (skb)
			*reinject = -1;
		else
			return NULL;
	}

	/* No splitting required, as we will only send one single segment */
	if (skb->len <= mss_now)
		return skb;

	/* The following is similar to tcp_mss_split_point, but
	 * we do not care about nagle, because we will anyways
	 * use TCP_NAGLE_PUSH, which overrides this.
	 *
	 * So, we first limit according to the cwnd/gso-size and then according
	 * to the subflow's window.
	 */

	gso_max_segs = (*subsk)->sk_gso_max_segs;
	if (!gso_max_segs) /* No gso supported on the subflow's NIC */
		gso_max_segs = 1;
	max_segs = min_t(unsigned int, tcp_cwnd_test(subtp, skb), gso_max_segs);
	if (!max_segs)
		return NULL;

	max_len = mss_now * max_segs;
	window = tcp_wnd_end(subtp) - subtp->write_seq;

	needed = min(skb->len, window);
	if (max_len <= skb->len)
		/* Take max_win, which is actually the cwnd/gso-size */
		*limit = max_len;
	else
		/* Or, take the window */
		*limit = needed;

	return skb;
}

static int olms_get_measurement_data(){
	struct mptcp_tcp_sock *mptcp;
	if (!target_mpcb)
		return -ENOENT;

	__uaccess_begin();
	mptcp_for_each_sub(target_mpcb, mptcp)
	{
		struct sock *sk = mptcp_to_sock(mptcp);
		struct tcp_sock *tp = tcp_sk(sk);
		/* const struct tcp_congestion_ops *ca;
		 * union tcp_cc_info info;
		 * int attr;
		 */
		int pi = tp->mptcp->path_index;
		/* pi starts from 1, therefore we minus 1. */
		measurement.rtt[pi - 1] = tp->srtt_us >> 3;
		/* disable the bw measurements temporally. */
		measurement.bw[pi - 1]= do_div(tp->rate_delivered, tp->rate_interval_us);
		measurement.loss[pi - 1]= do_div(tp->lost, tp->rate_interval_us);
		/* rtt_vec[pi - 1] = max(tp->srtt_us >> 3, 1U); */
		if (0 /* DEBUG */) {
			pr_info("subflow[%d] lsndtime %u unavaialable %u\n", pi,
				tp->lsndtime, mptcp->unavailable);
		}
		if (0 /* DEBUG */) {
			pr_info("selection: %u backup %u\n",
				target_mpcb->mptcp_num_selection,
				target_mpcb->mptcp_num_backup);
		}
		if (pi > target_num_subflows)
			target_num_subflows = pi;
	}
	__uaccess_end();

	return 0;
}

static int olms_get_measurement(void *data)
{
	struct olms_cmd_args *args = data;
	struct mptcp_tcp_sock *mptcp;

	u32 *rtt_vec, *bw_vec, *loss_vec;
	/* * u32 *loss_vec, *delivered_vec; */

	int err = 0;

	if (!target_mpcb)
		return -ENOENT;

	err = !access_ok(VERIFY_WRITE, args->rtt_vec_addr,
			 max_num_subflows(target_mpcb) * sizeof(u32));
	if (err)
		return err;

	rtt_vec = (u32 *) args->rtt_vec_addr;
	bw_vec  = (u32 *) args->bw_vec_addr;
	loss_vec  = (u32 *) args->loss_vec_addr;

	/*
	 * loss_vec = (u32 *)args->loss_vec_addr;
	 * delivered_vec = (u32 *)args->delivered_vec_addr;
	 */

	/*
	 * path_index is in reverse order because of linked-list insertion.
	 * Also, path_index starts from 1.
	 * FIXME However, it seems there can be holes in path_index array.
	 * Maybe I should handle the difference between max and num.
	 * How do we ensure that the memory does not overlap.
	 */
	__uaccess_begin();
	mptcp_for_each_sub(target_mpcb, mptcp)
	{
		struct sock *sk = mptcp_to_sock(mptcp);
		struct tcp_sock *tp = tcp_sk(sk);
		const struct tcp_congestion_ops *ca;
		union tcp_cc_info info;
		int attr;
		int pi = tp->mptcp->path_index;

		ca = inet_csk(sk)->icsk_ca_ops;
		// change here, we do not want to depend on BBR.
		if (ca->get_info) {
			/* XXX Only BBR has the get_info() method */
			ca->get_info(sk, (1 << (INET_DIAG_BBRINFO - 1)), &attr,
				     &info);
			if (attr == INET_DIAG_BBRINFO) {
				/* rtt_vec[pi-1]       = info.bbr.bbr_min_rtt; */
				/*
				 * bw_vec[pi - 1] = info.bbr.bbr_bw_lo;
				 * loss_vec[pi - 1] = tp->lost - info.bbr.bbr_last_loss;
				 * delivered_vec[pi - 1] = tp->delivered - info.bbr.bbr_last_delivered;
				 */
			} else {
				/* rtt_vec[pi-1]       = 0; */
				/*
				 * bw_vec[pi - 1] = 0;
				 * loss_vec[pi - 1] = 0;
				 * delivered_vec[pi - 1] = 0;
				 */
			}
		} else {
			rtt_vec[pi - 1] = 0;
			bw_vec[pi - 1] = 0;
			loss_vec[pi - 1] = 0;
			/*
			 * delivered_vec[pi - 1] = 0;
			 */
		}

		/* rtt_us = max(tp->srtt_us >> 3, 1U); */
		rtt_vec[pi - 1] = tp->srtt_us >> 3;
		/* rtt_vec[pi - 1] = max(tp->srtt_us >> 3, 1U); */

		if (0 /* DEBUG */) {
			pr_info("subflow[%d] lsndtime %u unavaialable %u\n", pi,
				tp->lsndtime, mptcp->unavailable);
		}

		if (0 /* DEBUG */) {
			pr_info("selection: %u backup %u\n",
				target_mpcb->mptcp_num_selection,
				target_mpcb->mptcp_num_backup);
		}
		/* if (tp->srtt_us == 0)                                       */
		/*         pr_info("zero srtt: flow %d\n"                      */
		/*                 "\tslave: %d\n"                             */
		/*                 "\testab: %d\n"                             */
		/*                 "\tattac: %d\n"                             */
		/*                 "\tsndmp: %d\n"                             */
		/*                 "\tincmp: %d\n"                             */
		/*                 "\tmappr: %d\n"                             */
		/*                 "\tmapfi: %d\n",                            */
		/*                 pi,                                         */
		/*                 mptcp->slave_sk, mptcp->fully_established,  */
		/*                 mptcp->attached, mptcp->send_mp_fail,       */
		/*                 mptcp->include_mpc, mptcp->mapping_present, */
		/*                 mptcp->map_data_fin);                       */
		if (pi > target_num_subflows)
			target_num_subflows = pi;
	}
	args->len = target_num_subflows;
	__uaccess_end();

	return 0;
}

static void olmssched_init(struct sock *sk)
{
	struct olmssched_priv *def_p = olmssched_get_priv(tcp_sk(sk));

	def_p->last_rbuf_opti = tcp_jiffies32;
}

static void olms_update_preference(u32 *paths, int n)
{
	int i;

	spin_lock(&pref_lock);
	pref_index_bits = 0;
	for (i = 0; i < n; i++) {
		set_bit(paths[i] + 1, &pref_index_bits);
		pref[i] = paths[i] + 1;
	}
	spin_unlock(&pref_lock);
}

/* Get path one-to-one correspondence between kernel space and user space. */
static void olms_set_kernel_path()
{
	int i;
	/* record number of the 1 bits. */
	/* test number of target subflows? */
	/* test path index. */
	/* test get rrt. */
	/* test get bandwidth data. */
	/* test lp solver. */
	int count = 0;
	spin_lock(&pref_lock);
	for (i = 1; i <= target_num_subflows; i++) {
		/* target_num_subflows is supposed to be 16 for a 4 by 4 connection. */
		if (test_bit(i, &path_status_bits)) {
			/* if path is fully established. */
			count += 1;
		}
		if (count == (user_path_id + 1)) /* User path id starts from 0. */
			break;
	}
	kernel_path_id = i;
	spin_unlock(&pref_lock);
}

/*
 * FIXME Actually, it includes the subflows that has indices.
 */
static int olms_num_paths(void *data)
{
	struct olms_cmd_args *args = data;
	struct mptcp_tcp_sock *mptcp;
	u32 num_established_subflows = 0;

	if (!target_mpcb) {
		args->len = 0;
		return -ENOENT;
	}

	mptcp_for_each_sub(target_mpcb, mptcp)
	{
		if (mptcp->fully_established)
			num_established_subflows++;
	}

	if (num_established_subflows > target_num_subflows)
		target_num_subflows = num_established_subflows;
	args->len = num_established_subflows;

	return 0;
}

static int olms_set_preferred_paths(void *data)
{
	struct olms_cmd_args *args = data;
	int err;

	err = !access_ok(VERIFY_READ, args->start, args->len * sizeof(u32));
	return err;

	olms_update_preference((u32 *)args->start, args->len);

	return 0;
}


static int (*olms_cmd_table[])(void *) = {
	/* [OLMS_CMD_CLIENT]      = olms_start_client, */
	/* [OLMS_CMD_SERVER]      = olms_start_server, */
	[OLMS_CMD_NUMPATHS] = olms_num_paths,
	[OLMS_CMD_GET_RTT] = olms_get_measurement,
	[OLMS_CMD_PREFER] = olms_set_preferred_paths,
};


int sanity_check(unsigned int cmd, void *arg)
{
	int err = 0;

	/* make sure the command belongs to this module */
	if (_IOC_TYPE(cmd) != OLMS_IOC_MAGIC) {
		return -ENOTTY;
	}
	/* make sure command number is in valid range */
	if (_IOC_NR(cmd) > OLMS_IOC_MAXNR) {
		return -ENOTTY;
	}
	/* verify user space pointer */
	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err = !access_ok(VERIFY_READ, arg, _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok(VERIFY_Write, arg, _IOC_SIZE(cmd));
	}
	return err;
}


static long olms_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned int cmd_nr = _IOC_NR(cmd);
	struct olms_cmd_args udata;
	int ret = 0, err = 0;

	if (sanity_check(cmd, (void __user *)arg)) {
		printk(KERN_INFO "error with cmd");
		return -EFAULT;
	}

	err = copy_from_user(&udata, (void __user *)arg, sizeof(udata));
	if (err) {
		return -EFAULT;
	}
	ret = olms_cmd_table[cmd_nr](&udata);
	/* If the userspace expects output */
	if (_IOC_DIR(cmd) & _IOC_READ) {
		err = copy_to_user((void __user *)arg, &udata, sizeof(udata));
		if (err) {
			return -EFAULT;
		}
	}

	return ret;
}

static int olms_open(struct inode *inode, struct file *filp)
{
	olms_enabled = true;
	/*
	 * log = vzalloc(sizeof(log[0]) * MAX_LOG_ENTRY);
	 * log_enabled = true;
	 * nlogent = 0;
	 */
	return 0;
}

static int olms_close(struct inode *inode, struct file *filp)
{
	olms_enabled = false;
	/*
	 * log_enabled = false;
	 * mdelay(10);
	 * vfree(log);
	 * nlogent = 0;
	 */
	return 0;
}

static ssize_t olms_read(struct file *fp, char __user *u,
				      size_t len, loff_t *offset)
{
	int err = 0;
	/* call get measurements function. */
	ret = olms_get_measurement_data();
	if (!ret) {
		printk(KERN_NOTICE "get measurement failed.");
		return -EFAULT;
	}
	// copy_to_user has the format ( * to, *from, size) and returns 0 on success
	err = copy_to_user(u, &measurement, sizeof(measurement));
	if (err)
	{
		printk(KERN_NOTICE "copy to user failed.");
		return -EFAULT;
	}
	printk(KERN_INFO "get bandwidth, rtt, n_subflow measurements.\n");
	return 0;
}

/* Get the prefered path index from the user. */
static ssize_t olms_write(struct file *fp, const char __user *u,
				  size_t len, loff_t *offset)
{
	/* set path index function. */
	int err = 0;
	err = copy_from_user(&user_path_id, (void __user *)u, sizeof(user_path_id));
	if (err) {
		return -EFAULT;
	}
	printk(KERN_INFO "set path index from user space.\n");
	return 0;
}

static const struct file_operations olms_fops = {
	.owner = THIS_MODULE,
	/* .llseek = olms_llseek, */
	.read = olms_read,
	.write = olms_write,
	.unlocked_ioctl = olms_ioctl,
	.open = olms_open,
	.release = olms_close,
};

static char *olms_devnode(struct device *dev, umode_t *mode)
{
	if (!mode) {
		return NULL;
	}
	*mode = 0666;
	return NULL;
}

/* use cdev interface to associate device number with a device */
static int olms_setup_cdev(struct olms_dev *olms_dev)
{
	int err;

	if ((err = alloc_chrdev_region(&devno, 0 /* firstminor */,
				       1 /* count */, MODULE_NAME)) != 0) {
		printk(KERN_ERR "fail to allocate character device\n");
		return err;
	}

	/* init chrdev with a set of file_operatoins */
	cdev_init(&olms_dev->cdev, &olms_fops);
	olms_dev->cdev.owner = THIS_MODULE;
	olms_dev->cdev.ops = &olms_fops;

	/* add cdev to the system, associating it with the devno */
	err = cdev_add(&olms_dev->cdev, devno, 1);
	if (err) {
		printk(KERN_ERR "fail to add olms\n");
		return err;
	}

	cl = class_create(THIS_MODULE, MODULE_NAME);
	if (IS_ERR(cl)) {
		printk(KERN_ERR "class_create failed\n");
		return PTR_ERR(cl);
	}

	cl->devnode = olms_devnode;

	device_create(cl, NULL, devno, NULL, MODULE_NAME);

	printk(KERN_INFO "olms cdev created.\n");

	spin_lock_init(&pref_lock);

	return 0;
}

struct mptcp_sched_ops mptcp_sched_olms = {
	.get_subflow = olms_get_available_subflow,
	.next_segment = mptcp_olms_next_segment,
	.init = olmssched_init,
	.name = "olms",
	.owner = THIS_MODULE,
};

static int __init olms_register(void)
{
	BUILD_BUG_ON(sizeof(struct olmssched_priv) > MPTCP_SCHED_SIZE);
	/* BUILD_BUG_ON(sizeof(struct olmssched_cb) > MPTCP_SCHED_DATA_SIZE); */

	if (mptcp_register_scheduler(&mptcp_sched_olms))
		return -1;
	/* also register the char device. */
	olms_setup_cdev(&olms_dev);
	return 0;
}

static void olms_unregister(void)
{
	mptcp_unregister_scheduler(&mptcp_sched_olms);
}

module_init(olms_register);
module_exit(olms_unregister);


MODULE_AUTHOR("Kechao Cai");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OLMS scheduler for MPTCP, based on online learning algorithm");
MODULE_VERSION("0.95");
