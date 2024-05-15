#ifndef PTASK_H
#define PTASK_H

#define _GNU_SOURCE
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sched.h>
#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <sys/syscall.h>
#include <pthread.h>

#define gettid() syscall(__NR_gettid)

/* Scheduling policies */
#define SCHED_NORMAL 	0
#define SCHED_FIFO 		1
#define SCHED_RR 		2
#define SCHED_BATCH 	3
#define SCHED_IDLE 		5
#define SCHED_DEADLINE	6

/* XX use the proper syscall numbers */
#ifdef __x86_64__
#define __NR_sched_setattr		314
#define __NR_sched_getattr		315
#endif

#ifdef __i386__
#define __NR_sched_setattr		351
#define __NR_sched_getattr		352
#endif

#ifdef __arm__
#define __NR_sched_setattr		380
#define __NR_sched_getattr		381
#endif

struct sched_attr {
	__u32 size;

	__u32 sched_policy;
	__u64 sched_flags;

	/* SCHED_NORMAL, SCHED_BATCH */
	__s32 sched_nice;

	/* SCHED_FIFO, SCHED_RR */
	__u32 sched_priority;

	/* SCHED_DEADLINE (nsec) */
	__u64 sched_runtime;
	__u64 sched_deadline;
	__u64 sched_period;
 };

struct 		task_par {
    int     			arg;				/* task argument		*/
    long    			wcet;				/* in microseconds		*/
    float     			period;				/* in milliseconds		*/
    float     			deadline;			/* relative (ms)		*/
    float				priority;			/* in [0,99]			*/
	int					dmiss;				/* number of misses		*/
	struct	timespec 	at;					/* next activation time */
	struct	timespec 	dl;					/* absolute deadline	*/
	struct sched_attr 	attr;
};

int sched_setattr(pid_t pid, const struct sched_attr *attr, int flags);
int sched_getattr(pid_t pid, struct sched_attr *attr, int size, int flags);

void time_copy(struct timespec *td, struct timespec ts);
void time_add_ms(struct timespec *t, int ms);
int time_cmp(struct timespec t1, struct timespec t2);
pthread_t task_create(int i, void *(*body)(void *), int policy, int flags, int nice, int priority, float runtime, float period, float deadline);
void set_period(struct task_par *tp);
void wait_for_period(struct task_par *tp);
int deadline_miss(struct task_par *tp);

#endif
