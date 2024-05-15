#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "ptask.h"

int sched_setattr(pid_t pid, const struct sched_attr *attr, int flags)
{
	return syscall(__NR_sched_setattr, pid, attr, flags);
}

int sched_getattr(pid_t pid, struct sched_attr *attr, int size, int flags)
{
	return syscall(__NR_sched_getattr, pid, attr, size, flags);
}

/******************************************************************
 *
 * This function copies a source time variable ts in a
 * destination variable pointed by td
 *
******************************************************************/
void time_copy(struct timespec *td, struct timespec ts)
{
	td->tv_sec = ts.tv_sec;
	td->tv_nsec = ts.tv_nsec;
}

/******************************************************************
 *
 * This function adds a value ms expressed in milliseconds to
 * the time variable pointed by t
 *
******************************************************************/
void time_add_ms(struct timespec *t, int ms)
{
	t->tv_sec += ms/1000;
	t->tv_nsec += (ms%1000)*1000000;

	if (t->tv_nsec > 1000000000) {
		t->tv_nsec -= 1000000000;
		t->tv_sec += 1;
	}
}

/******************************************************************
 *
 * This function compares two time variables t1 and t2 and
 * returns 0 if they are equal, 1 if t1 > t2, ‐1 if t1 < t2:
 *
******************************************************************/
int time_cmp(struct timespec t1, struct timespec t2)
{
	if (t1.tv_sec > t2.tv_sec)	return 1;
	if (t1.tv_sec < t2.tv_sec)	return -1;
	if (t1.tv_nsec > t2.tv_nsec)	return 1;
	if (t1.tv_nsec > t2.tv_nsec)	return -1;
	return 0;
}

/******************************************************************
 *
 * This function creates a task with the passed argument.
 * All the time parameters are passed in milliseconds.
 *
******************************************************************/
pthread_t task_create(int i, void *(*body)(void *), int policy,
int flags, int nice, int priority, float runtime, float period, float deadline)
{
int 				res;
struct task_par		tp[60];
struct sched_attr	attr;
pthread_t			tid[60];

	tp[i].arg = i;
	tp[i].wcet = runtime;
	tp[i].period = period;
	tp[i].deadline = deadline;
	tp[i].priority = 0;
	tp[i].dmiss = 0;
	attr.size = sizeof(struct sched_attr);
	attr.sched_flags = flags;
	attr.sched_nice = nice;
	attr.sched_priority = priority;

	attr.sched_policy = policy;
	attr.sched_runtime = runtime * 1000000;
	attr.sched_period = period * 1000000;
	attr.sched_deadline = deadline * 1000000;

	tp[i].attr = attr;

	res = pthread_create(&tid[i], NULL, body, &tp[i]);
	if (res < 0) {
		perror("pthread_create");
		return -1;
	}

	return tid[i];
}

/******************************************************************
 *
 * Reads the current time and computes the next activation
 * time and the absolute deadline of the task.
 *
******************************************************************/
void set_period(struct task_par *tp)
{
struct timespec t;

	clock_gettime(CLOCK_MONOTONIC, &t);
	time_copy(&(tp->at), t);
	time_copy(&(tp->dl), t);
	time_add_ms(&(tp->at), tp->period);
	time_add_ms(&(tp->dl), tp->deadline);
}

/******************************************************************
 *
 * Suspends the calling thread until the next activation and,
 * when awaken, updates activation time and deadline.
 *
******************************************************************/
void wait_for_period(struct task_par *tp)
{
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(tp->at), NULL);
	time_add_ms(&(tp->at), tp->period);
	time_add_ms(&(tp->dl), tp->period);
}

/******************************************************************
 *
 * If the thread is still in execution when re‐activated, it
 * increments the value of dmiss and returns 1, otherwise
 * returns 0.
 *
******************************************************************/
int deadline_miss(struct task_par *tp)
{
struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);

	if (time_cmp(now, tp->dl) > 0) {
		tp->dmiss++;
		return 1;
	}
	return 0;
}
