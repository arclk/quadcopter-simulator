/******************************************************************
 *
 * quadcopters.c: SIMULATION OF QUADCOPTERS
 *
******************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <allegro.h>
#include <time.h>
#include <stdbool.h>

#include "ptask.h"

/******************************************************************
 * GLOBAL CONSTANTS
******************************************************************/
#define XWIN 	800				// window x resolution
#define YWIN 	600     		// window y resolution
#define BKG 	0       		// background color (black)
#define PI 		3.1415926536
/******************************************************************/
#define LBOX 	588		// X lenght of the quadcopter box
#define HBOX 	476     // Y height of the quadcopter box
#define XBOX 	12      // left 	box X coordinate
#define YBOX 	112     // upper 	box Y coordinate
#define RBOX 	600		// right 	box X coordinate
#define BBOX 	588		// bottom 	box Y coordinate
/******************************************************************/
#define RUNG	10 		// graphic task wcet
#define PERG	30 		// graphic task period
#define DEAG	30 		// graphic task deadline
#define RUNS 	1	 	// status task wcet
#define PERS 	100 	// status task period
#define DEAS 	100 	// status task deadline
#define RUNQ	1		// quadcopter task wcet
#define PERQ	30		// quadcopter task period
#define DEAQ	30		// quadcopter task deadline
#define RUNC	1		// controller task wcet
#define PERC	30		// controller task period
#define DEAC	30		// controller task deadline
#define RUNM 	1 		// motor task wcet
#define PERM 	15		// motor task period
#define DEAM 	15 		// motor task deadline
/******************************************************************
 * QUADCOPTER CONSTANTS
******************************************************************/
#define MAXQ	12						// max number of quadcopters
#define MAXT	MAXQ*4+2				// max number of threads
#define LQUAD	50						// X lenght of the quadcopter
#define HQUAD	20						// Y lenght of the quadcopter
#define QN		HQUAD					// nose lenght of the quadcopter
#define QW		LQUAD/2					// wing lenght of the quadcopter
#define QD		30						// max dimension from the center
#define XCEN 	((RBOX-XBOX)/2)+XBOX	// initial X position of the quadcopter
#define YCEN 	BBOX-HQUAD-1			// initial Y position of the quadcopter
#define M		0.468					// mass of the quadcopter (kg)
#define L		0.225					// lenght of the wing (m)
#define K		4.29/100000				// lift constant
#define B		1.140*10^(-7)			// drag constant
#define IM		3.357*10^(-5)			// inertia of the rotors (kg*m^2)
#define IX		4.856*10^(-3)			// inertia around the center (kg*m^2)
#define MAXY 	0.7 					// max output value of the y position controller (maximum angle phi)
#define MAXP 	1 						// max output value of the angle phi controller (maximum force u2)
#define MAXZ 	13 						// max output value of the z podition controller (maximum force u1)
#define MAXM 	12	 					// max output value of the motor controller (maximum applied voltage to the motor)
#define MINY 	-0.7					// min output value of the y position controller (minimum angle phi)
#define MINP 	-1						// min output value of the angle ohi controller (minimum force u2)
#define MINZ 	0 						// min output value of the z position controller (minimum force u1)
#define MINM 	0	 					// min output value of the motor controller (minimum applied voltage to the motor)
#define Q		0.022					// system noise covariance

/******************************************************************
 * STRUCTURES USED
******************************************************************/
struct 					status {		// quadcopter structure
	pthread_mutex_t 	m;
	int 				c;				// color [1,15]
	float				xc;				// x coordinate of the center
	float				yc;				// y coordinate of the center
	float 				xd;				// x coordinate of desired position
	float				yd;				// y coordinate of desired position
	float 				a;				// orientation angle (rad)
};

struct					motor {			// motor structure
	pthread_mutex_t 	m;
	float 				des_w; 			// desired angual velocity
	float 				w;				// real angular velocity
};

struct 					controller { 	// controllers structure
	pthread_mutex_t 	m;
	float 				forceu1;		// force u1 to be applied to the plants
	float 				forceu2;		// force u2 to be applied to the plants
};

struct 			kalman {
	float 		y_est_last, z_est_last, p_est_last; 	// previous state estimation
	float 		y_temp_est, z_temp_est, p_temp_est;		// predicted (a priori) state estimate
	float 		y_est, z_est, p_est; 					// updated (a posteriori) state estimate
	float 		P_last_y, P_last_z, P_last_p; 			// previous estimate covariance
	float 		P_temp_y, P_temp_z, P_temp_p; 			// predicted (a priori) estimate covariance
	float 		P_y, P_z, P_p;							// updated (a posteriori) estimate covariance
	float 		y_measured, z_measured, p_measured; 	// the noisy value measured
	float 		K_y, K_z, K_p;							// Kalman gain
};

struct			coord {			// coordinate structure
	float		x;				// x coordinate of the arrival point
	float		y;				// y coordinate of the arrival point
};

struct			est {			// coordinate estimation structure
	float		y;				// estimated y coordinate
	float		z;				// estimated z coordinate
	float		p; 				// estimated phi angle
};

/******************************************************************
 * GLOBAL VARIABLES
******************************************************************/
struct status 		quadcopter[MAXQ];		// quadcopter buffer
struct controller 	contr[MAXQ];			// a controller structure for each quadcopter
struct motor 		mot[MAXQ*2];			// two motor structures for each quadcopter
struct kalman 		kal[MAXQ]; 				// a kalman structure for each quadcopter
int 				nat = 0;				// number of active task
int 				naq = 0;				// number of active quad
int     			end = 0;                // end flag
int 				wflag = 1;				// wait flag
float 				G = 9.81;				// acceleration of gravity
float 				R = 1; 					// observation noise covariance
int 				c_period = 30;			// period of the controller task
int 				m_period = 15;			// period of the motor task
bool				kalman = true;			// kalman flag

/******************************************************************
 * GAUSSIAN_NOISE: generate a random numeber following the
 * AWGN model with given a mean and a standard deviation. The
 * method used is the Marsaglia polar form that is a pseudo-random
 * number sampling method for generating a pair of independent
 * standard normal random variables.
******************************************************************/
float gaussian_noise(float mean, float stdDev)
{
float 	u, v, s;

 	do {
 		u = (rand() / ((float) RAND_MAX)) * 2.0 - 1.0;
 		v = (rand() / ((float) RAND_MAX)) * 2.0 - 1.0;
		s = u * u + v * v;
	}
	while( (s >= 1.0) || (s == 0.0) );
	s = sqrt(-2.0 * log(s) / s);

	return mean + stdDev * u * s;
}

/******************************************************************
 * CALCULATE_STATE: given the measurement of the y and z position
 * and tha angle phi, the function adds some random noise and
 * then estimate the actual position using a Kalman filter.
******************************************************************/
struct est calculate_state (float y, float z, float p, int i)
{
struct est 	state;

	kal[i].y_temp_est = kal[i].y_est_last; 															// do a prediction of the current state based on the previous
	kal[i].P_temp_y = kal[i].P_last_y + Q;															// do a predition of the error covariance based on the previous
	kal[i].K_y = kal[i].P_temp_y * (1.0/(kal[i].P_temp_y + R)); 									// calculate the Kalman gain
	kal[i].y_measured = gaussian_noise(y, R); 														// the real measurement plus noise
	kal[i].y_est = kal[i].y_temp_est + kal[i].K_y * (kal[i].y_measured - kal[i].y_temp_est);		// correction of the actual state through the Kalman gain
	state.y = kal[i].y_est;																			// store the value of the estimated actual state
	kal[i].P_y = (1-kal[i].K_y) * kal[i].P_temp_y * (1-kal[i].K_y) + kal[i].K_y*R*kal[i].K_y;		// correction of the error covariance
	kal[i].P_last_y = kal[i].P_y;																	// sotring the error covariance for the next iteration
	kal[i].y_est_last = kal[i].y_est;																// sotring the estimated state for the next iteration

	kal[i].z_temp_est = kal[i].z_est_last; 															// do a prediction of the current state based on the previous
	kal[i].P_temp_z = kal[i].P_last_z + Q;															// do a predition of the error covariance based on the previous
	kal[i].K_z = kal[i].P_temp_z * (1.0/(kal[i].P_temp_z + R)); 									// calculate the Kalman gain
	kal[i].z_measured = gaussian_noise(z, R); 														// the real measurement plus noise
	kal[i].z_est = kal[i].z_temp_est + kal[i].K_z * (kal[i].z_measured - kal[i].z_temp_est);		// correction of the actual state through the Kalman gain
	state.z = kal[i].z_est;																			// store the value of the estimated actual state
	kal[i].P_z = (1-kal[i].K_z) * kal[i].P_temp_z * (1-kal[i].K_z) + kal[i].K_z*R*kal[i].K_z;		// correction of the error covariance
	kal[i].P_last_z = kal[i].P_z;																	// sotring the error covariance for the next iteration
	kal[i].z_est_last = kal[i].z_est;																// sotring the estimated state for the next iteration

	kal[i].p_temp_est = kal[i].p_est_last; 															// do a prediction of the current state based on the previous
	kal[i].P_temp_p = kal[i].P_last_p + Q;															// do a predition of the error covariance based on the previous
	kal[i].K_p = kal[i].P_temp_p * (1.0/(kal[i].P_temp_p + R/100)); 								// calculate the Kalman gain
	kal[i].p_measured = gaussian_noise(p, R/100); 													// the real measurement plus noise
	kal[i].p_est = kal[i].p_temp_est + kal[i].K_p * (kal[i].p_measured - kal[i].p_temp_est);		// correction of the actual state through the Kalman gain
	state.p = kal[i].p_est;																			// store the value of the estimated actual state
	kal[i].P_p = (1-kal[i].K_p) * kal[i].P_temp_p * (1-kal[i].K_p) + kal[i].K_p*R/100*kal[i].K_p;	// correction of the error covariance
	kal[i].P_last_p = kal[i].P_p;																	// sotring the error covariance for the next iteration
	kal[i].p_est_last = kal[i].p_est;																// sotring the estimated state for the next iteration

	if (kalman == false) {
		state.y = kal[i].y_measured;
		state.z = kal[i].z_measured;
		state.p = kal[i].p_measured;
	}

	return state;
}

/******************************************************************
 * GET_RAND_POS: get a random position in the relative quadcopter
 * virtual box, in a way that there aren't any overlapped positions
******************************************************************/
struct coord get_rand_pos(int i)
{
struct coord 	pos;
int 			x, y;

	x = rand()%50;
	y = rand()%50;
	if (i > 3 && i < 8)
		x = RBOX-3*QW-((7-i)*122)-x;
	else
		x = XBOX+3*QW+(i%4)*122+x;
	y = YBOX+2*QN+QW+15+(i/4)*115+y;
	pos.x = x;
	pos.y = y;
	return pos;
}

/******************************************************************
 * INIT: initialize the program, draw the areas
******************************************************************/
void init(void)
{
int 	i;
char    s[32];
char 	quad[32];
char 	des1[64];
char 	des2[64];
char 	opt1[64];
char 	opt2[64];
char 	opt3[64];
char 	opt4[64];
char 	task[32];
char 	gravity[16];
char 	noise[16];
char 	per_c[16];
char 	per_m[16];
char 	q[MAXT][16];

	srand(time(NULL));
    allegro_init();
    set_gfx_mode(GFX_AUTODETECT_WINDOWED, XWIN, YWIN, 0, 0);
    clear_to_color(screen, BKG);
    install_keyboard();
	install_mouse();
	enable_hardware_cursor();
	show_mouse(screen);

	rect(screen, 12, 12, 600, 100, 3);								// menu area
	rect(screen, 612, 12, 788, 588, 4);								// status area
	rect(screen, XBOX, YBOX, RBOX, BBOX, 2);						// box area
	// initialize the menù area
	sprintf(s, "Press SPACE to create a task");
    textout_ex(screen, font, s, 10, 10, 14, BKG);
	sprintf(quad, "QUADCOPTERS");
	textout_ex(screen, font, quad, 30, 30, 14, BKG);
	sprintf(des1, "To modify the status of the Kalman filter press K.");
	textout_ex(screen, font, des1, 30, 40, 14, BKG);
	sprintf(des2, "Kalman filter on ");
	textout_ex(screen, font, des2, 450, 40, 14, BKG);
	sprintf(opt1, "To modify the gravity press UP and DOWN.");
	textout_ex(screen, font, opt1, 30, 50, 14, BKG);
	sprintf(opt2, "To modify the noise press + and - on the numerical keypad.");
	textout_ex(screen, font, opt2, 30, 60, 14, BKG);
	sprintf(opt3, "To increase controllers and motors period press P, to decrease M");
	textout_ex(screen, font, opt3, 30, 70, 14, BKG);
	sprintf(opt4, "To restore the default configuration press R.");
	textout_ex(screen, font, opt4, 30, 80, 14, BKG);
	// initialize the status area
	sprintf(task, "Task running: %d", nat+1);
	textout_ex(screen, font, task, 630, 25, 14, BKG);
	sprintf(gravity, "Gravity: %.2f", G);
	textout_ex(screen, font, gravity, 630, 35, 14, BKG);
	sprintf(noise, "Noise: %.2f", R);
	textout_ex(screen, font, noise, 630, 45, 14, BKG);
	sprintf(per_c, "t_c: %d ", c_period);
	textout_ex(screen, font, per_c, 630, 55, 14, BKG);
	sprintf(per_m, "t_m: %d ", m_period);
	textout_ex(screen, font, per_m, 710, 55, 14, BKG);
	for (i=0; i<MAXQ; i++) {
		sprintf (q[i], "Quadcopter %d", i+1);
		textout_ex(screen, font, q[i], 630, 70+i*43, 4, BKG);
	}
}

/******************************************************************
 * INIT_FLY: initialize the quadcopter with the relative
 * structures needed
******************************************************************/
void init_fly(int i, struct coord pos)
{
char 	q[16];

	quadcopter[i].c = i+1;
	quadcopter[i].xc = XCEN;
	quadcopter[i].yc = YCEN;
	quadcopter[i].xd = pos.x;
	quadcopter[i].yd = pos.y;
	quadcopter[i].a = 0;
	// motor initialization
	mot[i*2].des_w = 0;
	mot[i*2].w = 0;
	mot[i*2+1].des_w = 0;
	mot[i*2+1].w = 0;
	// making the relative 'Quadcopter' indicator green
	sprintf (q, "Quadcopter %d", i+1);
	textout_ex(screen, font, q, 630, 70+i*43, 2, BKG);
}

/******************************************************************
 * INIT_CONTROLLER: initialize the quadcopter controller's and
 * kalman's structures
******************************************************************/
void init_controller(int i)
{
	// controller initialization
	contr[i].forceu1 = 0;
	contr[i].forceu2 = 0;
	// kalman structure initialization
	kal[i].y_est_last = XCEN;
	kal[i].z_est_last = YCEN;
	kal[i].p_est_last = 0;
	kal[i].y_temp_est = 0;
	kal[i].z_temp_est = 0;
	kal[i].p_temp_est = 0;
	kal[i].y_est = 0;
	kal[i].z_est = 0;
	kal[i].p_est = 0;
	kal[i].P_last_y = 0;
	kal[i].P_last_z = 0;
	kal[i].P_last_p = 0;
	kal[i].P_temp_y = 0;
	kal[i].P_temp_z = 0;
	kal[i].P_temp_p = 0;
	kal[i].P_y = 0;
	kal[i].P_z = 0;
	kal[i].P_p = 0;
	kal[i].y_measured = 0;
	kal[i].z_measured = 0;
	kal[i].p_measured = 0;
	kal[i].K_y = 0;
	kal[i].K_z = 0;
	kal[i].K_p = 0;
}

void draw_quadcopter(int i)
{
float 	p1x, p2x, p3x, p1y, p2y, p3y;
float 	ca, sa;
float 	angle;
float 	x, y, a, fu1, fu2;
int 	color;
char 	status[3][32];

	pthread_mutex_lock(&quadcopter[i].m);
	x = quadcopter[i].xc;
	y = quadcopter[i].yc;
	a = quadcopter[i].a;
	color = quadcopter[i].c;
	pthread_mutex_unlock(&quadcopter[i].m);

	pthread_mutex_lock(&contr[i].m);
	fu1 = contr[i].forceu1;
	fu2 = contr[i].forceu2;
	pthread_mutex_unlock(&contr[i].m);
	if(y < YCEN-1) {
		angle = -a;
		ca = cos(angle);
		sa = sin(angle);

		p1x = x + QN*sa; 			// nose point
		p1y = y + QN*ca;

		p2x = x - QW*ca; 			// left wing
		p2y = y + QW*sa;

		p3x = x + QW*ca; 			// right wing
		p3y = y - QW*sa;

		if (p1x<XBOX+1 || p2x<XBOX+1 || p3x<XBOX+1 || p1x>RBOX-1 || p2x>RBOX-1 || p3x>RBOX-1 || p1y<YBOX+1 || p2y<YBOX+1 || p3y<YBOX+1)
			;
		else
			triangle(screen, p1x, p1y, p2x, p2y, p3x, p3y, color);
	}
	else
		triangle(screen, x-QW, YCEN, x+QW, YCEN, x, YCEN+QN-1, i+1);

	sprintf(status[0], "Angle: %.2f°     ", angle/PI*180);
	sprintf(status[1], "Force u1: %.2f   ", fu1);
	sprintf(status[2], "Force u2: %.2f   ", fu2);
	textout_ex(screen, font, status[0], 630, 70+i*43+10, 14, BKG);
	textout_ex(screen, font, status[1], 630, 70+i*43+20, 14, BKG);
	textout_ex(screen, font, status[2], 630, 70+i*43+30, 14, BKG);
}

/******************************************************************
 * QUADTASK: thread associated to a quadcopter
******************************************************************/
void *quadtask(void *arg)
{
int 				i;				// task index
int 				m1, m2; 		// motor indexes
int 				j, res;
struct task_par		*tp;
struct coord 		des_pos;
float 				out_p[3], out_y[3], out_z[3];
float 				in_p[3], in_y[3], in_z[3];
float 				fu1, fu2, w1, w2;

	tp = (struct task_par *)malloc(sizeof(struct task_par));
	tp = (struct task_par *)arg;
	i = (tp->arg/4)-1;
	m1 = i*2;
	m2 = i*2+1;
	pthread_mutex_init(&quadcopter[i].m, NULL);
	res = sched_setattr(gettid(), &(tp->attr), 0);	// set scheduling policy and attributes
	if (res < 0) {
		perror("sched_setattr()");
		return NULL;
	}
	// initialization
	for (j=0; j<3; j++) {
		out_p[j] = 0;
		out_y[j] = 0;
		out_z[j] = 0;
		in_p[j] = 0;
		in_y[j] = 0;
		in_z[j] = 0;
	}
	triangle(screen, XCEN-QW, YCEN, XCEN+QW, YCEN, XCEN, YCEN+QN-1, i+1);
	des_pos = get_rand_pos(i);
	init_fly(i, des_pos);
	set_period(tp);
	while (!end) {
		pthread_mutex_lock(&contr[i].m);
		fu1 = contr[i].forceu1;
		fu2 = contr[i].forceu2;
		pthread_mutex_unlock(&contr[i].m);

		pthread_mutex_lock(&mot[m1].m);
		mot[m1].des_w = sqrt(abs((L*fu1 - fu2)/(2*K*L)));
		w1 = mot[m1].w;
		pthread_mutex_unlock(&mot[m1].m);

		pthread_mutex_lock(&mot[m2].m);
		mot[m2].des_w = sqrt(abs((L*fu1 + fu2)/(2*K*L)));
		w2 = mot[m2].w;
		pthread_mutex_unlock(&mot[m2].m);

		fu1 = (w1*w1 + w2*w2) * K;
		if(quadcopter[i].yc < YCEN-1) {
			// angle and y position update
			fu2 = (w2*w2 - w1*w1) * K * L;
			in_p[0] = fu2;
			out_p[0] = 2*out_p[1] - out_p[2] + 0.06*in_p[1] + 0.06*in_p[2]; 				// model that generates the angle phi from force u2

			in_y[0] = out_p[0];																// input of the plant equal to the output of the previous plant
			out_y[0] = 2*out_y[1] - out_y[2] + 0.004414*in_y[1] + 0.004414*in_y[2]; 		// model that generates y from angle phi

			pthread_mutex_lock(&quadcopter[i].m);
			quadcopter[i].a = out_p[0];
			quadcopter[i].xc = XCEN + out_y[0]*YBOX;
			pthread_mutex_unlock(&quadcopter[i].m);
		}
		// z position update
		in_z[0] = fu1 - G;
		out_z[0] = 2*out_z[1] - out_z[2] + 0.0006429*in_z[1] + 0.0006429*in_z[2]; 			// model that generates z from force u1

		pthread_mutex_lock(&quadcopter[i].m);
		quadcopter[i].yc = YCEN - out_z[0]*YBOX;
		pthread_mutex_unlock(&quadcopter[i].m);

		for(j=0; j<2; j++) {
			out_p[j+1] = out_p[j];
			out_y[j+1] = out_y[j];
			out_z[j+1] = out_z[j];
			in_p[j+1] = in_p[j];
			in_y[j+1] = in_y[j];
			in_z[j+1] = in_z[j];
		}
		if(deadline_miss(tp)) {
			printf("%d DEADLINE MISS QUAD TASK %d\n", tp->dmiss, i);
		}
		wait_for_period(tp);
	}
	printf("TASK %d (QUAD) DEADLINE MISSES: %d\n", tp->arg, tp->dmiss);
	fflush(stdout);
	return NULL;
}

/******************************************************************
 * CONTROLLERTASK: thread associated to a quadcopter which compute
 * the Kalman filter and 3 controllers needed for position and
 * attitude
******************************************************************/
void *controllertask (void *arg)
{
struct task_par		*tp;
int 				i, j, res;
float 				actual_p, actual_y, actual_z;
float 				desired_p, desired_y, desired_z;
float 				out_c_p[3], out_c_y[3], out_c_z[3];
float 				in_c_p[3], in_c_y[3], in_c_z[3];
struct est 			state;
float 				x, y, a, x_d, y_d, fu1, fu2;

	tp = (struct task_par *)malloc(sizeof(struct task_par));
	tp = (struct task_par *)arg;
	j = (tp->arg/4)-1;
	pthread_mutex_init(&contr[j].m, NULL);

	res = sched_setattr(gettid(), &(tp->attr), 0);	// set scheduling policy and attributes
	if (res < 0) {
		perror("sched_setattr()");
		return NULL;
	}
	init_controller(j);
	for (i=0; i<3; i++) {
		in_c_y[i] = 0;
		in_c_p[i] = 0;
		in_c_z[i] = 0;
		out_c_y[i] = 0;
		out_c_p[i] = 0;
		out_c_z[i] = 0;
	}
	set_period(tp);
	while (!end) {
		tp->period = c_period;
		tp->deadline = c_period;

		pthread_mutex_lock(&quadcopter[j].m);
		x = quadcopter[j].xc;
		y = quadcopter[j].yc;
		a = quadcopter[j].a;
		x_d = quadcopter[j].xd;
		y_d = quadcopter[j].yd;
		pthread_mutex_unlock(&quadcopter[j].m);

		state = calculate_state(x, y, a, j);
		actual_y = state.y/YBOX;
		actual_z = state.z/YBOX;
		actual_p = state.p;
		desired_y = x_d/YBOX;
		desired_z = y_d/YBOX;

		if(y < YCEN-1) {														// compute the controller only if the quadcopter is in the box
			in_c_y[0] = desired_y - actual_y; 									// compute the input of the controller (error of y position)
			out_c_y[0] = 0.8078*out_c_y[1] + 1.3877*in_c_y[0]; 					// compute the output of the controller (angle phi desired)
			if (out_c_y[0] > MAXY)												// checking the value for saturation
				out_c_y[0] = MAXY;
			if (out_c_y[0] < MINY)
				out_c_y[0] = MINY;

			desired_p = out_c_y[0];
			in_c_p[0] = desired_p - actual_p; 									// compute the input of the controller (error of phi angle)
			out_c_p[0] = 0.2*out_c_p[1] + 1.4251*in_c_p[0]; 					// compute the output of the controller (force u2 desired)
			if (out_c_p[0] > MAXP)												// checking the value for saturation
				out_c_p[0] = MAXP;
			if (out_c_p[0] < MINP)
				out_c_p[0] = MINP;
			fu2 = out_c_p[0];													// force u2 to be applied to the plant
		}

		in_c_z[0] = -(desired_z - actual_z); 									// compute the input of the controller (error of z position)
		// compute the output of the controller (force u1 desired)
		out_c_z[0] = 1.6857*out_c_z[1] - 0.6857*out_c_z[2] + 49*in_c_z[0] - 95.1*in_c_z[1] + 46.76*in_c_z[2];
		if (out_c_z[0] > MAXZ)													// checking the value for saturation
			out_c_z[0] = MAXZ;
		if (out_c_z[0] < MINZ)
			out_c_z[0] = MINZ;
		fu1 = out_c_z[0];														// force u1 to be applied to the plant

		pthread_mutex_lock(&contr[j].m);
		contr[j].forceu1 = fu1;
		if(y < YCEN-1)
			contr[j].forceu2 = fu2;
		pthread_mutex_unlock(&contr[j].m);


		for (i=0; i<2; i++) {
			in_c_y[i+1] = in_c_y[i];
			in_c_p[i+1] = in_c_p[i];
			in_c_z[i+1] = in_c_z[i];
			out_c_y[i+1] = out_c_y[i];
			out_c_p[i+1] = out_c_p[i];
			out_c_z[i+1] = out_c_z[i];
		}
		if(deadline_miss(tp)) {
			printf("%d DEADLINE MISS CONTROLLER TASK %d\n", tp->dmiss, i);
		}
		wait_for_period(tp);
	}
	printf("TASK %d (CONTROLLER) DEADLINE MISSES: %d\n", tp->arg, tp->dmiss);
	fflush(stdout);
	return NULL;
}

/******************************************************************
 * DRAWTASK: thread associated to the graphic task
******************************************************************/
void *drawtask(void *arg)
{
int 				i, n, res;
struct task_par		*tp;

	tp = (struct task_par *)malloc(sizeof(struct task_par));
	tp = (struct task_par *)arg;
	i = tp->arg;

	res = sched_setattr(gettid(), &(tp->attr), 0);	// set scheduling policy and attributes
	if (res < 0) {
		perror("sched_setattr()");
		return NULL;
	}
	while(wflag);		// until a quadcopter is created, it won't start
	set_period(tp);
	while (!end) {
		rectfill(screen, XBOX+1, YBOX+1, RBOX-1, BBOX-1, BKG);			// fill the box area with black to redraw the quadcopters
		for(n=0; n<naq; n++) {
			draw_quadcopter(n);
			rectfill(screen, quadcopter[n].xd-2, quadcopter[n].yd-2, quadcopter[n].xd+2, quadcopter[n].yd+2, quadcopter[n].c);
		}
		if(deadline_miss(tp)) {
			printf("%d DEADLINE MISS GRAPHIC TASK %d\n", tp->dmiss, i);
		}
		wait_for_period(tp);

	}
	printf("TASK %d (GRAPHIC) DEADLINE MISSES: %d\n", tp->arg, tp->dmiss);
	fflush(stdout);
	return NULL;
}

/******************************************************************
 * STATUSTASK: thread associated to the system parameters
******************************************************************/
void *statustask(void *arg)
{
struct task_par		*tp;
char 				task[32];
char 				gravity[16];
char 				noise[16];
char 				per_c[16];
char 				per_m[16];
char 				des2[16];
int 				i, res;

	tp = (struct task_par *)malloc(sizeof(struct task_par));
	tp = (struct task_par *)arg;
	i = tp->arg;

	res = sched_setattr(gettid(), &(tp->attr), 0);	// set scheduling policy and attributes
	if (res < 0) {
		perror("sched_setattr()");
		return NULL;
	}
	set_period(tp);
	while (!end) {
		rect(screen, 612, 12, 788, 588, 4);								// redraw status area
		rect(screen, XBOX, YBOX, RBOX, BBOX, 2);						// redraw box area
		sprintf(task, "Task running: %d", nat+1);
		textout_ex(screen, font, task, 630, 25, 14, BKG);
		sprintf(gravity, "Gravity: %.2f  ", G);
		textout_ex(screen, font, gravity, 630, 35, 14, BKG);
		sprintf(noise, "Noise: %.2f", R);
		textout_ex(screen, font, noise, 630, 45, 14, BKG);
		sprintf(per_c, "t_c: %d ", c_period);
		textout_ex(screen, font, per_c, 630, 55, 14, BKG);
		sprintf(per_m, "t_m: %d ", m_period);
		textout_ex(screen, font, per_m, 710, 55, 14, BKG);
		sprintf(des2, "Kalman filter on ");
		if (kalman == true)
			textout_ex(screen, font, des2, 450, 40, 2, BKG);
		else
			textout_ex(screen, font, des2, 450, 40, 4, BKG);

		if(deadline_miss(tp)){
			printf("%d DEADLINE MISS STATUS TASK %d\n", tp->dmiss, i);
		}
		wait_for_period(tp);
	}
	printf("TASK %d (STATUS) DEADLINE MISSES: %d\n", tp->arg, tp->dmiss);
	fflush(stdout);
	return NULL;
}

/******************************************************************
 * MOTORTASK: thread associated to a motor
******************************************************************/
void *motortask(void *arg)
{
float 				in_c[3], out_c[3], out[3];
struct task_par		*tp;
int 				i = 0, res, j;

	tp = (struct task_par *)malloc(sizeof(struct task_par));
	tp = (struct task_par *)arg;
	i = ((tp->arg+1)/4) + (tp->arg/4);
	// initialization
	for(j=0; j<3; j++) {
		in_c[j] = 0;
		out_c[j] = 0;
		out[j] = 0;
	}
	pthread_mutex_init(&mot[i].m, NULL);

	res = sched_setattr(gettid(), &(tp->attr), 0);						// set scheduling policy and attributes
	if (res < 0) {
		perror("sched_setattr()");
		return NULL;
	}
	set_period(tp);
	while(!end) {
		tp->period = m_period;
		tp->deadline = m_period;

		pthread_mutex_lock(&mot[i].m);
		in_c[0] = mot[i].des_w - out[0];
		pthread_mutex_unlock(&mot[i].m);

		out_c[0] = out_c[1] + 0.009*in_c[1];							// digital controller
		if (out_c[0] > MAXM)											// checking the value for saturation
			out_c[0] = MAXM;
		if (out_c[0] < MINM)
			out_c[0] = MINM;
		out[0] = 0.3896*out[1] + 19.6*out_c[1] + 0.3821*out_c[2];		// plant of the motor from voltage to angular velocity

		pthread_mutex_lock(&mot[i].m);
		mot[i].w = out[0];
		pthread_mutex_unlock(&mot[i].m);

		for(j=0; j<2; j++) {
			in_c[j+1] = in_c[j];
			out_c[j+1] = out_c[j];
			out[j+1] = out[j];
		}
		if(deadline_miss(tp)){
			printf("%d DEADLINE MISS MOTOR TASK %d\n", tp->dmiss, i);
		}
		wait_for_period(tp);
	}
	printf("TASK %d (MOTOR) DEADLINE MISSES: %d\n", tp->arg, tp->dmiss);
	fflush(stdout);
	return NULL;
}

int main(void)
{
int     	res = 0, n;
char    	scan;
pthread_t	th[MAXT];
int 		x_m, y_m, x2_m, y2_m, exit_mouse = 0;

    init();

#ifdef EDF
	th[nat] = task_create(nat, drawtask, SCHED_DEADLINE, 0, 0, 0, RUNG, PERG, DEAG);
	nat++;
	th[nat] = task_create(nat, statustask, SCHED_DEADLINE, 0, 0, 0, RUNS, PERS, DEAS);
	nat++;
#endif
#ifdef FIFO
	th[nat] = task_create(nat, drawtask, SCHED_FIFO, 0, 0, 1, RUNG, PERG, DEAG);
	nat++;
	th[nat] = task_create(nat, statustask, SCHED_FIFO, 0, 0, 1, RUNS, PERS, DEAS);
	nat++;
#endif

	do {
		scan = 0;

		if (mouse_b & 1) {
			x_m = mouse_x;
			y_m = mouse_y;
			for (n=0; n<naq; n++) {
				if (x_m >= quadcopter[n].xd-2 && x_m <= quadcopter[n].xd+2)
					if (y_m >= quadcopter[n].yd-2 && y_m <= quadcopter[n].yd+2) {
						do {
							exit_mouse = 0;
							scan = 0;
							if (keypressed())
					            scan = readkey() >> 8;
							if (mouse_b & 1) {
								x2_m = mouse_x;
								y2_m = mouse_y;
								if (x2_m > XBOX+2*QW && x2_m < RBOX-2*QW)
									if (y2_m > YBOX+2*QN && y2_m < BBOX-3*QN) {
										quadcopter[n].xd = x2_m;
										quadcopter[n].yd = y2_m;
										exit_mouse = 1;
									}
							}
						} while (scan != KEY_ESC && exit_mouse);
					}
			}

		}
        if (keypressed())
            scan = readkey() >> 8;

        if (scan == KEY_SPACE && nat < MAXT) {
			wflag = 0;
#ifdef EDF
			th[nat] = task_create(nat, motortask, SCHED_DEADLINE, 0, 0, 0, RUNM, PERM, DEAM);
			nat++;
			th[nat] = task_create(nat, motortask, SCHED_DEADLINE, 0, 0, 0, RUNM, PERM, DEAM);
			nat++;
			th[nat] = task_create(nat, quadtask, SCHED_DEADLINE, 0, 0, 0, RUNQ, PERQ, DEAQ);
			nat++;
			th[nat] = task_create(nat, controllertask, SCHED_DEADLINE, 0, 0, 0, RUNC, PERC, DEAC);
			nat++;
#endif
#ifdef FIFO
			th[nat] = task_create(nat, motortask, SCHED_FIFO, 0, 0, 1, RUNM, PERM, DEAM);
			nat++;
			th[nat] = task_create(nat, motortask, SCHED_FIFO, 0, 0, 1, RUNM, PERM, DEAM);
			nat++;
			th[nat] = task_create(nat, quadtask, SCHED_FIFO, 0, 0, 1, RUNQ, PERQ, DEAQ);
			nat++;
			th[nat] = task_create(nat, controllertask, SCHED_FIFO, 0, 0, 1, RUNC, PERC, DEAC);
			nat++;
#endif
			naq++;
        }
		if (scan == KEY_UP && G < 12)
			G++;
		if (scan == KEY_DOWN && G > 3)
			G--;
		if (scan == KEY_P) {
			c_period += 2;
			m_period ++;
		}
		if (scan == KEY_M && c_period > PERC && m_period >PERM) {
			c_period -= 2;
			m_period --;
		}
		if (scan == KEY_PLUS_PAD && R < 31)
			R++;
		if (scan == KEY_MINUS_PAD && R > 0)
			R--;
		if (scan == KEY_K) {
			if (kalman == true)
				kalman = false;
			else
				kalman = true;
		}
		if (scan == KEY_R) {
			G = 9.81;
			R = 1;
			c_period = PERC;
			m_period = PERM;
			kalman = true;
		}
    } while (scan != KEY_ESC);

	// closing procedure of the application
	end = 1;
	wflag = 0;
	for (n=0; n<nat; n++) {
		res = pthread_join(th[n], NULL);
		if (res != 0)
			perror("pthread_join()");
	}
    allegro_exit();

    return 0;
}
