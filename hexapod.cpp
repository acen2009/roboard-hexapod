#include <stdio.h>
#include <conio.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <allegro.h>
#define  USE_COMMON
#include <roboard.h>


#define RAD_TO_DEG   (57.2957795147199)     // radian to degree
#define DEG_TO_RAD   (0.01745329251944)     // degree to radian
//#define PI      				 (3.14159265)		      

 #define EXIT					-1
 #define IDLE					0x00
 #define CONVOLUTE  	0x10
 #define FORWARD		0x01 
 #define BACKWARD	0x02 
 #define LEFTWARD		0x03
 #define RIGHTWARD	0x04
 #define FLWARD			0x05
 #define FRWARD			0x06
 #define BLWARD			0x07
 #define BRWARD			0x08
 #define LCIRCLE			0x09
 #define RCIRCLE			0x0a
 #define BALANCE		0x0b


 #define POS_550		(550L)
 #define POS_400		(400L)
 #define POS_300		(300L)
 #define POS_250		(250L)
 #define POS_200		(200L)
 #define POS_150		(150L)
 #define POS_100		(100L)
  #define POS_50		(50L)

#define IDLETIME		(4000L)
#define SLOWEST		(1000L)
#define SLOWER		(500L)
#define SLOW				(400L)
#define GENERAL		(300L)
#define FAST				(150L)
#define FASTER			(100L)
#define FASTEST		(50L)

unsigned long POSITION[32] = {
	1500L, 1500L, 1500L, //left mid
    0L,	
    1500L, 1500L, 1500L, //left hind
	0, 0, 
	1500L, 1500L, 1500L, //right hind 
	0L,	
	1500l, 1500L, 1500L, //right mid
	1500L, 1500L, 1500L, //left front
    0L, 0L, 	
	1500L, 1500L, 1500L, // right front
	0L, 0L, 0L, 0L, 0L, 0L, 0l, 0L
	};

unsigned long NORMAL[32] = {
	1480L, 1500L, 1500L, //left mid
    0L,	
    1400L, 1500L, 1500L, //left hind
	0, 0, 
	1560L, 1500L, 1500L, //right hind 
	0L,	
	1480l, 1500L, 1500L, //right mid
	1500L, 1500L, 1500L, //left front
    0L, 0L, 	
	1450L, 1500L, 1500L, // right front
	0L, 0L, 0L, 0L, 0L, 0L, 0l, 0L
	};	

unsigned long STANDBY[32] = {
	1500L, 1800L, 1500L, //left mid
    0L,	
    1500L, 1800L, 1500L, //left hind
	0L, 0L, 
	1500L, 1200L, 1500L, //right hind 
	0L,	
	1500L, 1200L, 1500L, //right mid
	1500L, 1800L, 1500L, //left front
    0L, 0L, 	
	1500L, 1200L, 1500L, // right front
	0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L
	};
	
int LEGMAP[6] = {16, 0, 4, 21, 13, 9};	
double J3MAP[6] = {PI*2/3, PI, -PI*2/3, PI/3, 0.0 , -PI/3};
double INITMAP[6] = {PI*2/3, PI, -PI*2/3, PI/3, 0.0 , -PI/3};

double OFFSET_3 = 60.0, LENGTH = 120.0 ,OFFSET_3Z = -32.0;
double OFFSET_X2 = 12.0, OFFSET_Y2 = -11.0, OFFSET_Z2 = 25.0;
//double L1 = 57.0, L2 = 45.0;
double L2 = 33.0 , L2_Z = 4.0, L3_X = 12.0, L3_Z = 8.0, L1 = 57.0;


int LEGFRAME[6] = {0}, IDLEFRAME = 0;
double LEGORIGIN[18] = {0.0}, LEGGOAL[18] = {0.0}, LEGOPPOSITE[18] = {0.0};
double GOAL[3], J3GOAL[3], J2GOAL[3], LEGPOS[3], JANGLE[3];
int STATE, PRE_STATE = EXIT;


double GAIN[3];
double GOALPOS[9];
double GOAL_YAW = 0.0, GOAL_PITCH = 0.0, GOAL_ROLL = 0.0;
double ROTATE = 0.0;
double BODY_YAW = 0.0, BODY_PITCH = 0.0, BODY_ROLL = 0.0;
double ALPHA = 0.0;
int SWING = 0;
double DISTANCE1, DISTANCE2, DISTANCE3, DISTANCE4;
double gain_p = 8.0, gain_r = 8.0;
int STEP_1, STEP_2;

void SetLegPos(double* cpos, int num, double *lpos)
{
	lpos[0] = LENGTH*cos(INITMAP[num]) + cpos[0];
	lpos[1] = LENGTH*sin(INITMAP[num]) + cpos[1];
	lpos[2] = -cpos[2] + OFFSET_3Z;
}

void GetLegOrigin(double *coord)
{
	int i;
	double p[3] = {0.0, 0.0, 0.0};
	for(i = 0; i < 6; i++)
	{
		SetLegPos(p, i,coord+i*3);
	}
}
void CenterToJoint3(double *cpos, int num, double *j3pos);

void GetLegGoal(double *pos, int num, double *coord)
{
	int map = num%3;
	SetLegPos(pos+3*map, num,coord+num*3);
}

void GetLegOpposite(double *pos, int num, int state, double *coord)
{
	double p[3];
	int map = num%3;
	
	switch(state)
	{
			case FORWARD:
			case BACKWARD:
				p[0] = pos[map*3] ;
				p[1] = -pos[map*3+1];
				p[2] = pos[map*3+2];
				
				SetLegPos(p, num,coord+num*3);
			break;
			case RIGHTWARD:
			case LEFTWARD:
				p[0] = -pos[map*3] ;
				p[1] = pos[map*3+1];
				p[2] = pos[map*3+2];
				SetLegPos(p, num,coord+num*3);
			break;
			case FLWARD:
			case FRWARD:
			case BLWARD:
			case BRWARD:
				p[0] = -pos[map*3];
				p[1] = -pos[map*3+1];
				p[2] = pos[map*3+2];
				SetLegPos(p, num,coord+num*3);
				break;
			case IDLE:	
			case LCIRCLE:
			case RCIRCLE:
				p[0] = pos[map*3];
				p[1] = pos[map*3+1];
				p[2] = pos[map*3+2];
				SetLegPos(p, num,coord+num*3);
				break;
	}

}

#define DOWN	5.0
#define FRAME	12
double RISE = 50.0; 


void GetLegMotion(double *opposite, double *goal, int num, int step, double *lpos)
{
	double gain = GAIN[num%3];
	if(step == 0)
	{
		lpos[0] = (opposite[3*num] + goal[3*num])*0.5;
		lpos[1] = (opposite[3*num+1] + goal[3*num+1])*0.5;
		lpos[2] = (opposite[3*num+2] + goal[3*num+2])*0.5 + RISE;
	}		
	else
	{
		lpos[0] = goal[3*num] + gain*(opposite[3*num] - goal[3*num])*(step-1)/(FRAME - 2);
		lpos[1] = goal[3*num+1] + gain*(opposite[3*num+1] - goal[3*num+1])*(step-1)/(FRAME - 2);
		lpos[2] = goal[3*num+2] + (opposite[3*num+2] - goal[3*num+2])*(step-1)/(FRAME - 2) - DOWN;
	}
	
}

void CenterToJoint3(double *cpos, int num, double *j3pos)
{
	double t, p_x, p_y;
	p_x = cpos[0] - OFFSET_3*cos(J3MAP[num]);
	p_y = cpos[1] - OFFSET_3*sin(J3MAP[num]);
	t = atan2(p_y,p_x);
	j3pos[0] = p_x*cos(t) + p_y*sin(t);
	j3pos[1] = -p_x*sin(t) + p_y*cos(t);
	j3pos[2] = cpos[2];
	
}

void Joint3ToJoint2(double *j3pos, double *j2pos)
{
	j2pos[0] = j3pos[0] - OFFSET_X2;
	j2pos[1] = j3pos[1] - OFFSET_Y2;
	j2pos[2] = j3pos[2] - OFFSET_Z2;
	
}	

void JointAngle(double *cpos, double *j2pos, int num, double *jangle)
{
	double a, b, c, temp1,temp2, temp3, root;
	a = -2.0*L2*(j2pos[2] + L3_Z);
	b = 2.0*(j2pos[0] - L3_X)*L2;
	c = L1*L1 - L2*L2 - (j2pos[0] - L3_X)*(j2pos[0] - L3_X) - (j2pos[2]+L3_Z)*(j2pos[2]+L3_Z);
	temp1 = a*c;
	temp2 = sqrt(a*a*c*c - (a*a+b*b)*(c*c-b*b));
	temp3 = a*a+b*b;
	//T2

	root = (temp1+ temp2)/temp3;

	jangle[1] = asin(root)*RAD_TO_DEG;
	//T1
	temp1 = 1.0 - root*root;
	temp1 = sqrt(temp1);
	root = (j2pos[0]-L3_X-L2*temp1)/L1;

	jangle[2] = asin(root)*RAD_TO_DEG;
	//T3
	double p_x, p_y;
	p_x = cpos[0] - OFFSET_3*cos(J3MAP[num]);
	p_y = cpos[1] - OFFSET_3*sin(J3MAP[num]);
	jangle[0] = atan2(p_y,p_x) - INITMAP[num];
	jangle[0] = jangle[0]*RAD_TO_DEG;
	if(jangle[0] > 180.0)
		jangle[0] = jangle[0] - 360.0;
	else if (jangle[0] < -180.0)
		jangle[0] = jangle[0] + 360.0;

}

void SetPosition(long offset, int map, unsigned long *norm, unsigned long *pos)
{
	pos[map] = norm[map] + offset;
}

void SetJoint(double *jangle, int num)
{
	long offset;
	if(num < 3)
	{
		offset = -(long)(jangle[0]*10.0);
		SetPosition(offset,LEGMAP[num],NORMAL,POSITION);
		offset = (long)(jangle[1]*10.0);
		SetPosition(offset,LEGMAP[num]+1,NORMAL,POSITION);
		offset = -(long)((jangle[2])*10.0);
		SetPosition(offset,LEGMAP[num]+2,NORMAL,POSITION);
	}	
	else
	{
		offset = -(long)(jangle[0]*10.0);
		SetPosition(offset,LEGMAP[num],NORMAL,POSITION);
		offset = -(long)(jangle[1]*10.0);
		SetPosition(offset,LEGMAP[num]+1,NORMAL,POSITION);
		offset = (long)((jangle[2])*10.0);
		SetPosition(offset,LEGMAP[num]+2,NORMAL,POSITION);
	}
}

void SetOrigin(unsigned long *initm, unsigned long *pos)
{
	for(int i = 0; i < 32; i++)
		pos[i] = initm[i];
}

void Transform(double y, double p, double r, double *pos)
{
	double p_x, p_y, p_z;
	 p_x = pos[0]; 
	 p_y = pos[1];
	 p_z = pos[2];
	pos[0] = cos(y)*cos(p)*p_x + (-sin(y)*cos(r) + cos(y)*sin(p)*sin(r))*p_y +  (sin(y)*sin(r) + cos(y)*sin(p)*cos(r))*p_z;
	pos[1] = sin(y)*cos(p)*p_x + (cos(y)*cos(r) + sin(y)*sin(p)*sin(r))*p_y  +  (-cos(y)*sin(r) + sin(y)*sin(p)*cos(r))*p_z;
	pos[2] = -sin(p)*p_x           +  cos(p)*sin(r)*p_y                                       +  cos(p)*cos(r)*p_z;
}

unsigned long IDLEPLAYTIME =SLOW , PLAYTIME = GENERAL, STARTTIME[6];
unsigned long LEGTIMER[6], PLAYTIMER[6], IDLETIMER;
double ADJUST = 0.0;
void Update(void)
{
	switch(STATE)
	{
		case FORWARD:
			// front
			GAIN[0] = 1.2;	GOALPOS[0] = 0.0;	GOALPOS[1] = DISTANCE1 + ADJUST;		GOALPOS[2] = 0.0;
			//mid
			GAIN[1] = 1.0;	GOALPOS[3] = 0.0;	GOALPOS[4] = DISTANCE2 + ADJUST;		GOALPOS[5] = 0.0;
			// hint
			GAIN[2] = 1.2;	GOALPOS[6] = 0.0;	GOALPOS[7] = DISTANCE3 + ADJUST;		GOALPOS[8] = 0.0;
			GOAL_YAW = 0.0; 		GOAL_PITCH = 0.0;				GOAL_ROLL = 0.0;  
			ROTATE = 0.0;
			break;
		case LEFTWARD:
			// front
			GAIN[0] = 1.2;	GOALPOS[0] = -DISTANCE4- ADJUST;	GOALPOS[1] = 0.0;	GOALPOS[2] = 0.0;
			//mid
			GAIN[1] = 1.2;	GOALPOS[3] = -DISTANCE4- ADJUST;	GOALPOS[4] = 0.0;	GOALPOS[5] = 0.0;
			// hint
			GAIN[2] = 1.2;	GOALPOS[6] = -DISTANCE4- ADJUST;	GOALPOS[7] = 0.0;	GOALPOS[8] = 0.0;
			GOAL_YAW = 0.0; 		GOAL_PITCH = 0.0;				GOAL_ROLL = 0.0;
			ROTATE = 0.0;
			break;
		case RIGHTWARD:
				// front
			GAIN[0] = 1.2;	GOALPOS[0] = DISTANCE4 + ADJUST;		GOALPOS[1] = 0.0;	GOALPOS[2] = 0.0;
			//mid
			GAIN[1] = 1.2;	GOALPOS[3] = DISTANCE4 + ADJUST;		GOALPOS[4] = 0.0;	GOALPOS[5] = 0.0;
			// hint
			GAIN[2] = 1.2;	GOALPOS[6] = DISTANCE4 + ADJUST;		GOALPOS[7] = 0.0;	GOALPOS[8] = 0.0;
			GOAL_YAW = 0.0; 		GOAL_PITCH = 0.0;				GOAL_ROLL = 0.0;
			ROTATE = 0.0;
			break;			
		case BACKWARD:
			// front
			GAIN[0] = 1.2;	GOALPOS[0] = 0.0;		GOALPOS[1] = -DISTANCE3 - ADJUST;	GOALPOS[2] = 0.0;
			//mid
			GAIN[1] = 1.0;	GOALPOS[3] = 0.0;		GOALPOS[4] = -DISTANCE2 - ADJUST;	GOALPOS[5] = 0.0;
			// hint
			GAIN[2] = 1.2;	GOALPOS[6] = 0.0;		GOALPOS[7] = -DISTANCE1 - ADJUST;	GOALPOS[8] = 0.0;
			GOAL_YAW = 0.0; 		GOAL_PITCH = 0.0;					GOAL_ROLL = 0.0;
			ROTATE = 0.0;
			break;
		case FLWARD:
			// front
			GAIN[0] = 1.2;	GOALPOS[0] = 0.0;	GOALPOS[1] = DISTANCE1 + ADJUST;		GOALPOS[2] = 0.0;
			//mid
			GAIN[1] = 1.0;	GOALPOS[3] = 0.0;	GOALPOS[4] = DISTANCE2 + ADJUST;		GOALPOS[5] = 0.0;
			// hint
			GAIN[2] = 0.8;	GOALPOS[6] = 0.0;	GOALPOS[7] = DISTANCE3 + ADJUST;		GOALPOS[8] = 0.0;
			GOAL_YAW = PI/6; 		GOAL_PITCH = 0.0;				GOAL_ROLL = 0.0;
			ROTATE = 0.0;
			Transform(GOAL_YAW, GOAL_PITCH, GOAL_ROLL, GOALPOS);
			Transform(GOAL_YAW, GOAL_PITCH, GOAL_ROLL, GOALPOS+3);
			Transform(GOAL_YAW, GOAL_PITCH, GOAL_ROLL,  GOALPOS+6);
			break;
		case FRWARD:
		// front
			GAIN[0] = 1.2;	GOALPOS[0] = 0.0;	GOALPOS[1] = DISTANCE1 + ADJUST;		GOALPOS[2] = 0.0;
			//mid
			GAIN[1] = 1.0;	GOALPOS[3] = 0.0;	GOALPOS[4] = DISTANCE2 + ADJUST;		GOALPOS[5] = 0.0;
			// hint
			GAIN[2] = 0.8;	GOALPOS[6] = 0.0;	GOALPOS[7] = DISTANCE3 + ADJUST;		GOALPOS[8] = 0.0;
			GOAL_YAW = -PI/6; 		GOAL_PITCH = 0.0;				GOAL_ROLL = 0.0;
			ROTATE = 0.0;
			Transform(GOAL_YAW, GOAL_PITCH, GOAL_ROLL, GOALPOS);
			Transform(GOAL_YAW, GOAL_PITCH, GOAL_ROLL, GOALPOS+3);
			Transform(GOAL_YAW, GOAL_PITCH, GOAL_ROLL,  GOALPOS+6);
			break;
		case BRWARD:
			// front
			GAIN[0] = 0.8;	GOALPOS[0] = 0.0;		GOALPOS[1] = -DISTANCE3 - ADJUST;	GOALPOS[2] = 0.0;
			//mid
			GAIN[1] = 1.0;	GOALPOS[3] = 0.0;		GOALPOS[4] = -DISTANCE2 - ADJUST;	GOALPOS[5] = 0.0;
			// hint
			GAIN[2] = 1.2;	GOALPOS[6] = 0.0;		GOALPOS[7] = -DISTANCE1 - ADJUST;	GOALPOS[8] = 0.0;
			GOAL_YAW = PI/6; 		GOAL_PITCH = 0.0;					GOAL_ROLL = 0.0;
			ROTATE = 0.0;
			Transform(GOAL_YAW, GOAL_PITCH, GOAL_ROLL, GOALPOS);
			Transform(GOAL_YAW, GOAL_PITCH, GOAL_ROLL, GOALPOS+3);
			Transform(GOAL_YAW, GOAL_PITCH, GOAL_ROLL,  GOALPOS+6);
			break;			
		case BLWARD:
			// front
			GAIN[0] = 0.8;	GOALPOS[0] = 0.0;		GOALPOS[1] = -DISTANCE3 - ADJUST;	GOALPOS[2] = 0.0;
			//mid
			GAIN[1] = 1.0;	GOALPOS[3] = 0.0;		GOALPOS[4] = -DISTANCE2 - ADJUST;	GOALPOS[5] = 0.0;
			// hint
			GAIN[2] = 1.2;	GOALPOS[6] = 0.0;		GOALPOS[7] = -DISTANCE1 - ADJUST;	GOALPOS[8] = 0.0;
			GOAL_YAW = -PI/6; 		GOAL_PITCH = 0.0;					GOAL_ROLL = 0.0;
			ROTATE = 0.0;
			Transform(GOAL_YAW, GOAL_PITCH, GOAL_ROLL, GOALPOS);
			Transform(GOAL_YAW, GOAL_PITCH, GOAL_ROLL, GOALPOS+3);
			Transform(GOAL_YAW, GOAL_PITCH, GOAL_ROLL,  GOALPOS+6);
			break;		
		case RCIRCLE:
			// front
			GAIN[0] = 1.0;	GOALPOS[0] = 0.0;		GOALPOS[1] = 0.0;	GOALPOS[2] = 0.0;
			//mid
			GAIN[1] = 1.0;	GOALPOS[3] = 0.0;		GOALPOS[4] = 0.0;	GOALPOS[5] = 0.0;
			// hint
			GAIN[2] = 1.0;	GOALPOS[6] = 0.0;		GOALPOS[7] = 0.0;	GOALPOS[8] = 0.0;
			ROTATE = PI/12;
			break;
		case LCIRCLE:
			// front
			GAIN[0] = 1.0;	GOALPOS[0] = 0.0;		GOALPOS[1] = 0.0;	GOALPOS[2] = 0.0;
			//mid
			GAIN[1] = 1.0;	GOALPOS[3] = 0.0;		GOALPOS[4] = 0.0;	GOALPOS[5] = 0.0;
			// hint
			GAIN[2] = 1.0;	GOALPOS[6] = 0.0;		GOALPOS[7] = 0.0;	GOALPOS[8] = 0.0;
			ROTATE = -PI/12;
			break;
		default:
			break;	
	}
}

double POS_GAIN[3] = {0.0,0.0,0.0};

void AdjustPos(double *gain, double *pos)
{
	pos[0] = pos[0] + gain[0];
	pos[1] = pos[1] + gain[1];
	pos[2] = pos[2] + gain[2];
}
void SetMotion(int num)
{
	double temp_p, temp_r;
	GetLegMotion(LEGOPPOSITE, LEGGOAL, num, LEGFRAME[num],LEGPOS);
	if(SWING == 1)
	{
		ALPHA = ALPHA + PI/36.0;
		if(ALPHA >= 2*PI - PI/36.0)
			ALPHA = 0.0;
		temp_p = BODY_PITCH + gain_p*sin(ALPHA)*DEG_TO_RAD;
		temp_r = BODY_ROLL+ gain_r*sin(ALPHA+PI/2)*DEG_TO_RAD;
	}
	else
	{
		temp_p = BODY_PITCH;
		temp_r = BODY_ROLL;
	}
	Transform(BODY_YAW, temp_p, temp_r, LEGPOS);
	AdjustPos(POS_GAIN, LEGPOS);
	CenterToJoint3(LEGPOS,num,J3GOAL);
	Joint3ToJoint2(J3GOAL, J2GOAL);
	JointAngle(LEGPOS, J2GOAL,num,JANGLE);
	SetJoint(JANGLE,num);	
}

void PlayMotion(void)
{
	int i;
	
	for(i =0; i < 6; i++)
	{
		switch(LEGFRAME[i])
		{
			case -1:
				if(timer_nowtime() - PLAYTIMER[i] > STARTTIME[i]) 
				{
					LEGFRAME[i] = 0;
					SetMotion(i);
					rcservo_SetAction(POSITION, PLAYTIME);
					LEGFRAME[i]++;
					LEGTIMER[i] = timer_nowtime();
				}
				break;
			case 11:
				if(timer_nowtime() - LEGTIMER[i] > PLAYTIME) 
				{
					SetMotion(i);
					rcservo_SetAction(POSITION, PLAYTIME);
					LEGFRAME[i] = 0;
					LEGTIMER[i] = timer_nowtime();
				}
				break;	
			case 0:	
				if(timer_nowtime() - LEGTIMER[i] > PLAYTIME) 
				{
					SetMotion(i);
					rcservo_SetAction(POSITION, PLAYTIME);
					LEGFRAME[i]++;
					LEGTIMER[i] = timer_nowtime();
				}
				break;
			case 9:		
			case 1:
				if(timer_nowtime() - LEGTIMER[i] > PLAYTIME) 
				{
					SetMotion(i);
					rcservo_SetAction(POSITION, PLAYTIME);
					LEGFRAME[i] = LEGFRAME[i] + STEP_1;
					LEGTIMER[i] = timer_nowtime();
				}
				break;
			case 3:
			case 6:
				if(timer_nowtime() - LEGTIMER[i] > PLAYTIME) 
				{
					SetMotion(i);
					rcservo_SetAction(POSITION, PLAYTIME);
					LEGFRAME[i] = LEGFRAME[i] + STEP_2;
					LEGTIMER[i] = timer_nowtime();
				}
				break;		
			default:
				if(timer_nowtime() - LEGTIMER[i] > PLAYTIME) 
				{
					SetMotion(i);
					rcservo_SetAction(POSITION, PLAYTIME);
					LEGFRAME[i] = LEGFRAME[i]++;
					LEGTIMER[i] = timer_nowtime();
				}
				break;
		}
	}	
	rcservo_PlayAction();
}

#define 	LEN		17

void Idle(void)
{
	int i;
	double temp_p, temp_r;
	if(rcservo_PlayAction() == RCSERVO_PLAYEND)
	{
		for(i =0 ; i < 6; i++ )
		{
			LEGPOS[0] = LEGGOAL[3*i];
			LEGPOS[1] = LEGGOAL[3*i+1];
			LEGPOS[2] = LEGGOAL[3*i+2];
			if(SWING == 1)
			{
				ALPHA = ALPHA + PI/36.0;
				if(ALPHA >= 2*PI - PI/36.0)
					ALPHA = 0.0;
				temp_p = BODY_PITCH + gain_p*sin(ALPHA)*DEG_TO_RAD;
				temp_r = BODY_ROLL+ gain_r*sin(ALPHA+PI/2)*DEG_TO_RAD;
			}
			else
			{
				temp_p = BODY_PITCH;
				temp_r = BODY_ROLL;
			}
			Transform(BODY_YAW, temp_p, temp_r, LEGPOS);
			AdjustPos(POS_GAIN, LEGPOS);
			CenterToJoint3(LEGPOS,i,J3GOAL);
			Joint3ToJoint2(J3GOAL, J2GOAL);
			JointAngle(LEGPOS, J2GOAL,i,JANGLE);
			SetJoint(JANGLE,i);	
			rcservo_SetAction(POSITION, IDLEPLAYTIME);
			
		}
		IDLEFRAME = 0;
	}
	rcservo_PlayAction();
}

unsigned int G_SENSOR = 0x53;
int G_AXIS_VALUE[3];


int InitSensor(void)
{	
    if (i2c_Init2(0xffff,I2C_USEMODULE0+I2C_USEMODULE1,I2CIRQ_DISABLE,I2CIRQ_DISABLE) == false)
	{
		printf("FALSE!!  %s\n", roboio_GetErrMsg());
		return false;
	}	

	i2c0_SetSpeed(I2CMODE_AUTO, 400000L);
	i2c1_SetSpeed(I2CMODE_AUTO, 400000L);
	
	if(i2c0master_StartN(G_SENSOR,I2C_WRITE,2) == false)
	{
		printf("FALSE!!  %s\n", roboio_GetErrMsg());
		return false;
	}	
	i2c0master_WriteN(0x2d); 	//mode register
	i2c0master_WriteN(0x28); 	//Link and measure mode
	delay_ms(100);
	
	if(i2c0master_StartN(G_SENSOR,I2C_WRITE,2) == false)
	{
		printf("FALSE!!  %s\n", roboio_GetErrMsg());
		return false;
	}	
	i2c0master_WriteN(0x31); 	//mode register
	i2c0master_WriteN(0x08); 	//Full-resolution
	delay_ms(100);
	
	if(i2c0master_StartN(G_SENSOR,I2C_WRITE,2) == false)
	{
		printf("FALSE!!  %s\n", roboio_GetErrMsg());
		return false;
	}	
	i2c0master_WriteN(0x38); 	//mode register
	i2c0master_WriteN(0x00); 	//bypass mode
	delay_ms(100);
	
	return true;
}

int ReadGSensor(void)
{
	unsigned char d1,d2,d3,d4,d5,d6;
	if(i2c0master_StartN(G_SENSOR, I2C_WRITE, 1) == false)
	{
		printf("Gsensor : %s !!\n",roboio_GetErrMsg());
		return false;
	}
	i2c0master_SetRestartN(I2C_READ, 6);
	i2c0master_WriteN(0x32); 	//Read from X register (Address : 0x32)
	
	/*  write stop and start read  */
	d1 = i2c0master_ReadN();	//X MSB
	d2 = i2c0master_ReadN();	//X LSB
	d3 = i2c0master_ReadN();	//Y MSB
	d4 = i2c0master_ReadN();	//Y LSB 
	d5 = i2c0master_ReadN();	//Z MSB 
	d6 = i2c0master_ReadN();	//Z LSB 
			
	G_AXIS_VALUE[0] = (d2 & 0x02) ? ~(0xFFFF ^ (d2*256+d1)) : d2*256+d1;
	G_AXIS_VALUE[1] = (d4 & 0x02) ? ~(0xFFFF ^ (d4*256+d3)) : d4*256+d3;
	G_AXIS_VALUE[2] = (d6 & 0x02) ? ~(0xFFFF ^ (d6*256+d5)) : d6*256+d5;

	if(G_AXIS_VALUE[0] == 0 && G_AXIS_VALUE[1] == 0 && G_AXIS_VALUE[2] == 0) 
		return false;

	return true;
}

double Abs(double v)
{
	return v > 0.0 ? v : -v;
}

double GetPitch(double *gval)
{
	if(Abs(gval[2])< 0.000001)
	{
		if(Abs(gval[1]) < 0.000001)
			return 0.0;
		return gval[1] > 0.0 ? PI/2 : -PI/2;
	}
	return atan2(gval[1],gval[2]);	
}

double GetRoll(double *gval)
{
	double p;
	double angle;
	p = gval[1]*gval[1] + gval[2]*gval[2];
	p = sqrt(p);
	if(gval[2] > 0.0)
		angle = atan2(gval[0],p);
	else
		angle = atan2(gval[0],-p);
	return angle;
}

#define FRESHTIME		50
double STABLETIME = 100L, STABLETIMER;
double STABLE_P = 0.16, STABLE_D = 6.5;
void Stable(void)
{
	static double gval[3];
	static double pre_pitch, pitch, pre_roll, roll;
	int i;
	if(timer_nowtime() - STABLETIMER > FRESHTIME)
	{
		ReadGSensor();
		gval[0] = (double)(G_AXIS_VALUE[0]);
		gval[1] = (double)(G_AXIS_VALUE[1]);
		gval[2] = (double)(G_AXIS_VALUE[2]);
		pitch = (GetPitch(gval)- 5.0*DEG_TO_RAD)*0.5 + pre_pitch*0.5;
		roll = (GetRoll(gval) - 4.5*DEG_TO_RAD)*0.5 + pre_roll*0.5;
	
	
		BODY_PITCH = BODY_PITCH + STABLE_P*pitch + (pitch - pre_pitch)*STABLE_D/FRESHTIME;
		BODY_ROLL = BODY_ROLL + STABLE_P*roll + (roll - pre_roll)*STABLE_D/FRESHTIME;

		if(BODY_PITCH > 60.0*DEG_TO_RAD)
			BODY_PITCH = 60.0*DEG_TO_RAD;
		else if(BODY_PITCH < -60.0*DEG_TO_RAD)
			BODY_PITCH = -60.0*DEG_TO_RAD;
		if(BODY_ROLL > 60.0*DEG_TO_RAD)
			BODY_ROLL = 60.0*DEG_TO_RAD;
		else if(BODY_ROLL < -60.0*DEG_TO_RAD)
			BODY_ROLL = -60.0*DEG_TO_RAD;
			
		
		for(i =0 ; i < 6; i++ )
		{
			LEGPOS[0] = LEGGOAL[3*i] - cos(J3MAP[i])*20.0;
			LEGPOS[1] = LEGGOAL[3*i+1] - sin(J3MAP[i])*20.0;
			LEGPOS[2] = LEGGOAL[3*i+2];
			Transform(BODY_YAW, BODY_PITCH, BODY_ROLL, LEGPOS);
			AdjustPos(POS_GAIN, LEGPOS);
			CenterToJoint3(LEGPOS,i,J3GOAL);
			Joint3ToJoint2(J3GOAL, J2GOAL);
			JointAngle(LEGPOS, J2GOAL,i,JANGLE);
			SetJoint(JANGLE,i);	
			rcservo_SetAction(POSITION, STABLETIME);
			STABLETIMER = timer_nowtime(); 
		}
		pre_pitch = pitch;
		pre_roll = roll;
	}
	rcservo_PlayAction();
}

double  GOAL_HIGHT = 0.0;
double BUF[LEN] = {150.0, 0.0, 2.0, 0.0, 2.0, 0.0, 2.0, 25.0, 25.0, 25.0, 20.0, 20.0, 30.0, 5.0, 5.0, 0.16, 6.5};

void SetVal(void)
{
	int i;
	/*for( i = 0; i < LEN; i++)
		printf("%f ",BUF[i]);
	putchar('\n');
	*/
	PLAYTIME = (long)BUF[0];
	for(i = 1; i < 7; i++)
		STARTTIME[i-1] = BUF[i] *PLAYTIME;
	DISTANCE1 = BUF[7];
	DISTANCE2 = BUF[8]; 
	DISTANCE3 = BUF[9];
	DISTANCE4 = BUF[10];
	GOAL_HIGHT = BUF[11];
	RISE = BUF[12];
	STEP_1 = (int)BUF[13];
	STEP_2 = (int) BUF[14];
	STABLE_P = BUF[15];
	STABLE_D = BUF[16];
	
}

void Prepare(void)
{
	int i;
	GetLegOrigin(LEGORIGIN);
	Update();
	switch(STATE)
	{
		case IDLE:
			
			for(i = 0; i < 6; i++)
			{
				LEGGOAL[i*3] = LEGORIGIN[i*3];
				LEGGOAL[i*3+1] = LEGORIGIN[i*3+1];
				LEGGOAL[i*3 + 2] = LEGORIGIN[i*3 + 2] + GOAL_HIGHT;
			}
			break;
		case BALANCE:
			InitSensor();
			for(i = 0; i < 6; i++)
			{
				LEGGOAL[i*3] = LEGORIGIN[i*3];
				LEGGOAL[i*3+1] = LEGORIGIN[i*3+1];
				LEGGOAL[i*3 + 2] = LEGORIGIN[i*3 + 2];
			}
			SetOrigin(NORMAL, POSITION);
			STABLETIMER = timer_nowtime();	
			break;
			
		default:
			if(SWING == 0)
				SetOrigin(NORMAL, POSITION);
			for(i = 0; i < 6; i++)
			{
				LEGFRAME[i] = -1;
				GetLegGoal(GOALPOS, i, LEGGOAL);
				GetLegOpposite(GOALPOS, i, STATE,LEGOPPOSITE);
				if(SWING == 1)
				{
					LEGGOAL[i*3 + 2] = LEGGOAL[i*3 + 2] + 10.0;
					LEGOPPOSITE[i*3 + 2] = LEGOPPOSITE[i*3 + 2] + GOAL_HIGHT;
				}	
					
				if(STATE == RCIRCLE || STATE == LCIRCLE)
				{ 
					Transform(ROTATE, GOAL_PITCH, GOAL_ROLL, LEGGOAL+3*i);
					Transform(-ROTATE, GOAL_PITCH, GOAL_ROLL, LEGOPPOSITE+3*i);
				}	
				PLAYTIMER[i] = timer_nowtime();	
			}
			break;
	}	

}

int HOLD = ~0;
void GetInput(void)
{
	int c = 0, i;
	
	if(key[KEY_B])
	{
		HOLD = ~(HOLD);
		ADJUST = 0.0;
		BODY_YAW = 0.0;
		BODY_PITCH = 0.0;
		BODY_ROLL = 0.0;
		POS_GAIN[0] = 0.0;
		POS_GAIN[1] = 0.0;
		POS_GAIN[2] = 0.0;
		if(HOLD != 0)
		{
			STATE = BALANCE;
			printf("enable balance mode !\n");
			Prepare();
		}	
		else
		{
			printf("disable balance mode !\n");
			i2c_Close();
		}
		delay_ms(500);
	}
	
	if(key[KEY_1])
	{
		BUF[0] = 150.0;		
		BUF[1] = 0.0;		BUF[2] = 2.0;		BUF[3] = 0.0;		BUF[4] = 2.0;		BUF[5] = 0.0;		BUF[6] = 2.0;
		BUF[7] = 25.0;	BUF[8] = 25.0;	BUF[9] = 25.0;	BUF[10] = 20.0;
		BUF[11] = 20.0;	BUF[12] = 30.0;
		BUF[13] = 5.0;	BUF[14] = 5.0;

		SetVal();
		SWING = 0;
		delay_ms(500);
	}
	else if(key[KEY_2])
	{
		BUF[0] = 300.0;		
		BUF[1] = 4.0;		BUF[2] = 2.0;		BUF[3] = 0.0;		BUF[4] = 1.0;		BUF[5] = 5.0;		BUF[6] = 3.0;
		BUF[7] = 25.0;	BUF[8] = 25.0;	BUF[9] = 25.0;	BUF[10] = 20.0;
		BUF[11] = 20.0;	BUF[12] = 30.0;
		BUF[13] = 2.0;	BUF[14] = 3.0;

		SetVal();
		SWING = 0;
		delay_ms(500);
	}
	else if(key[KEY_3])
	{
		BUF[0] = 150.0;		
		BUF[1] = 4.0;		BUF[2] = 2.0;		BUF[3] = 0.0;		BUF[4] = 10.0;		BUF[5] = 8.0;		BUF[6] = 6.0;
		BUF[7] = 25.0;	BUF[8] = 25.0;	BUF[9] = 25.0;	BUF[10] = 20.0;
		BUF[11] = 20.0;	BUF[12] = 30.0;
		BUF[13] = 1.0;	BUF[14] = 1.0;

		SetVal();
		SWING = 0;
		delay_ms(500);
	}
	else if(key[KEY_4])
	{
		BUF[0] = 300.0;		
		BUF[1] = 0.0;		BUF[2] = 2.0;		BUF[3] = 0.0;		BUF[4] = 2.0;		BUF[5] = 0.0;		BUF[6] = 2.0;
		BUF[7] = 15.0;	BUF[8] = 15.0;	BUF[9] = 15.0;	BUF[10] = 15.0;
		BUF[11] = 10.0;	BUF[12] = 20.0;
		BUF[13] = 5.0;	BUF[14] = 5.0;

		IDLEPLAYTIME = 300.0;
		SetVal();
		SWING = 1;
		printf("swing!!\n");
		delay_ms(500);
	}

	if(key[KEY_U])
	{
		ADJUST = ADJUST + 0.2;
		printf("gain adjust:%f!!\n",ADJUST);
	}	
	if(key[KEY_D])	
	{
		ADJUST = ADJUST - 0.2;
		printf("gain adjust:%f!!\n",ADJUST);
	}	
	if(key[KEY_L])
	{
		printf("play slow!!\n");
		PLAYTIME = PLAYTIME + 50;
		printf("playtime:%ld\n",PLAYTIME);
		printf("starttime:");
		for(i = 0; i < 6; i++)
		{
			STARTTIME[i] = BUF[i+1]*PLAYTIME;
			printf("%ld ",STARTTIME[i]);
		}	
		printf("\n");
		delay_ms(500);
	}	
	if(key[KEY_F])	
	{
		printf("play fast!!\n");
		PLAYTIME = PLAYTIME - 50;
		printf("playtime:%ld\n",PLAYTIME);
		printf("starttime:");
		for(i = 0; i < 6; i++)
		{
			STARTTIME[i] = BUF[i+1]*PLAYTIME;
			printf("%ld ",STARTTIME[i]);
		}	
		printf("\n");
		delay_ms(500);
	}	
	if(key[KEY_K])
	{
		ADJUST = 0.0;
		PLAYTIME = (long)BUF[0];
		for(i = 0; i < 6; i++)
			STARTTIME[i] = BUF[i+1]*PLAYTIME;
		BODY_YAW = 0.0;
		BODY_PITCH = 0.0;
		BODY_ROLL = 0.0;
		POS_GAIN[0] = 0.0;
		POS_GAIN[1] = 0.0;
		POS_GAIN[2] = 0.0;
		STATE = IDLE;
	}
	
	if(key[KEY_X] && key[KEY_PGUP])
	{
		if(POS_GAIN[0] < 30.0)
			POS_GAIN[0] = POS_GAIN[0]  + 0.2;
		printf("pos_x:%f\n",POS_GAIN[0]);
	}
	if(key[KEY_X] && key[KEY_PGDN])
	{
		if(POS_GAIN[0] > -30.0)
			POS_GAIN[0] = POS_GAIN[0]  - 0.2;
		printf("pos_x:%f\n",POS_GAIN[0]);
	}
	if(key[KEY_Y] && key[KEY_PGUP])
	{
		if(POS_GAIN[1] < 25.0)
		POS_GAIN[1] = POS_GAIN[1]  + 0.2;
		printf("pos_y:%f\n",POS_GAIN[1]);
	}
	if(key[KEY_Y] && key[KEY_PGDN])
	{
		if(POS_GAIN[1] > -25.0)
			POS_GAIN[1] = POS_GAIN[1]  - 0.2;
		printf("pos_y:%f\n",POS_GAIN[1]);
	}
	if(key[KEY_Z] && key[KEY_PGUP])
	{
		if(POS_GAIN[2] < 32.0)
			POS_GAIN[2] = POS_GAIN[2]  + 0.2;
		printf("pos_z:%f\n",POS_GAIN[2]);
	}
	if(key[KEY_Z] && key[KEY_PGDN])
	{
		if(POS_GAIN[2] > -20.0)
			POS_GAIN[2] = POS_GAIN[2]  - 0.2;
		printf("pos_z:%f\n",POS_GAIN[2]);
	}
	if(key[KEY_V] && key[KEY_PGUP])
	{
		BODY_YAW = BODY_YAW  + 0.5*DEG_TO_RAD ;
		printf("yaw:%f\n",BODY_YAW*RAD_TO_DEG);
	}
	if(key[KEY_V] && key[KEY_PGDN])
	{
		BODY_YAW = BODY_YAW -0.5*DEG_TO_RAD;
		printf("yaw:%f\n",BODY_YAW*RAD_TO_DEG);
	}
	if(key[KEY_P] && key[KEY_PGUP])
	{
		BODY_PITCH = BODY_PITCH  + 0.2*DEG_TO_RAD ;
		printf("pitch:%f\n",BODY_PITCH*RAD_TO_DEG);
	}
	if(key[KEY_P] && key[KEY_PGDN])
	{
		BODY_PITCH = BODY_PITCH - 0.2*DEG_TO_RAD ;
		printf("pitch:%f\n",BODY_PITCH*RAD_TO_DEG);
	}
	if(key[KEY_R] && key[KEY_PGUP])
	{
		BODY_ROLL = BODY_ROLL  + 0.2*DEG_TO_RAD ;
		printf("roll:%f\n",BODY_ROLL*RAD_TO_DEG);
	}
	if(key[KEY_R] && key[KEY_PGDN])
	{
		BODY_ROLL = BODY_ROLL - 0.2*DEG_TO_RAD ;
		printf("roll:%f\n",BODY_ROLL*RAD_TO_DEG);
	}
	if(HOLD != 0)
		return;
		
	c = -key[KEY_UP];
	c = c | -(key[KEY_LEFT]<<1);
	c = c | -(key[KEY_RIGHT]<<2);
	c = c | -(key[KEY_DOWN]<<3);
	c = c | -(key[KEY_SPACE]<<4);
	c = c | -(key[KEY_O]<<5);
	
	PRE_STATE = STATE;
	switch(c)
	{
		case 1:
			STATE =  FORWARD;
			break;
		case 2:
			STATE = LEFTWARD;
			break;
		case 4:
			STATE = RIGHTWARD;
			break;			
		case 8:
			STATE = BACKWARD;
			break;
		case 3:
			STATE =  FLWARD;
			break;
		case 5:
			STATE = FRWARD;
			break;
		case 10:
			STATE = BLWARD;
			break;			
		case 12:
			STATE = BRWARD;
			break;	
		case 34:
			STATE = LCIRCLE;
			break;
		case 36: 	
			STATE = RCIRCLE;
			break;
		case 16:
			STATE = EXIT;
			break;	
			
		default:
				STATE = IDLE;
			break;
	}
	if(PRE_STATE != STATE)
		Prepare();

}



int main()
{	
	unsigned long usedchannel = RCSERVO_USECHANNEL0 +RCSERVO_USECHANNEL1 +RCSERVO_USECHANNEL2 +
		          RCSERVO_USECHANNEL4 +RCSERVO_USECHANNEL5 +RCSERVO_USECHANNEL6+
				  RCSERVO_USECHANNEL9 +RCSERVO_USECHANNEL10+RCSERVO_USECHANNEL11+
				  RCSERVO_USECHANNEL13+RCSERVO_USECHANNEL14+RCSERVO_USECHANNEL15+
				  RCSERVO_USECHANNEL16+RCSERVO_USECHANNEL17+RCSERVO_USECHANNEL18+
				  RCSERVO_USECHANNEL21+RCSERVO_USECHANNEL22+RCSERVO_USECHANNEL23; // for RB-100
		
	if(allegro_init())
	{
		printf("error:initialize allegro library failed!!\n");
		return -1;
	}
	if(install_keyboard())
	{
		printf("error:initialize keybaord failed!!\n");
		return -1;
	}
	roboio_SetRBVer(RB_100);
	
	if(rcservo_SetServos(usedchannel, RCSERVO_DMP_RS0263) == false)
	{
		printf("Set servo fails!%s\n",roboio_GetErrMsg());
		return -1;
	}
	if(rcservo_Initialize(usedchannel) == false)
	{
		printf("RC servo initialize fails!%s\n",roboio_GetErrMsg());
		return -1;
	}
	
	rcservo_SetFPS(500);
	rcservo_EnterPlayMode_NOFB(NORMAL);
	SetVal();
	PRE_STATE = STATE = BALANCE;
	ADJUST = 0.0;
	BODY_YAW = 0.0;
	BODY_PITCH = 0.0;
	BODY_ROLL = 0.0;
	POS_GAIN[0] = 0.0;
	POS_GAIN[1] = 0.0;
	POS_GAIN[2] = 0.0;
	Prepare();
	while(STATE != EXIT)
	{
		GetInput();
		switch(STATE)
		{
			case FORWARD:
			case BACKWARD:
			case RIGHTWARD:
			case LEFTWARD:
			case FLWARD:
			case FRWARD:
			case BLWARD:
			case BRWARD:
			case LCIRCLE:
			case RCIRCLE:
					PlayMotion();
				break;
		
			case IDLE:
					Idle();
				break;	
			case BALANCE:
					Stable();
				break;
			default:
				break;
		}
	}

	rcservo_Close();
	i2c_Close();
	remove_keyboard();
	
	return 0;
}
END_OF_MAIN();