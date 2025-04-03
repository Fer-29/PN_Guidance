#include<stdio.h>
#include<math.h>
#include<stdbool.h>
// This is a 2 Dimensional P-N Guidance algorithm  simulation in C

// structs are supposed to be vectors :)
// for compactness I've merged pos and vel

typedef struct {
    float xpos;
    float ypos;
    float xvel;
    float yvel;
} Target;


typedef struct {
    float xpos;
    float ypos;
    float xvel;
    float yvel;
} Chaser;


typedef struct {
    float xlos;
    float ylos;
    float angle;
    float rateAngle;  
} LOS;

typedef struct {
    float xaccel;
    float yaccel;
    int N; // Navigation Gain
} cmdAccel;

// I've assumed MKS system, you can go for british units if you like 
void initialize_states(LOS *los, Target *target, Chaser *chaser){
    target->xpos = 100;
    target->ypos = 50;
    target->xvel = 10;
    target->yvel = 0;

    chaser->xpos = 0;
    chaser->ypos = 0;
    chaser->xvel = 0;
    chaser->yvel = 0;
  
    los->xlos = 0;
    los->ylos = 0;
    los->angle = 0;
    los->rateAngle = 0;
}
//Step 3 in the picture, we upate states with the time step 
void  updateState(LOS *los, Target *target, Chaser *chaser, cmdAccel *acmd, int dt){
    target->xvel = target->xvel;
    target->yvel = target->yvel;

    target->xpos = target->xpos;
    target->ypos = target->ypos;
  
    //upate vel first since a change in pos depends on the change of velocity
    chaser->xvel = chaser->xvel + acmd->xaccel * dt;
    chaser->yvel = chaser->yvel + acmd->yaccel * dt;

    chaser->xpos = chaser->xpos + chaser->xvel * dt;
    chaser->ypos = chaser->ypos + chaser->yvel * dt;
  
    los->xlos = target->xpos - chaser->xpos;
    los->ylos = target->ypos - chaser->ypos;
  
    los->angle = atan((target->ypos - chaser->ypos)/(target->xpos - chaser->xpos));
    los->rateAngle = ((target->xpos - chaser->xpos) * (target->yvel - chaser->yvel)
		      - (target->ypos - chaser->ypos) * (target->xvel - chaser->xvel))/(powf(los->xlos,2) + powf(los->ylos,2));
}
// Commanded acceleration by the algorithm
void calculateAccel(cmdAccel *acmd, LOS *los){ 
    acmd->N = 4;
    acmd->xaccel = acmd->N * los->rateAngle * -sin(los->angle); 
    acmd->yaccel = acmd->N * los->rateAngle * cos(los->angle); 
}
void printValues(LOS *los, Target *target, Chaser *chaser, cmdAccel *acmd, int dt){
    printf("%2.2f%s\n",chaser->xpos," x position");
    printf("%2.2f%s\n",chaser->ypos," y position");
    printf("%2.2f%s\n",acmd->xaccel," x cmd accel");
    printf("%2.2f%s\n",acmd->yaccel," y cmd accel");
    printf("%d%s\n", dt," <- Time Step");
    float xdiff = target->xpos - chaser->xpos;     
    float ydiff = target->ypos - chaser->ypos;
    printf("%2.2f%s\n",xdiff," delta x pos");
    printf("%2.2f%s\n",ydiff," delta y pos");
    
}

bool checkTargetHit(Target *target, Chaser *chaser){
    float xdiff = target->xpos - chaser->xpos;     
    float ydiff = target->ypos - chaser->ypos;
    if (xdiff <= 0 || ydiff <= 0){
	return true;
    }else{
	return false;
    }
}

int main(){
  
    LOS los;
    Target target;
    Chaser chaser;
    cmdAccel acmd;
    initialize_states(&los, &target, &chaser);
    for(int t=0; t<200; t++){
	updateState(&los, &target, &chaser, &acmd,t);
	calculateAccel(&acmd,&los);
	printValues(&los, &target, &chaser, &acmd,t);
	printf("%B\n",checkTargetHit(&target,&chaser));
	if (checkTargetHit(&target,&chaser) == true){
	    break;
	}
    }
	
    
    return 0;
}
