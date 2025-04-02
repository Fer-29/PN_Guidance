#include<stdio.h>

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
} LOS;

// I've assumed mks system, you can go for british units if you like in initialization
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
}

int main(){
  
  LOS los;
  Target target;
  Chaser chaser;
  initialize_states(&los, &target, &chaser);

  for(int t=0; t<10; t++){
    printf("test\n");
  }
  return 0;
}
