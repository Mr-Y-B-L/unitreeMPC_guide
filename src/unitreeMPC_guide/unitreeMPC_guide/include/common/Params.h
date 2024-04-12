//
// Created by shuoy on 10/18/21.
//

#ifndef PARAMS_H
#define PARAMS_H

// control time related
#define MAIN_UPDATE_FREQUENCY 2.5 // ms

// mpc#define PLAN_HORIZON 16
#define PLAN_HORIZON 10
#define MPC_STATE_DIM 13
#define MPC_CONSTRAINT_DIM 20

// robot constant
#define NUM_LEG 4
#define NUM_DOF_PER_LEG 3
#define DIM_GRF 12
#define NUM_DOF 12

#endif //PARAMS_H
