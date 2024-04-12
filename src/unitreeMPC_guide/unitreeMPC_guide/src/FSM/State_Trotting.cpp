/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Trotting.h"
#include <iomanip>
#include "utils/Utils.h"

State_Trotting::State_Trotting(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::TROTTING, "trotting"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), 
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel){
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.08;
    ctrl_states.reset();
    
    mpc_init_counter=0;
#ifdef ROBOT_TYPE_Go1
    _Kpp = Vec3(70, 70, 70).asDiagonal();
    _Kdp = Vec3(10, 10, 10).asDiagonal();
    _kpw = 780; 
    _Kdw = Vec3(70, 70, 70).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

#ifdef ROBOT_TYPE_A1
    _Kpp = Vec3(20, 20, 100).asDiagonal();
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 400;
    _Kdw = Vec3(50, 50, 50).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();

}

State_Trotting::~State_Trotting(){
    delete _gait;
}

void State_Trotting::enter(){
    _pcd = _est->getPosition();
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);
    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();
    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();

}

void State_Trotting::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_Trotting::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::TROTTING;
    }
}

void State_Trotting::run(){
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();
    
    mpc_init_counter++;
    if(mpc_init_counter==10000)
    {
        mpc_init_counter=0;
    }

    _rpy=_lowState->quatToRPY();
    _rpyd=_lowState->getRPYD();
    _userValue = _lowState->userValue;

    getUserCmd();
    calcCmd();

    _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcTau();
    calcQQd();

    if(checkStepOrNot()){
        _ctrlComp->setStartWave();
    }else{
        _ctrlComp->setAllStance();
    }

    _lowCmd->setTau(_tau);
    _lowCmd->setQ(vec34ToVec12(_qGoal));
    _lowCmd->setQd(vec34ToVec12(_qdGoal));

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _lowCmd->setSwingGain(i);
        }else{
            _lowCmd->setStableGain(i);
        }
    }

}

bool State_Trotting::checkStepOrNot(){
    if( (fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_posError(0)) > 0.08) ||
        (fabs(_posError(1)) > 0.08) ||
        (fabs(_velError(0)) > 0.05) ||
        (fabs(_velError(1)) > 0.05) ||
        (fabs(_dYawCmd) > 0.20) ){
        return true;
    }else{
        return false;
    }
}

void State_Trotting::setHighCmd(double vx, double vy, double wz){
    _vCmdBody(0) = vx;
    _vCmdBody(1) = vy;
    _vCmdBody(2) = 0; 
    _dYawCmd = wz;
}

void State_Trotting::getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(_userValue.ly, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;
    
}

void State_Trotting::calcCmd(){
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2));

    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    _vCmdGlobal(2) = 0;

    /* Turning */
    
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;
    
    
    _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;

    ctrl_states.root_euler=_rpy;

    ctrl_states.root_pos=_posBody;

    ctrl_states.root_ang_vel=_rpyd;

    // std::cout<<"rpy"<<_rpy<<std::endl;

    ctrl_states.root_lin_vel=_velBody;//
    ctrl_states.root_rot_mat=_B2G_RotMat;
    ctrl_states.root_lin_vel_d=_vCmdBody;
    ctrl_states.root_ang_vel_d[2]=_dYawCmd;
    ctrl_states.foot_pos_abs=_posFeet2BGlobal;
    ctrl_states.contacts=*_contact;
    //ctrl_states.root_pos_d[2]=_pcd(2);
    ctrl_states.root_pos_d[2]=_pcd(2);
    // std::cout<<"pcd"<<_pcd(2)<<std::endl;
}

void State_Trotting::calcTau(){

    if(mpc_init_counter%25==0)
        _forceFeetGlobal=-compute_grf(ctrl_states,0.05);

    double _force1=_forceFeetGlobal(0,0);
    //std::cout<<"力"<<"_forceFeetGlobal:"<<_forceFeetGlobal<<std::endl;

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    Eigen::Matrix<double, 4, 1> norm_f;
    norm_f<<_forceFeetBody.col(0).norm(),
            _forceFeetBody.col(1).norm(),
            _forceFeetBody.col(2).norm(),
            _forceFeetBody.col(3).norm();

    // _ctrlComp->plot->addFrame("F_R_f",norm_f(0));
    // _ctrlComp->plot->addFrame("F_L_f",norm_f(1));
    // _ctrlComp->plot->addFrame("F_R_B",norm_f(2));
    // _ctrlComp->plot->addFrame("F_L_B",norm_f(3));

    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);
}

void State_Trotting::calcQQd(){

    Vec34 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState,FrameType::BODY);
    
    for(int i(0); i<4; ++i){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); 
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12) 
    }
    
    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
}

Eigen::Matrix<double, 3, NUM_LEG> State_Trotting::compute_grf(CtrlStates &state, double dt)//MPC部分
{
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
    ConvexMpc mpc_solver = ConvexMpc(state.q_weights, state.r_weights);
    mpc_solver.reset();

    state.root_ang_vel_d_world;

    state.mpc_states << state.root_euler[0], state.root_euler[1], state.root_euler[2],
            state.root_pos[0], state.root_pos[1], state.root_pos[2],
            state.root_ang_vel[0], state.root_ang_vel[1], state.root_ang_vel[2],
            state.root_lin_vel[0], state.root_lin_vel[1], state.root_lin_vel[2],
            -9.8;

        
    double mpc_dt = dt;
    // initialize the desired mpc states trajectory

    state.root_lin_vel_d_world = state.root_rot_mat * state.root_lin_vel_d;
    state.root_ang_vel_d_world=state.root_rot_mat *state.root_ang_vel_d;

        //state.root_ang_vel_d = state.root_rot_mat * state.root_ang_vel_d;

        // state.mpc_states_d.resize(13 * PLAN_HORIZON);
    for (int i = 0; i < PLAN_HORIZON; ++i) {
        state.mpc_states_d.segment(i * 13, 13)
                <<
                state.root_euler_d[0],
                state.root_euler_d[1],
                // 0,
                // 0,
                //state.root_euler[2] + state.root_ang_vel_d_world[2] * mpc_dt * (i + 1),
                state.root_euler[2] + state.root_ang_vel_d_world[2] * mpc_dt * (i + 1),
                state.root_pos[0] + state.root_lin_vel_d_world[0] * mpc_dt * (i + 1),
                state.root_pos[1] + state.root_lin_vel_d_world[1] * mpc_dt * (i + 1),
                state.root_pos_d[2],
                state.root_ang_vel_d_world[0],
                state.root_ang_vel_d_world[1],
                state.root_ang_vel_d_world[2],
                state.root_lin_vel_d_world[0],
                state.root_lin_vel_d_world[1],
                0,
                -9.8;
    }

        // a single A_c is computed for the entire reference trajectory
        auto t1 = std::chrono::high_resolution_clock::now();
        mpc_solver.calculate_A_mat_c(state.root_euler);//计算单刚体动力学模型中连续矩阵A

        // for each point in the reference trajectory, an approximate B_c matrix is computed using desired values of euler angles and feet positions
        // from the reference trajectory and foot placement controller
        // state.foot_pos_abs_mpc = state.foot_pos_abs;
        auto t2 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < PLAN_HORIZON; i++) {
            // calculate current B_c matrix
            mpc_solver.calculate_B_mat_c(state.robot_mass,
                                         state.inertia,
                                         state.root_rot_mat,
                                         state.foot_pos_abs);
            // state space discretization, calculate A_d and current B_d
            mpc_solver.state_space_discretization(mpc_dt);

            // store current B_d matrix
            mpc_solver.B_mat_d_list.block<13, 12>(i * 13, 0) = mpc_solver.B_mat_d;
        }

        // calculate QP matrices
        auto t3 = std::chrono::high_resolution_clock::now();
        mpc_solver.calculate_qp_mats(state);

        // solve
        auto t4 = std::chrono::high_resolution_clock::now();
        if (!solver.isInitialized()) {
            solver.settings()->setVerbosity(false);
            solver.settings()->setWarmStart(true);
            solver.data()->setNumberOfVariables(NUM_DOF * PLAN_HORIZON);
            solver.data()->setNumberOfConstraints(MPC_CONSTRAINT_DIM * PLAN_HORIZON);
            solver.data()->setLinearConstraintsMatrix(mpc_solver.linear_constraints);
            solver.data()->setHessianMatrix(mpc_solver.hessian);
            solver.data()->setGradient(mpc_solver.gradient);
            solver.data()->setLowerBound(mpc_solver.lb);
            solver.data()->setUpperBound(mpc_solver.ub);
            solver.initSolver();
        } else {
            solver.updateHessianMatrix(mpc_solver.hessian);
            solver.updateGradient(mpc_solver.gradient);
            solver.updateLowerBound(mpc_solver.lb);
            solver.updateUpperBound(mpc_solver.ub);
        }
        auto t5 = std::chrono::high_resolution_clock::now();
        solver.solve();
        auto t6 = std::chrono::high_resolution_clock::now();

        // std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
        // std::chrono::duration<double, std::milli> ms_double_2 = t3 - t2;
        // std::chrono::duration<double, std::milli> ms_double_3 = t4 - t3;
        // std::chrono::duration<double, std::milli> ms_double_4 = t5 - t4;
        // std::chrono::duration<double, std::milli> ms_double_5 = t6 - t5;

//        std::cout << "mpc cal A_mat_c: " << ms_double_1.count() << "ms" << std::endl;
//        std::cout << "mpc cal B_mat_d_list: " << ms_double_2.count() << "ms" << std::endl;
//        std::cout << "mpc cal qp mats: " << ms_double_3.count() << "ms" << std::endl;
//        std::cout << "mpc init time: " << ms_double_4.count() << "ms" << std::endl;
//        std::cout << "mpc solve time: " << ms_double_5.count() << "ms" << std::endl << std::endl;

        Eigen::VectorXd solution = solver.getSolution();
        // std::cout << solution.transpose() << std::endl;

        for (int i = 0; i < NUM_LEG; ++i) {
            if (!isnan(solution.segment<3>(i * 3).norm()))
                foot_forces_grf.block<3, 1>(0, i) = solution.segment<3>(i * 3);
        }
        
    return foot_forces_grf;
    
}

