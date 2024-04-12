/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOInterface.h"
#include "interface/CmdPanel.h"
#include "common/unitreeRobot.h"
#include "Gait/WaveGenerator.h"
#include "control/Estimator.h"
#include "control/BalanceCtrl.h"
#include <string>
#include <iostream>
// #include "common/PyPlot.h"
#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

struct CtrlComponents{
public:
    CtrlComponents(IOInterface *ioInter):ioInter(ioInter){
        lowCmd = new LowlevelCmd();
        lowState = new LowlevelState();
        contact = new VecInt4;
        phase = new Vec4;
        *contact = VecInt4(0, 0, 0, 0);
        *phase = Vec4(0.5, 0.5, 0.5, 0.5);
    }
    ~CtrlComponents(){
        delete lowCmd;
        delete lowState;
        delete ioInter;
        delete robotModel;
        delete waveGen;
        delete estimator;
        delete balCtrl;
        // delete plot;
#ifdef COMPILE_DEBUG
        delete plot;
#endif  // COMPILE_DEBUG
    }
    LowlevelCmd *lowCmd;
    LowlevelState *lowState;
    IOInterface *ioInter;
    QuadrupedRobot *robotModel;
    WaveGenerator *waveGen;
    Estimator *estimator;
    BalanceCtrl *balCtrl;
    // PyPlot *plot;
#ifdef COMPILE_DEBUG
    PyPlot *plot;
#endif  // COMPILE_DEBUG

    VecInt4 *contact;
    Vec4 *phase;

    double dt;
    bool *running;
    CtrlPlatform ctrlPlatform;

    void sendRecv(){
        ioInter->sendRecv(lowCmd, lowState);
    }

    void runWaveGen(){
        waveGen->calcContactPhase(*phase, *contact, _waveStatus);
    }

    void setAllStance(){
        _waveStatus = WaveStatus::STANCE_ALL;
    }

    void setAllSwing(){
        _waveStatus = WaveStatus::SWING_ALL;
    }

    void setStartWave(){
        _waveStatus = WaveStatus::WAVE_ALL;
    }

    void geneObj(){
        estimator = new Estimator(robotModel, lowState, contact, phase, dt);
        balCtrl = new BalanceCtrl(robotModel);
        // plot=new PyPlot();
        //plot->addPlot("F_view",4,std::vector<std::string>{"F_R_f","F_L_f","F_R_B","F_L_B"});
        // plot->addPlot("F_R_f",1,std::vector<std::string>{"F_R_f"});
        // plot->addPlot("F_L_f",1,std::vector<std::string>{"F_L_f"});
        // plot->addPlot("F_R_B",1,std::vector<std::string>{"F_R_B"});
        // plot->addPlot("F_L_B",1,std::vector<std::string>{"F_L_B"});
        //plot->addPlot("euler",3,std::vector<std::string>{"r","p","y"});
#ifdef COMPILE_DEBUG
        plot = new PyPlot();
        balCtrl->setPyPlot(plot);
        estimator->setPyPlot(plot);
#endif  // COMPILE_DEBUG
    }

private:
    WaveStatus _waveStatus = WaveStatus::SWING_ALL;

};

#endif  // CTRLCOMPONENTS_H