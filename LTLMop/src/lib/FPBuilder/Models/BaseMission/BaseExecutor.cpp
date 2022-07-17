// ======================================================================
// \title  #Base#Executor.cpp
// \author nonsense
// \brief  cpp file for #Base#Executor component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================


#include <Ref/#Base#Mission/#Base#Executor.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Ref {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  #Base#Executor ::
    #Base#Executor(
        const char *const compName
    ) : #Base#ExecutorComponentBase(compName)
  {

  }

  void #Base#Executor ::
    init(
        const NATIVE_INT_TYPE queueDepth,
        const NATIVE_INT_TYPE instance
    )
  {
    #Base#ExecutorComponentBase::init(queueDepth, instance);
    this->SpecStrategy = new Strategy("#Base#Mission/Config/#Base#.spec");
    this->envChange = false;
    this->stopFlag = false;
    this->curActions.clear();
    this->doneActionsMap.clear();
  }

  #Base#Executor ::
    ~#Base#Executor()
  {
    delete this->SpecStrategy;
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void #Base#Executor ::
    envSetin_handler(
        const NATIVE_INT_TYPE portNum,
        const nameString &name,
        I8 value
    )
  {
    // TODO
    if (this->SpecStrategy->curSensorMap[name.toChar()] != value)
      this->envChange = true;

    this->SpecStrategy->curSensorMap[name.toChar()] = value;
    this->log_ACTIVITY_HI_ENV_UPDATE(name, value == 1);
  }

  void #Base#Executor ::
    #base#ExeCtrlin_handler(
        const NATIVE_INT_TYPE portNum,
        I8 control_id
    )
  {
    // TODO
    switch (control_id) {
      case 0: {  // start
        this->log_ACTIVITY_LO_INIT_SUCC();
        this->log_ACTIVITY_HI_START_RECV();
        if (!this->stopFlag)
          this->#base#Startout_out(0, 0);
        this->stopFlag = false;
        break;
      }
      case 1:   // stop
        this->stopFlag = true;
        break;
      case 2:   // reset
        this->envChange = false;
        this->SpecStrategy->setCurState(0);
        this->doneActionsMap.clear();
        this->curActions.clear();
        this->log_ACTIVITY_HI_RESET_RECV();
        break;
      default:
        break;
    }
  }

  bool #Base#Executor ::
    #base#Wakein_handler(
        const NATIVE_INT_TYPE portNum
    )
  {
    // TODO return
    std::vector<std::string> tmp;
    I8 envFlag = 0;

    if (this->stopFlag){
      this->log_ACTIVITY_HI_STOP_RECV();
      for (std::string act : this->curActions){
        if (act.find("!") == 0)
          this->RunActuators(act, 0);
        else
          this->RunActuators("!"+act, 0);
      }
      return true;
    }

    // check whether the env has changed  -- state change
    if (this->envChange)
    {
      this->envChange = false;
      envFlag = 1;
      std::unordered_map<std::string, int> backupMap(this->SpecStrategy->curSensorMap);
      if (this->curActions.size() == this->doneActionsMap.size()){
        // unchanged states will not be printed
        if (!curActions.empty()) {
          std::cout<< "==============> Finish Actions -- State Change <==============" << std::endl;
          //  print current state information
          this->SpecStrategy->refreshCurState();
          this->SaveCurStateInfo(this->SpecStrategy->curStateID);
        }
      }
      std::cout << "==============> Recv Env Change : "<< backupMap["enemy"];
      std::cout << "\t" << backupMap["ground_casualty"] <<"<==============" << std::endl;
      tmp = this->SpecStrategy->runSingleTransition(backupMap);
      std::cout << "Env change, Transition needs Actions : ";
      printStringVector(tmp);
      // run the actuators to do actions
      this->curActions.assign(tmp.begin(), tmp.end());
      this->doneActionsMap.clear();
      std::cout<< "==============> Do Actions to Env Change <==============" << std::endl;
      for (std::string act : this->curActions)
        this->RunActuators(act, 0);
    }
    // env not change, actions not finish, then check the actuators -- no state change
    else if (!envFlag && this->curActions.size() != this->doneActionsMap.size()){
      // set the flag for checking
      for (std::string act : this->curActions)
          this->RunActuators(act, 1);
    }

    // check whether the actions are finished -- state change
    if (this->curActions.size() == this->doneActionsMap.size()) {
      //  refresh state after the acts are executed
      this->SpecStrategy->refreshCurState();

      tmp = this->SpecStrategy->runSingleTransition(this->SpecStrategy->curSensorMap);
      // if the state stay unchanged, no actions will be executed because they are done
      if (this->SpecStrategy->nextStateID != this->SpecStrategy->curStateID){
        //  print current state information
        std::cout<< "==============> Finish Actions -- State Change <==============" << std::endl;
        this->SaveCurStateInfo(this->SpecStrategy->curStateID);
        std::cout << "Transition needs actions : ";
        printStringVector(tmp);

        // run the actuators to do actions
        this->curActions.assign(tmp.begin(), tmp.end());
        this->doneActionsMap.clear();
        for (std::string act : this->curActions)
          this->RunActuators(act, 0);
      }
      else {
        // record the state if env change actions are finished
        if (envFlag) {
          std::cout<< "==============> Finish Actions -- State Change <==============" << std::endl;
          this->SaveCurStateInfo(this->SpecStrategy->curStateID);
        }
        this->doneActionsMap.clear();
        this->curActions.clear();
      }
    }

    // // set frequency as 2hz
    // sleep(0.5);

    // invocate the controller for next loop
    this->#base#Callbackout_out(0);

    return false;
  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void #Base#Executor ::
    SET_ENV_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        Ref::EnvSensor name,
        Ref::EnvValue value
    )
  {
    // TODO
    I8 val;
    switch (name.e) {
      case EnvSensor::enemy:
          val = (value.e == EnvValue::TRUE) ? 1 : 0;
          if (this->SpecStrategy->curSensorMap["enemy"] != val)
            this->envChange = true;
          this->SpecStrategy->curSensorMap["enemy"] = val;
          this->log_ACTIVITY_HI_ENV_UPDATE("enemy", val == 1);
          break;
      case EnvSensor::ground_casualty:
          val = (value.e == EnvValue::TRUE) ? 1 : 0;
          if (this->SpecStrategy->curSensorMap["ground_casualty"] != val)
            this->envChange = true;
          this->SpecStrategy->curSensorMap["ground_casualty"] = val;
          this->log_ACTIVITY_HI_ENV_UPDATE("ground_casualty", val == 1);
          break;
      default:
          // FW_ASSERT(0, name.e);
          break;
      }
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

  // ----------------------------------------------------------------------
  // Customized Functions
  // ----------------------------------------------------------------------
  void #Base#Executor ::
    RunActuators(
        std::string action,
        I8 flag
    )
  {
    // flag 0:doAct; flag 1:checkAct; flag 2:stopAll. first do region move
    if (action.compare("attack") == 0){
      if (this->#base#_attack_out_out(0, flag) == 1)
        this->doneActionsMap[action] = 1;
    }
    else if (action.compare("!attack") == 0){
      if (this->#base#_attack_out_out(0, flag + 2) == 1)
        this->doneActionsMap[action] = 1;
    }
    else if (action.compare("attack_over") == 0){
      if (this->#base#_attack_over_out_out(0, flag) == 1)
        this->doneActionsMap[action] = 1;
    }
    else if (action.compare("!attack_over") == 0){
      if (this->#base#_attack_over_out_out(0, flag + 2) == 1)
        this->doneActionsMap[action] = 1;
    }
    else if (action.compare("take_photo") == 0){
      if (this->#base#_take_photo_out_out(0, flag) == 1)
        this->doneActionsMap[action] = 1;
    }
    else if (action.compare("!take_photo") == 0){
      if (this->#base#_take_photo_out_out(0, flag + 2) == 1)
        this->doneActionsMap[action] = 1;
    }
    // region move
    else{
      if (action.find("!") == 0){
        if (this->#base#Moveout_out(0, flag + 2, action.substr(1).c_str()) == 1)
          this->doneActionsMap[action] = 1;
      }
      else{
        if (this->#base#Moveout_out(0, flag, action.c_str()) == 1)
          this->doneActionsMap[action] = 1;
      }
    }

  }

  void #Base#Executor ::
    SaveCurStateInfo(int state_id)
  {
    State curState = this->SpecStrategy->States[state_id];
    log_ACTIVITY_HI_SHOW_TANSITION(curState.state_id,
      curState.propositions[0].second,
      curState.propositions[1].second,
      curState.propositions[2].second,
      curState.propositions[3].second,
      curState.propositions[4].second,
      curState.propositions[5].second,
      this->SpecStrategy->getCurRegionNamePair().first.c_str(),
      this->SpecStrategy->getCurRegionNamePair().second.c_str());
    this->tlmWrite_state_id(curState.state_id);
    this->tlmWrite_enemy(curState.propositions[0].second);
    this->tlmWrite_ground_casualty(curState.propositions[1].second);
    this->tlmWrite_attack(curState.propositions[2].second);
    this->tlmWrite_attack_over(curState.propositions[3].second);
    this->tlmWrite_take_photo(curState.propositions[4].second);
    this->tlmWrite_attack_mode(curState.propositions[5].second);
    this->tlmWrite_region(this->SpecStrategy->getCurRegionNamePair().first.c_str());
  }

} // end namespace Ref
