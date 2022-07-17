// ======================================================================
// \title  ResCapExecutor.cpp
// \author nonsense
// \brief  cpp file for ResCapExecutor component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================


#include <Ref/ResCapMission/ResCapExecutor.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Ref {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  ResCapExecutor ::
    ResCapExecutor(
        const char *const compName
    ) : ResCapExecutorComponentBase(compName)
  {

  }

  void ResCapExecutor ::
    init(
        const NATIVE_INT_TYPE queueDepth,
        const NATIVE_INT_TYPE instance
    )
  {
    ResCapExecutorComponentBase::init(queueDepth, instance);
    this->SpecStrategy = new Strategy("ResCapMission/Config/ResCap.spec");
    this->stopFlag = false;
  }

  ResCapExecutor ::
    ~ResCapExecutor()
  {
    delete this->SpecStrategy;
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void ResCapExecutor ::
    envSetin_handler(
        const NATIVE_INT_TYPE portNum,
        const nameString &name,
        I8 value
    )
  {
    // TODO
    this->SpecStrategy->curSensorMap[name.toChar()] = value;
    this->log_ACTIVITY_HI_ENV_UPDATE(name, value == 1);
  }

  void ResCapExecutor ::
    resCapActStatusin_handler(
        const NATIVE_INT_TYPE portNum,
        const actionString &action,
        I8 status
    )
  {
    // status 0:doAct; status 1:checkAct; status 2:stopAct.

  }

  void ResCapExecutor ::
    resCapExeCtrlin_handler(
        const NATIVE_INT_TYPE portNum,
        I8 control_id
    )
  {
    // TODO
    switch (control_id) {
    case 0: {  // start
      this->stopFlag = false;
      this->log_ACTIVITY_LO_INIT_SUCC();
      this->log_ACTIVITY_HI_START_RECV();
      State curState = this->SpecStrategy->States[this->SpecStrategy->curStateID];
      log_ACTIVITY_HI_SHOW_TANSITION(
          curState.state_id, curState.propositions[0].second,
          curState.propositions[1].second, curState.propositions[2].second,
          curState.propositions[3].second, curState.propositions[4].second,
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
      break;
    }
    case 1:   // stop
      this->stopFlag = true;
      break;
    case 2:   // reset
      this->SpecStrategy->setCurState(0);
      this->log_ACTIVITY_HI_RESET_RECV();
      break;
    default:
      break;
    }
  }

  bool ResCapExecutor ::
    resCapWakein_handler(
        const NATIVE_INT_TYPE portNum
    )
  {
    // TODO return
    if (this->stopFlag){
      this->log_ACTIVITY_HI_STOP_RECV();
      return true;
    }

    printStringVector(this->SpecStrategy->runSingleTransition(this->SpecStrategy->curSensorMap));

    //  refresh state after the acts are executed
    this->SpecStrategy->refreshCurState();
    // wait for 10s
    sleep(10);
    //  print current state information
    this->SpecStrategy->printCurStateInfo();

    State curState = this->SpecStrategy->States[this->SpecStrategy->curStateID];
    log_ACTIVITY_HI_SHOW_TANSITION(
        curState.state_id, curState.propositions[0].second,
        curState.propositions[1].second, curState.propositions[2].second,
        curState.propositions[3].second, curState.propositions[4].second,
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

    // invocate the controller for next loop
    this->resCapCallbackout_out(0);

    return false;
  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void ResCapExecutor ::
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
            this->SpecStrategy->curSensorMap["enemy"] = val;
            this->log_ACTIVITY_HI_ENV_UPDATE("enemy", val == 1);
            break;
        case EnvSensor::ground_casualty:
            val = (value.e == EnvValue::TRUE) ? 1 : 0;
            this->SpecStrategy->curSensorMap["ground_casualty"] = val;
            this->log_ACTIVITY_HI_ENV_UPDATE("ground_casualty", val == 1);
            break;
        default:
            FW_ASSERT(0, name.e);
            break;
      }
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

} // end namespace Ref
