// ======================================================================
// \title  #Base#Controller.cpp
// \author nonsense
// \brief  cpp file for #Base#Controller component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================


#include <Ref/#Base#Mission/#Base#Controller.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Ref {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  #Base#Controller ::
    #Base#Controller(
        const char *const compName
    ) : #Base#ControllerComponentBase(compName)
  {

  }

  void #Base#Controller ::
    init(
        const NATIVE_INT_TYPE queueDepth,
        const NATIVE_INT_TYPE instance
    )
  {
    #Base#ControllerComponentBase::init(queueDepth, instance);
    this->loopIndex = 0;
  }

  #Base#Controller ::
    ~#Base#Controller()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void #Base#Controller ::
    #base#Callbackin_handler(
        const NATIVE_INT_TYPE portNum
    )
  {
    // TODO
    this->tlmWrite_num(++this->loopIndex);
    // this->log_ACTIVITY_LO_LOOPINFO(this->loopIndex);
    // display single transition
    if (!this->skipOnce)
      this->skipOnce = this->#base#Wakeout_out(0);
  }

  void #Base#Controller ::
    simControlin_handler(
        const NATIVE_INT_TYPE portNum,
        I8 control_id
    )
  {
    // TODO
    switch (control_id) {
    case 0:   // start
      this->#base#ExeCtrlout_out(0, 0);
      // display single transition
      if (this->skipOnce && this->loopIndex != 0)
        this->loopIndex--;
      this->tlmWrite_num(++this->loopIndex);
      // this->log_ACTIVITY_LO_LOOPINFO(this->loopIndex);
      this->skipOnce = this->#base#Wakeout_out(0);
      break;
    case 1:   // stop
      this->#base#ExeCtrlout_out(0, 1);
      break;
    case 2:   // reset
      this->#base#ExeCtrlout_out(0, 2);
      this->loopIndex = 0;
      break;
    default:
      break;
    }
  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void #Base#Controller ::
    START_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
    // TODO
    this->#base#ExeCtrlout_out(0, 0);
    // display single transition
    if (this->skipOnce && this->loopIndex != 0)
      this->loopIndex--;
    this->tlmWrite_num(++this->loopIndex);
    this->log_ACTIVITY_LO_LOOPINFO(this->loopIndex);
    this->skipOnce = this->#base#Wakeout_out(0);
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

  void #Base#Controller ::
    STOP_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
    // TODO
    this->#base#ExeCtrlout_out(0, 1);
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

  void #Base#Controller ::
    RESET_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
    // TODO
    this->#base#ExeCtrlout_out(0, 2);
    this->loopIndex = 0;
    this->cmdResponse_out(opCode,cmdSeq,Fw::CmdResponse::OK);
  }

} // end namespace Ref
