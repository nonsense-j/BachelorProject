// ======================================================================
// \title  #Base#Executor.hpp
// \author nonsense
// \brief  hpp file for #Base#Executor component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef #Base#Executor_HPP
#define #Base#Executor_HPP

#include "Ref/#Base#Mission/#Base#ExecutorComponentAc.hpp"
#include "Ref/AutParser/autparser.hpp"

namespace Ref {

  class #Base#Executor :
    public #Base#ExecutorComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // SpecStrategy, stop sign
      // ----------------------------------------------------------------------
      Strategy *SpecStrategy;
      bool stopFlag, envChange;
      std::vector<std::string> curActions;
      std::unordered_map<std::string, int> doneActionsMap;

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object #Base#Executor
      //!
      #Base#Executor(
          const char *const compName /*!< The component name*/
      );

      //! Initialize object #Base#Executor
      //!
      void init(
          const NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object #Base#Executor
      //!
      ~#Base#Executor();

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for envSetin
      //!
      void envSetin_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          const nameString &name,
          I8 value
      );

      //! Handler implementation for #base#ExeCtrlin
      //!
      void #base#ExeCtrlin_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          I8 control_id
      );

      //! Handler implementation for #base#Wakein
      //!
      bool #base#Wakein_handler(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Command handler implementations
      // ----------------------------------------------------------------------

      //! Implementation for SET_ENV command handler
      //! Set Sensor value
      void SET_ENV_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          Ref::EnvSensor name,
          Ref::EnvValue value
      );

      // ----------------------------------------------------------------------
      // Customized Functions
      // ----------------------------------------------------------------------
      void RunActuators(
          std::string action,
          I8 flag
      );
      void SaveCurStateInfo(int state_id);


    };

} // end namespace Ref

#endif
