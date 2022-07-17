// ======================================================================
// \title  #Base#Controller.hpp
// \author nonsense
// \brief  hpp file for #Base#Controller component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef #Base#Controller_HPP
#define #Base#Controller_HPP

#include "Ref/#Base#Mission/#Base#ControllerComponentAc.hpp"

namespace Ref {

  class #Base#Controller :
    public #Base#ControllerComponentBase
  {

    public:

      I32 loopIndex;
      bool skipOnce;

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object #Base#Controller
      //!
      #Base#Controller(
          const char *const compName /*!< The component name*/
      );

      //! Initialize object #Base#Controller
      //!
      void init(
          const NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object #Base#Controller
      //!
      ~#Base#Controller();

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for #base#Callbackin
      //!
      void #base#Callbackin_handler(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Handler implementation for simControlin
      //!
      void simControlin_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          I8 control_id 
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Command handler implementations
      // ----------------------------------------------------------------------

      //! Implementation for START command handler
      //! Start single simulation
      void START_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      //! Implementation for STOP command handler
      //! Stop single simulation
      void STOP_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      //! Implementation for RESET command handler
      //! Reset single simulation
      void RESET_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );


    };

} // end namespace Ref

#endif
