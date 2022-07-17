module Ref {
  @ Component for controlling the running of automata
  active component #Base#Controller {

    # ----------------------------------------------------------------------
    # General ports
    # ----------------------------------------------------------------------

    @ Port for innovating the executor
    output port #base#Wakeout: #Base#Wake

    @ Port for controlling the executor
    output port #base#ExeCtrlout: #Base#ExeCtrl

    @ Port for entering next loop
    async input port #base#Callbackin: #Base#Callback

    @ Port for receiving controlling cmd
    async input port simControlin: SimControl

    # ----------------------------------------------------------------------
    # Special ports
    # ----------------------------------------------------------------------

    @ Command receive port
    command recv port cmdIn

    @ Command registration port
    command reg port cmdRegOut

    @ Command response port
    command resp port cmdResponseOut

    @ Event port
    event port eventOut

    @ Telemetry port
    telemetry port tlmOut

    @ Text event port
    text event port textEventOut

    @ Time get port
    time get port timeGetOut

    # ----------------------------------------------------------------------
    # Commands
    # ----------------------------------------------------------------------

    @ Start single simulation
    async command START()

    @ Stop single simulation
    async command STOP()

    @ Reset single simulation
    async command RESET()

    # ----------------------------------------------------------------------
    # Events
    # ----------------------------------------------------------------------

    @ Print the index of simulation
    event LOOPINFO(
                  num: I32 @< The loop index
                ) \
      severity activity low \
      format "[#Base#] Transition NO.{d} Starts."

    # ----------------------------------------------------------------------
    # Telemetry
    # ----------------------------------------------------------------------

    @ The index of transition
    telemetry num: I32

  }

}
