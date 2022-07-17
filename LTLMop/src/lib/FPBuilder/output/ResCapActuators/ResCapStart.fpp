module Ref {
  @ Component for customize your own actions
  passive component ResCapStart {

    # ----------------------------------------------------------------------
    # General ports
    # ----------------------------------------------------------------------

    @ Port for innovating the executor
    output port resCapActStatusout : ResCapActStatus

    sync input port resCapStartin : ResCapAct


    # ----------------------------------------------------------------------
    # Special ports
    # ----------------------------------------------------------------------

    @ Event port
    event port eventOut

    @ Telemetry port
    telemetry port tlmOut

    @ Text event port
    text event port textEventOut

    @ Time get port
    time get port timeGetOut

    # ----------------------------------------------------------------------
    # Events
    # ----------------------------------------------------------------------

    @ Print the index of simulation
    event STARTFINISHED() \
      severity activity high \
      format "[ResCap] Starting actions have finished."

  }

}
