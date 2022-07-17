module Ref {
  @ Component for customize your own actions
  active component ResCapMove {

    # ----------------------------------------------------------------------
    # General ports
    # ----------------------------------------------------------------------

    @ Port for innovating the executor
    output port resCapActStatusout : ResCapActStatus

    async input port resCapMovein : ResCapAct


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

    @ arrived at region
    event MOVEFINISHED(
                  region: string
                ) \
      severity activity high \
      format "[ResCap] Arrived at {}."

  }

}
