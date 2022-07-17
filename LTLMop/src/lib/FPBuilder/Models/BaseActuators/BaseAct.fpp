module Ref {
  @ Component for customize your own actions
  passive component #Base#_#act# {

    # ----------------------------------------------------------------------
    # General ports
    # ----------------------------------------------------------------------

    sync input port #base#_#act#_in : #Base#Act

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
    event ACTFINISHED() \
      severity activity low \
      format "[#Base#] Action: #act# has finished."

  }

}
