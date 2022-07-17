module Ref {
  @ Component for customize your own actions
  passive component #Base#Move {

    # ----------------------------------------------------------------------
    # General ports
    # ----------------------------------------------------------------------

    sync input port #base#Movein : #Base#Move

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
      severity activity low \
      format "[#Base#] Arrived at {}."

  }

}
