module Ref {

  @ Component for receiving and performing a math operation
  active component #Base#Executor {
 
    # ----------------------------------------------------------------------
    # General ports
    # ----------------------------------------------------------------------

    @ Port for receiving the operation request
    sync input port #base#Wakein: #Base#Wake

    @ Port for env setting
    async input port envSetin: EnvSet

    @ Port for controlling the executor
    sync input port #base#ExeCtrlin: #Base#ExeCtrl

    @ Port for entering next loop by calling back to controller
    output port #base#Callbackout: #Base#Callback

    output port #base#Startout : #Base#Act

    output port #base#Moveout : #Base#Move
    ## add your output ports for actions ##
    # ----------------------------------------------------------------------
    # Special ports
    # ----------------------------------------------------------------------

    @ Command receive
    command recv port cmdIn

    @ Command registration
    command reg port cmdRegOut

    @ Command response
    command resp port cmdResponseOut

    @ Event
    event port eventOut

    @ Telemetry
    telemetry port tlmOut

    @ Text event
    text event port textEventOut

    @ Time get
    time get port timeGetOut

    # ----------------------------------------------------------------------
    # Events
    # ----------------------------------------------------------------------

    @ init_succ
    event INIT_SUCC() \
      severity activity low \
      format "[#Base#] Stategy initializes successfully. Specification is located at : [Ref/#Base#Mission/Config/#Base#.spec]." \

    @ Transition Info: evrytime entering a transition, state info should be emitted
    event SHOW_TANSITION(
                          state_id: I32 @< state id
                          ## add your props ##
                          region: string @< current region name
                          region_base: string @< current region real name
                        ) \
      severity activity high \
      format "[#Base#] State #{d} ==> ## add your format ##; @Region: {}({})." \

    @ env update
    event ENV_UPDATE(
                  name: string
                  value: bool
                ) \
      severity activity high \
      format ">> [#Base#] Environment Sensor Updates: {} = {}"
        
    @ Start Simulation
    event START_RECV() \
      severity activity high \
      format "[#Base#] Simulation Starts --- "

    @ Stop Simulation
    event STOP_RECV() \
      severity activity high \
      format "[#Base#] Simulation Stops --- "

    @ Reset Simulation
    event RESET_RECV() \
      severity activity high \
      format "[#Base#] Simulation Resets --- "

    # ----------------------------------------------------------------------
    # Commands
    # ----------------------------------------------------------------------

    @ Set Sensor value
    async command SET_ENV(
                           name: EnvSensor
                           value: EnvValue
                         )

    # ----------------------------------------------------------------------
    # Telemetry
    # ----------------------------------------------------------------------

    telemetry state_id: I32
    ## add your telemetry ##
    telemetry region: string

  }

}
