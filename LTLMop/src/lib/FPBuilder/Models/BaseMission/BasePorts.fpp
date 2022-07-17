module Ref {

  @ Port connecting controller(Loop) and executor(single transition)
  port #Base#Wake() -> bool

  @ Port for calling back to controoler to activate loop
  port #Base#Callback()

  @ Port to ctrl executor(0:start, 1:stop, 2:reset)
  port #Base#ExeCtrl(control_id: I8)

  @ Port to invocate actuators for different actions.flag 0:doAct; flag 1:checkAct; flag 2:stopAct. return 1:finished.
  port #Base#Act(flag: I8) -> I8

  @ Port to invocate actuators for going to regions. flag 0:doAct; flag 1:checkAct; flag 2:stopAct. return 1:finished.
  port #Base#Move(
               flag: I8
               #base#region: string
             ) -> I8

}
