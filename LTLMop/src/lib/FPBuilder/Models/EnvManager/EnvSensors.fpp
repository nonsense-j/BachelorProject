module Ref {

  @ A env sensor list
  enum EnvSensor {
    ## add your sensors ##
  }

  enum EnvValue {
      TRUE
      FALSE
  }

  @ Port for env management
  port EnvSet(
               name: string
               value: I8
             )

}