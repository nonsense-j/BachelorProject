
"use strict";

let AttitudeCommand = require('./AttitudeCommand.js');
let Supply = require('./Supply.js');
let PositionXYCommand = require('./PositionXYCommand.js');
let HeadingCommand = require('./HeadingCommand.js');
let VelocityZCommand = require('./VelocityZCommand.js');
let Altimeter = require('./Altimeter.js');
let ControllerState = require('./ControllerState.js');
let MotorStatus = require('./MotorStatus.js');
let ServoCommand = require('./ServoCommand.js');
let HeightCommand = require('./HeightCommand.js');
let MotorPWM = require('./MotorPWM.js');
let VelocityXYCommand = require('./VelocityXYCommand.js');
let ThrustCommand = require('./ThrustCommand.js');
let RawMagnetic = require('./RawMagnetic.js');
let YawrateCommand = require('./YawrateCommand.js');
let RuddersCommand = require('./RuddersCommand.js');
let RawRC = require('./RawRC.js');
let RawImu = require('./RawImu.js');
let MotorCommand = require('./MotorCommand.js');
let Compass = require('./Compass.js');
let RC = require('./RC.js');

module.exports = {
  AttitudeCommand: AttitudeCommand,
  Supply: Supply,
  PositionXYCommand: PositionXYCommand,
  HeadingCommand: HeadingCommand,
  VelocityZCommand: VelocityZCommand,
  Altimeter: Altimeter,
  ControllerState: ControllerState,
  MotorStatus: MotorStatus,
  ServoCommand: ServoCommand,
  HeightCommand: HeightCommand,
  MotorPWM: MotorPWM,
  VelocityXYCommand: VelocityXYCommand,
  ThrustCommand: ThrustCommand,
  RawMagnetic: RawMagnetic,
  YawrateCommand: YawrateCommand,
  RuddersCommand: RuddersCommand,
  RawRC: RawRC,
  RawImu: RawImu,
  MotorCommand: MotorCommand,
  Compass: Compass,
  RC: RC,
};
