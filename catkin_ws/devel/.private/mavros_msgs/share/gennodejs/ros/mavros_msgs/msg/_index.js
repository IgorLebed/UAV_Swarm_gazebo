
"use strict";

let PositionTarget = require('./PositionTarget.js');
let BatteryStatus = require('./BatteryStatus.js');
let RadioStatus = require('./RadioStatus.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let Vibration = require('./Vibration.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let WaypointReached = require('./WaypointReached.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let DebugValue = require('./DebugValue.js');
let MountControl = require('./MountControl.js');
let ParamValue = require('./ParamValue.js');
let LogData = require('./LogData.js');
let RCOut = require('./RCOut.js');
let Thrust = require('./Thrust.js');
let VehicleInfo = require('./VehicleInfo.js');
let HilSensor = require('./HilSensor.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let HomePosition = require('./HomePosition.js');
let LandingTarget = require('./LandingTarget.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let ExtendedState = require('./ExtendedState.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let RCIn = require('./RCIn.js');
let Mavlink = require('./Mavlink.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let HilControls = require('./HilControls.js');
let VFR_HUD = require('./VFR_HUD.js');
let FileEntry = require('./FileEntry.js');
let Altitude = require('./Altitude.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let HilGPS = require('./HilGPS.js');
let WaypointList = require('./WaypointList.js');
let CommandCode = require('./CommandCode.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let ActuatorControl = require('./ActuatorControl.js');
let ManualControl = require('./ManualControl.js');
let StatusText = require('./StatusText.js');
let RTCM = require('./RTCM.js');
let Waypoint = require('./Waypoint.js');
let Param = require('./Param.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let State = require('./State.js');
let LogEntry = require('./LogEntry.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let Trajectory = require('./Trajectory.js');

module.exports = {
  PositionTarget: PositionTarget,
  BatteryStatus: BatteryStatus,
  RadioStatus: RadioStatus,
  HilActuatorControls: HilActuatorControls,
  Vibration: Vibration,
  WheelOdomStamped: WheelOdomStamped,
  WaypointReached: WaypointReached,
  OpticalFlowRad: OpticalFlowRad,
  DebugValue: DebugValue,
  MountControl: MountControl,
  ParamValue: ParamValue,
  LogData: LogData,
  RCOut: RCOut,
  Thrust: Thrust,
  VehicleInfo: VehicleInfo,
  HilSensor: HilSensor,
  TimesyncStatus: TimesyncStatus,
  HomePosition: HomePosition,
  LandingTarget: LandingTarget,
  CamIMUStamp: CamIMUStamp,
  ExtendedState: ExtendedState,
  HilStateQuaternion: HilStateQuaternion,
  RCIn: RCIn,
  Mavlink: Mavlink,
  OverrideRCIn: OverrideRCIn,
  AttitudeTarget: AttitudeTarget,
  HilControls: HilControls,
  VFR_HUD: VFR_HUD,
  FileEntry: FileEntry,
  Altitude: Altitude,
  GlobalPositionTarget: GlobalPositionTarget,
  HilGPS: HilGPS,
  WaypointList: WaypointList,
  CommandCode: CommandCode,
  ADSBVehicle: ADSBVehicle,
  ActuatorControl: ActuatorControl,
  ManualControl: ManualControl,
  StatusText: StatusText,
  RTCM: RTCM,
  Waypoint: Waypoint,
  Param: Param,
  CompanionProcessStatus: CompanionProcessStatus,
  OnboardComputerStatus: OnboardComputerStatus,
  State: State,
  LogEntry: LogEntry,
  EstimatorStatus: EstimatorStatus,
  Trajectory: Trajectory,
};
