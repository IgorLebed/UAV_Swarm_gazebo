
"use strict";

let WaypointPull = require('./WaypointPull.js')
let ParamSet = require('./ParamSet.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let SetMavFrame = require('./SetMavFrame.js')
let CommandInt = require('./CommandInt.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let FileWrite = require('./FileWrite.js')
let FileRead = require('./FileRead.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let LogRequestList = require('./LogRequestList.js')
let FileRemove = require('./FileRemove.js')
let FileChecksum = require('./FileChecksum.js')
let CommandHome = require('./CommandHome.js')
let FileRename = require('./FileRename.js')
let MountConfigure = require('./MountConfigure.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let FileList = require('./FileList.js')
let SetMode = require('./SetMode.js')
let FileOpen = require('./FileOpen.js')
let CommandTOL = require('./CommandTOL.js')
let FileTruncate = require('./FileTruncate.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let ParamGet = require('./ParamGet.js')
let LogRequestData = require('./LogRequestData.js')
let FileClose = require('./FileClose.js')
let StreamRate = require('./StreamRate.js')
let ParamPush = require('./ParamPush.js')
let ParamPull = require('./ParamPull.js')
let CommandLong = require('./CommandLong.js')
let CommandBool = require('./CommandBool.js')
let WaypointPush = require('./WaypointPush.js')
let MessageInterval = require('./MessageInterval.js')
let FileMakeDir = require('./FileMakeDir.js')
let WaypointClear = require('./WaypointClear.js')

module.exports = {
  WaypointPull: WaypointPull,
  ParamSet: ParamSet,
  CommandTriggerControl: CommandTriggerControl,
  SetMavFrame: SetMavFrame,
  CommandInt: CommandInt,
  LogRequestEnd: LogRequestEnd,
  FileWrite: FileWrite,
  FileRead: FileRead,
  CommandVtolTransition: CommandVtolTransition,
  CommandTriggerInterval: CommandTriggerInterval,
  LogRequestList: LogRequestList,
  FileRemove: FileRemove,
  FileChecksum: FileChecksum,
  CommandHome: CommandHome,
  FileRename: FileRename,
  MountConfigure: MountConfigure,
  FileRemoveDir: FileRemoveDir,
  FileList: FileList,
  SetMode: SetMode,
  FileOpen: FileOpen,
  CommandTOL: CommandTOL,
  FileTruncate: FileTruncate,
  VehicleInfoGet: VehicleInfoGet,
  WaypointSetCurrent: WaypointSetCurrent,
  ParamGet: ParamGet,
  LogRequestData: LogRequestData,
  FileClose: FileClose,
  StreamRate: StreamRate,
  ParamPush: ParamPush,
  ParamPull: ParamPull,
  CommandLong: CommandLong,
  CommandBool: CommandBool,
  WaypointPush: WaypointPush,
  MessageInterval: MessageInterval,
  FileMakeDir: FileMakeDir,
  WaypointClear: WaypointClear,
};
