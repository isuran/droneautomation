
"use strict";

let MissionWaypoint = require('./MissionWaypoint.js');
let MobileData = require('./MobileData.js');
let LandingActivation = require('./LandingActivation.js');
let MissionHotpointTask = require('./MissionHotpointTask.js');
let MissionWaypointTask = require('./MissionWaypointTask.js');
let Gimbal = require('./Gimbal.js');
let Waypoint = require('./Waypoint.js');
let landingActivation = require('./landingActivation.js');
let MissionWaypointAction = require('./MissionWaypointAction.js');
let WaypointList = require('./WaypointList.js');
let LandingData = require('./LandingData.js');

module.exports = {
  MissionWaypoint: MissionWaypoint,
  MobileData: MobileData,
  LandingActivation: LandingActivation,
  MissionHotpointTask: MissionHotpointTask,
  MissionWaypointTask: MissionWaypointTask,
  Gimbal: Gimbal,
  Waypoint: Waypoint,
  landingActivation: landingActivation,
  MissionWaypointAction: MissionWaypointAction,
  WaypointList: WaypointList,
  LandingData: LandingData,
};
