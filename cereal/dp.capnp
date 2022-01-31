using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0xbfa7e645486440c7;

# dp
struct DragonConf {
  dpAtl @0 :Bool;
  dpAtlOpLong @1 :Bool;
  dpDashcamd @2 :Bool;
  dpAutoShutdown @3 :Bool;
  dpAthenad @4 :Bool;
  dpUploader @5 :Bool;
  dpLateralMode @6 :UInt8;
  dpSignalOffDelay @7 :Float32;
  dpLcMinMph @8 :UInt8;
  dpLcAutoMinMph @9 :UInt8;
  dpLcAutoDelay @10 :Float32;
  dpLaneLessModeCtrl @11 :Bool;
  dpLaneLessMode @12 :UInt8;
  dpAllowGas @13 :Bool;
  dpAccelProfileCtrl @14 :Bool;
  dpAccelProfile @15 :UInt8;
  dpGearCheck @16 :Bool;
  dpSpeedCheck @17 :Bool;
  dpUiDisplayMode @18 :UInt8;
  dpUiSpeed @19 :Bool;
  dpUiEvent @20 :Bool;
  dpUiMaxSpeed @21 :Bool;
  dpUiFace @22 :Bool;
  dpUiLane @23 :Bool;
  dpUiLead @24 :Bool;
  dpUiSide @25 :Bool;
  dpUiTop @26 :Bool;
  dpUiBlinker @27 :Bool;
  dpUiBrightness @28 :UInt8;
  dpUiVolume @29 :Int8;
  dpToyotaLdw @30 :Bool;
  dpToyotaSng @31 :Bool;
  dpToyotaCruiseOverride @32 :Bool;
  dpToyotaCruiseOverrideVego @33 :Bool;
  dpToyotaCruiseOverrideAt @34 :Float32;
  dpToyotaCruiseOverrideSpeed @35 :Float32;
  dpIpAddr @36 :Text;
  dpCameraOffset @37 :Int8;
  dpPathOffset @38 :Int8;
  dpLocale @39 :Text;
  dpSrLearner @40 :Bool;
  dpSrCustom @41 :Float32;
  dpMapd @42 :Bool;
}


# use on mapd
struct LiveMapData {
  speedLimitValid @0 :Bool;
  speedLimit @1 :Float32;
  speedLimitAheadValid @2 :Bool;
  speedLimitAhead @3 :Float32;
  speedLimitAheadDistance @4 :Float32;
  turnSpeedLimitValid @5 :Bool;
  turnSpeedLimit @6 :Float32;
  turnSpeedLimitEndDistance @7 :Float32;
  turnSpeedLimitSign @8 :Int16;
  turnSpeedLimitsAhead @9 :List(Float32);
  turnSpeedLimitsAheadDistances @10 :List(Float32);
  turnSpeedLimitsAheadSigns @11 :List(Int16);
  lastGpsTimestamp @12 :Int64;  # Milliseconds since January 1, 1970.
  currentRoadName @13 :Text;
  lastGpsLatitude @14 :Float64;
  lastGpsLongitude @15 :Float64;
  lastGpsSpeed @16 :Float32;
  lastGpsBearingDeg @17 :Float32;
  lastGpsAccuracy @18 :Float32;
  lastGpsBearingAccuracyDeg @19 :Float32;
}
