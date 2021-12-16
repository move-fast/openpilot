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
  dpFollowingProfileCtrl @14 :Bool;
  dpFollowingProfile @15 :UInt8;
  dpAccelProfileCtrl @16 :Bool;
  dpAccelProfile @17 :UInt8;
  dpGearCheck @18 :Bool;
  dpSpeedCheck @19 :Bool;
  dpUiDisplayMode @20 :UInt8;
  dpUiSpeed @21 :Bool;
  dpUiEvent @22 :Bool;
  dpUiMaxSpeed @23 :Bool;
  dpUiFace @24 :Bool;
  dpUiLane @25 :Bool;
  dpUiLead @26 :Bool;
  dpUiSide @27 :Bool;
  dpUiTop @28 :Bool;
  dpUiBlinker @29 :Bool;
  dpUiBrightness @30 :UInt8;
  dpUiVolume @31 :Int8;
  dpToyotaLdw @32 :Bool;
  dpToyotaSng @33 :Bool;
  dpToyotaCruiseOverride @34 :Bool;
  dpToyotaCruiseOverrideVego @35 :Bool;
  dpToyotaCruiseOverrideAt @36 :Float32;
  dpToyotaCruiseOverrideSpeed @37 :Float32;
  dpIpAddr @38 :Text;
  dpCameraOffset @39 :Int8;
  dpPathOffset @40 :Int8;
  dpLocale @41 :Text;
  dpSrLearner @42 :Bool;
  dpSrCustom @43 :Float32;
  dpMapd @44 :Bool;
}
