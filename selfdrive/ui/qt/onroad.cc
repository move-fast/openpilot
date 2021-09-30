#include "selfdrive/ui/qt/onroad.h"

#include <QDebug>
#include <iomanip>

#include "selfdrive/common/timing.h"
#include "selfdrive/ui/paint.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/api.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map.h"
#endif


OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(bdr_s);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  nvg = new NvgWindow(VISION_STREAM_RGB_BACK, this);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addWidget(nvg);

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(this, &OnroadWindow::updateStateSignal, this, &OnroadWindow::updateState);
  QObject::connect(this, &OnroadWindow::offroadTransitionSignal, this, &OnroadWindow::offroadTransition);
}

void OnroadWindow::updateState(const UIState &s) {
  SubMaster &sm = *(s.sm);
  QColor bgColor = bg_colors[s.status];
  if ((sm.frame - s.scene.display_debug_alert_frame) <= 1 * UI_FREQ) {
    alerts->updateAlert(DEBUG_SNAPSHOT_ALERT, bgColor);
  } 
  else if (sm.updated("controlsState")) {
    const cereal::ControlsState::Reader &cs = sm["controlsState"].getControlsState();
    alerts->updateAlert({QString::fromStdString(cs.getAlertText1()),
                 QString::fromStdString(cs.getAlertText2()),
                 QString::fromStdString(cs.getAlertType()),
                 cs.getAlertSize(), cs.getAlertSound()}, bgColor);
  } else if ((sm.frame - s.scene.started_frame) > 5 * UI_FREQ) {
    // Handle controls timeout
    if (sm.rcv_frame("controlsState") < s.scene.started_frame) {
      // car is started, but controlsState hasn't been seen at all
      alerts->updateAlert(CONTROLS_WAITING_ALERT, bgColor);
    } else if ((nanos_since_boot() - sm.rcv_time("controlsState")) / 1e9 > CONTROLS_TIMEOUT) {
      // car is started, but controls is lagging or died
      bgColor = bg_colors[STATUS_ALERT];
      alerts->updateAlert(CONTROLS_UNRESPONSIVE_ALERT, bgColor);
    }
  }
  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }
}

void issue_debug_snapshot(SubMaster &sm) {
  auto longitudinal_plan = sm["longitudinalPlan"].getLongitudinalPlan();
  auto live_map_data = sm["liveMapData"].getLiveMapData();
  auto car_state = sm["carState"].getCarState();

  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream param_name_os;
  param_name_os << std::put_time(&tm, "%Y-%m-%d--%H-%M-%S");

  std::ostringstream os;
  os.setf(std::ios_base::fixed);
  os.precision(2);
  os << "Datetime: " << param_name_os.str() << ", vEgo: " << car_state.getVEgo() * 3.6 << "\n\n";
  os.precision(6);
  os << "Location: (" << live_map_data.getLastGpsLatitude() << ", " << live_map_data.getLastGpsLongitude()  << ")\n";
  os.precision(2);
  os << "Bearing: " << live_map_data.getLastGpsBearingDeg() << "; ";
  os << "GPSSpeed: " << live_map_data.getLastGpsSpeed() * 3.6 << "\n\n";
  os.precision(1);
  os << "Speed Limit: " << live_map_data.getSpeedLimit() * 3.6 << ", ";
  os << "Valid: " << live_map_data.getSpeedLimitValid() << "\n";
  os << "Speed Limit Ahead: " << live_map_data.getSpeedLimitAhead() * 3.6 << ", ";
  os << "Valid: " << live_map_data.getSpeedLimitAheadValid() << ", ";
  os << "Distance: " << live_map_data.getSpeedLimitAheadDistance() << "\n";
  os << "Turn Speed Limit: " << live_map_data.getTurnSpeedLimit() * 3.6 << ", ";
  os << "Valid: " << live_map_data.getTurnSpeedLimitValid() << ", ";
  os << "End Distance: " << live_map_data.getTurnSpeedLimitEndDistance() << ", ";
  os << "Sign: " << live_map_data.getTurnSpeedLimitSign() << "\n\n";

  const auto turn_speeds = live_map_data.getTurnSpeedLimitsAhead();
  os << "Turn Speed Limits Ahead:\n";
  os << "VALUE\tDIST\tSIGN\n";

  if (turn_speeds.size() == 0) {
    os << "-\t-\t-" << "\n\n";
  } else {
    const auto distances = live_map_data.getTurnSpeedLimitsAheadDistances();
    const auto signs = live_map_data.getTurnSpeedLimitsAheadSigns();
    for(int i = 0; i < turn_speeds.size(); i++) {
      os << turn_speeds[i] * 3.6 << "\t" << distances[i] << "\t" << signs[i] << "\n";
    }
    os << "\n";
  }

  os << "SPEED LIMIT CONTROLLER:\n";
  os << "sl: " << longitudinal_plan.getSpeedLimit() * 3.6  << ", ";
  os << "state: " << int(longitudinal_plan.getSpeedLimitControlState()) << ", ";
  os << "isMap: " << longitudinal_plan.getIsMapSpeedLimit() << "\n\n";

  os << "TURN SPEED CONTROLLER:\n";
  os << "speed: " << longitudinal_plan.getTurnSpeed() * 3.6 << ", ";
  os << "state: " << int(longitudinal_plan.getTurnSpeedControlState()) << "\n\n";

  os << "VISION TURN CONTROLLER:\n";
  os << "speed: " << longitudinal_plan.getVisionTurnSpeed() * 3.6 << ", ";
  os << "state: " << int(longitudinal_plan.getVisionTurnControllerState()); 

  Params().put(param_name_os.str().c_str(), os.str().c_str(), os.str().length());
  QUIState::ui_state.scene.display_debug_alert_frame = sm.frame;
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
  bool sidebarVisible = geometry().x() > 0;
  bool propagate_event = true;

    // Toggle speed limit control enabled
  SubMaster &sm = *(QUIState::ui_state.sm);
  auto longitudinal_plan = sm["longitudinalPlan"].getLongitudinalPlan();
  
  Rect speed_limit_touch_rect = QUIState::ui_state.scene.speed_limit_sign_touch_rect;
  Rect debug_tap_rect = {rect().center().x() - 200, rect().center().y() - 200, 400, 400};

  if (longitudinal_plan.getSpeedLimit() > 0.0 && speed_limit_touch_rect.ptInRect(e->x(), e->y())) {
    // If touching the speed limit sign area when visible
    QUIState::ui_state.scene.last_speed_limit_sign_tap = seconds_since_boot();
    QUIState::ui_state.scene.speed_limit_control_enabled = !QUIState::ui_state.scene.speed_limit_control_enabled;
    Params().putBool("SpeedLimitControl", QUIState::ui_state.scene.speed_limit_control_enabled);
    propagate_event = false;
  }
  else if (QUIState::ui_state.scene.debug_snapshot_enabled && debug_tap_rect.ptInRect(e->x(), e->y())) {
    issue_debug_snapshot(*(QUIState::ui_state.sm));
    propagate_event = false;
  }
  else if (map != nullptr) {
    map->setVisible(!sidebarVisible && !map->isVisible());
  }

  // propagation event to parent(HomeWindow)
  if (propagate_event) {
    QWidget::mousePressEvent(e);
  }
}

void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad) {
    if (map == nullptr && QUIState::ui_state.has_prime) {
      QMapboxGLSettings settings;

      // Valid for 4 weeks since we can't swap tokens on the fly
      QString token = CommaApi::create_jwt({}, 4 * 7 * 24 * 3600);

      if (!Hardware::PC()) {
        settings.setCacheDatabasePath("/data/mbgl-cache.db");
      }
      settings.setApiBaseUrl(MAPS_HOST);
      settings.setCacheDatabaseMaximumSize(20 * 1024 * 1024);
      settings.setAccessToken(token.trimmed());

      MapWindow * m = new MapWindow(settings);
      m->setFixedWidth(width() / 2 - bdr_s);
      QObject::connect(this, &OnroadWindow::offroadTransitionSignal, m, &MapWindow::offroadTransition);
      split->addWidget(m, 0, Qt::AlignRight);
      map = m;
    }
  }
#endif

  alerts->updateAlert({}, bg);

  // update stream type
  bool wide_cam = Hardware::TICI() && Params().getBool("EnableWideCamera");
  nvg->setStreamType(wide_cam ? VISION_STREAM_RGB_WIDE : VISION_STREAM_RGB_BACK);
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));
}

// ***** onroad widgets *****

void OnroadAlerts::updateAlert(const Alert &a, const QColor &color) {
  if (!alert.equal(a) || color != bg) {
    alert = a;
    bg = color;
    update();
  }
}

void OnroadAlerts::paintEvent(QPaintEvent *event) {
  if (alert.size == cereal::ControlsState::AlertSize::NONE) {
    return;
  }
  static std::map<cereal::ControlsState::AlertSize, const int> alert_sizes = {
    {cereal::ControlsState::AlertSize::SMALL, 271},
    {cereal::ControlsState::AlertSize::MID, 420},
    {cereal::ControlsState::AlertSize::FULL, height()},
  };
  int h = alert_sizes[alert.size];
  QRect r = QRect(0, height() - h, width(), h);

  QPainter p(this);

  // draw background + gradient
  p.setPen(Qt::NoPen);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  p.setBrush(QBrush(bg));
  p.drawRect(r);

  QLinearGradient g(0, r.y(), 0, r.bottom());
  g.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.05));
  g.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0.35));

  p.setCompositionMode(QPainter::CompositionMode_DestinationOver);
  p.setBrush(QBrush(g));
  p.fillRect(r, g);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  // text
  const QPoint c = r.center();
  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setRenderHint(QPainter::TextAntialiasing);
  if (alert.size == cereal::ControlsState::AlertSize::SMALL) {
    configFont(p, "Open Sans", 74, "SemiBold");
    p.drawText(r, Qt::AlignCenter, alert.text1);
  } else if (alert.size == cereal::ControlsState::AlertSize::MID) {
    configFont(p, "Open Sans", 88, "Bold");
    p.drawText(QRect(0, c.y() - 125, width(), 150), Qt::AlignHCenter | Qt::AlignTop, alert.text1);
    configFont(p, "Open Sans", 66, "Regular");
    p.drawText(QRect(0, c.y() + 21, width(), 90), Qt::AlignHCenter, alert.text2);
  } else if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    bool l = alert.text1.length() > 15;
    configFont(p, "Open Sans", l ? 132 : 177, "Bold");
    p.drawText(QRect(0, r.y() + (l ? 240 : 270), width(), 600), Qt::AlignHCenter | Qt::TextWordWrap, alert.text1);
    configFont(p, "Open Sans", 88, "Regular");
    p.drawText(QRect(0, r.height() - (l ? 361 : 420), width(), 300), Qt::AlignHCenter | Qt::TextWordWrap, alert.text2);
  }
}

void NvgWindow::initializeGL() {
  CameraViewWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  ui_nvg_init(&QUIState::ui_state);
  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);
}

void NvgWindow::paintGL() {
  CameraViewWidget::paintGL();
  ui_draw(&QUIState::ui_state, width(), height());

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  if (dt > 66) {
    // warn on sub 15fps
    LOGW("slow frame time: %.2f", dt);
  }
  prev_draw_t = cur_draw_t;
}
