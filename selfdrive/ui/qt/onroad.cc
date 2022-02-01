#include "selfdrive/ui/qt/onroad.h"

#include <cmath>

#include <QDebug>

#include "selfdrive/common/timing.h"
#include "selfdrive/ui/qt/util.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#endif
#include <chrono>
#include <string>
namespace sc = std::chrono;
using std::to_string;


OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(bdr_s);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  QStackedLayout *road_view_layout = new QStackedLayout;
  road_view_layout->setStackingMode(QStackedLayout::StackAll);
  nvg = new NvgWindow(VISION_STREAM_RGB_BACK, this);
  road_view_layout->addWidget(nvg);
  hud = new OnroadHud(this);
  road_view_layout->addWidget(hud);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addLayout(road_view_layout);

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
  QColor bgColor = bg_colors[s.status];
  Alert alert = Alert::get(*(s.sm), s.scene.started_frame);
  if (s.sm->updated("controlsState") || !alert.equal({})) {
    if (alert.type == "controlsUnresponsive") {
      bgColor = bg_colors[STATUS_ALERT];
    }
    alerts->updateAlert(alert, bgColor);
  }

  hud->updateState(s);

  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }
}

// dp - action when btn is triggered
void OnroadWindow::btnTriggered(std::string param, int val, int min, int max) {
  Params p = Params();
  val += 1;
  val = (val > max)? min : val;
  p.put(param, std::to_string(val));
  // touch dp_last_modified
  auto time = sc::system_clock::now(); // get the current time
  auto since_epoch = time.time_since_epoch(); // get the duration since epoch
  auto millis = sc::duration_cast<sc::milliseconds>(since_epoch);
  p.put("dp_last_modified", std::to_string(millis.count()));
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
  bool sidebarVisible = geometry().x() > 0;
  bool propagate_event = true;

  int x = rect().right() - dp_sidebar_width - bdr_s * 2; // from drawBtn init
  const int y = height() - bdr_s*2 - dp_btn_wh - 20; // from drawBtn init
  const int x_offset = dp_btn_wh + dp_btn_wh_gap;

  //if (propagate_event && QUIState::ui_state.scene.dpFollowingProfileCtrl) {
  //  x -= x_offset;
  //  QRect fp_rc = QRect(x, y, dp_btn_wh, dp_btn_wh);
  //  if (fp_rc.contains(e->x(), e->y())) {
  //    btnTriggered("dp_following_profile", QUIState::ui_state.scene.dpFollowingProfile, 0, 3);
  //    propagate_event = false;
  //  }
  //}

  if (propagate_event && QUIState::ui_state.scene.dpAccelProfileCtrl) {
    x -= x_offset;
    QRect ap_rc = QRect(x, y, dp_btn_wh, dp_btn_wh);
    if (ap_rc.contains(e->x(), e->y())) {
      btnTriggered("dp_accel_profile", QUIState::ui_state.scene.dpAccelProfile, 0, 2);
      propagate_event = false;
    }
  }

  if (propagate_event && QUIState::ui_state.scene.dpLaneLessModeCtrl) {
    x -= x_offset;
    QRect dlp_rc = QRect(x, y, dp_btn_wh, dp_btn_wh);
    if (dlp_rc.contains(e->x(), e->y())) {
      btnTriggered("dp_lane_less_mode", QUIState::ui_state.scene.dpLaneLessMode, 0, 2);
      propagate_event = false;
    }
  }

  // nav show
  if (propagate_event) { // add this if so we can handle dp btn
    // Toggle speed limit control enabled
    SubMaster &sm = *(QUIState::ui_state.sm);
    auto longitudinal_plan = sm["longitudinalPlan"].getLongitudinalPlan();
    const QRect speed_limit_touch_rect = speed_sgn_rc.adjusted(-50, -50, 50, 50);

    if (longitudinal_plan.getSpeedLimit() > 0.0 && speed_limit_touch_rect.contains(e->x(), e->y())) {
      // If touching the speed limit sign area when visible
      QUIState::ui_state.scene.last_speed_limit_sign_tap = seconds_since_boot();
      QUIState::ui_state.scene.speed_limit_control_enabled = !QUIState::ui_state.scene.speed_limit_control_enabled;
      Params().putBool("SpeedLimitControl", QUIState::ui_state.scene.speed_limit_control_enabled);
      propagate_event = false;
    }
  }
  if (propagate_event && map != nullptr) {
    map->setVisible(!sidebarVisible && !map->isVisible());
  }

  // propagation event to parent(HomeWindow)
  if (propagate_event) {
    QWidget::mousePressEvent(e);
  }
}

void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad && !Params().getBool("dp_panda_no_gps")) {
    bool dp_nav_enabled = Params().getBool("dp_otisserv") && QString::fromStdString(Params().get("dp_nav_mapbox_token_sk")) != "";
    if (map == nullptr && (QUIState::ui_state.has_prime || !MAPBOX_TOKEN.isEmpty() || dp_nav_enabled)) {
      MapWindow * m = new MapWindow(get_mapbox_settings());
      if (Params().getBool("dp_nav_full_screen")) {
        m->setFixedWidth(topWidget(this)->width());
      }
      else {
        m->setFixedWidth(topWidget(this)->width() / 2);
      }
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

// OnroadAlerts
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

// OnroadHud
OnroadHud::OnroadHud(QWidget *parent) : QWidget(parent) {
  engage_img = QPixmap("../assets/img_chffr_wheel.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  dm_img = QPixmap("../assets/img_driver_face.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  map_img = QPixmap("../assets/img_world_icon.png").scaled(subsign_img_size, subsign_img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  left_img = QPixmap("../assets/img_turn_left_icon.png").scaled(subsign_img_size, subsign_img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  right_img = QPixmap("../assets/img_turn_right_icon.png").scaled(subsign_img_size, subsign_img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);

  connect(this, &OnroadHud::valueChanged, [=] { update(); });
}

void OnroadHud::updateState(const UIState &s) {
  const int SET_SPEED_NA = 255;
  const SubMaster &sm = *(s.sm);
  const auto cs = sm["controlsState"].getControlsState();
  // dp
  const auto dp = sm["dragonConf"].getDragonConf();
  setProperty("dpUiSpeed", dp.getDpUiSpeed());
  setProperty("dpUiEvent", dp.getDpUiEvent());
  setProperty("dpUiFace", dp.getDpUiFace());
  setProperty("dpUiMaxSpeed", dp.getDpUiMaxSpeed());

  float maxspeed = cs.getVCruise();
  bool cruise_set = maxspeed > 0 && (int)maxspeed != SET_SPEED_NA;
  if (cruise_set && !s.scene.is_metric) {
    maxspeed *= KM_TO_MILE;
  }
  QString maxspeed_str = cruise_set ? QString::number(std::nearbyint(maxspeed)) : OnroadHud::tr("N/A");
  float cur_speed = std::max(0.0, sm["carState"].getCarState().getVEgo() * (s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH));

  setProperty("is_cruise_set", cruise_set);
  setProperty("speed", QString::number(std::nearbyint(cur_speed)));
  setProperty("maxSpeed", maxspeed_str);
  setProperty("speedUnit", s.scene.is_metric ? OnroadHud::tr("km/h") : OnroadHud::tr("mph"));
  setProperty("hideDM", cs.getAlertSize() != cereal::ControlsState::AlertSize::NONE);
  setProperty("status", s.status);

  // update engageability and DM icons at 2Hz
  if (sm.frame % (UI_FREQ / 2) == 0) {
    setProperty("engageable", cs.getEngageable() || cs.getEnabled());
    setProperty("dmActive", sm["driverMonitoringState"].getDriverMonitoringState().getIsActiveMode());

    const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
    const auto vtcState = lp.getVisionTurnControllerState();
    const float vtc_speed = lp.getVisionTurnSpeed() * (s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH);
    const auto lpSoruce = lp.getLongitudinalPlanSource();
    QColor vtc_color = tcs_colors[int(vtcState)];
    vtc_color.setAlpha(lpSoruce == cereal::LongitudinalPlan::LongitudinalPlanSource::TURN ? 255 : 100);

    setProperty("showVTC", vtcState > cereal::LongitudinalPlan::VisionTurnControllerState::DISABLED);
    setProperty("vtcSpeed", QString::number(std::nearbyint(vtc_speed)));
    setProperty("vtcColor", vtc_color);
    setProperty("showDebugUI", s.scene.show_debug_ui);

    const float speed_limit = lp.getSpeedLimit() * (s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH);
    const float speed_limit_offset = lp.getSpeedLimitOffset() * (s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH);
    const auto slcState = lp.getSpeedLimitControlState();
    const bool sl_force_active = s.scene.speed_limit_control_enabled &&
                                 seconds_since_boot() < s.scene.last_speed_limit_sign_tap + 2.0;
    const bool sl_inactive = !sl_force_active && (!s.scene.speed_limit_control_enabled ||
                             slcState == cereal::LongitudinalPlan::SpeedLimitControlState::INACTIVE);
    const bool sl_temp_inactive = !sl_force_active && (s.scene.speed_limit_control_enabled &&
                                  slcState == cereal::LongitudinalPlan::SpeedLimitControlState::TEMP_INACTIVE);
    const int sl_distance = int(lp.getDistToSpeedLimit() * (s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH) / 10.0) * 10;
    const QString sl_distance_str(QString::number(sl_distance) + (s.scene.is_metric ? "m" : "f"));
    const QString sl_offset_str(speed_limit_offset > 0.0 ?
                                "+" + QString::number(std::nearbyint(speed_limit_offset)) : "");
    const QString sl_inactive_str(sl_temp_inactive ? "TEMP" : "");
    const QString sl_substring(sl_inactive || sl_temp_inactive ? sl_inactive_str :
                               sl_distance > 0 ? sl_distance_str : sl_offset_str);

    setProperty("showSpeedLimit", speed_limit > 0.0);
    setProperty("speedLimit", QString::number(std::nearbyint(speed_limit)));
    setProperty("slcSubText", sl_substring);
    setProperty("slcSubTextSize", sl_inactive || sl_temp_inactive || sl_distance > 0 ? 22.2 : 37.0);
    setProperty("mapSourcedSpeedLimit", lp.getIsMapSpeedLimit());
    setProperty("slcActive", !sl_inactive && !sl_temp_inactive);

    const float tsc_speed = lp.getTurnSpeed() * (s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH);
    const auto tscState = lp.getTurnSpeedControlState();
    const int t_distance = int(lp.getDistToTurn() * (s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH) / 10.0) * 10;
    const QString t_distance_str(QString::number(t_distance) + (s.scene.is_metric ? "m" : "f"));

    setProperty("showTurnSpeedLimit", tsc_speed > 0.0 && (tsc_speed < cur_speed || s.scene.show_debug_ui));
    setProperty("turnSpeedLimit", QString::number(std::nearbyint(tsc_speed)));
    setProperty("tscSubText", t_distance > 0 ? t_distance_str : QString(""));
    setProperty("tscActive", tscState > cereal::LongitudinalPlan::SpeedLimitControlState::TEMP_INACTIVE);
    setProperty("curveSign", lp.getTurnSign());
  }
}

void OnroadHud::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  // Header gradient
  QLinearGradient bg(0, header_h - (header_h / 2.5), 0, header_h);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, width(), header_h, bg);

  QRect rc(bdr_s * 2, bdr_s * 1.5, 184, 202);
  if (dpUiMaxSpeed) {
  // max speed
  p.setPen(QPen(QColor(0xff, 0xff, 0xff, 100), 10));
  p.setBrush(QColor(0, 0, 0, 100));
  p.drawRoundedRect(rc, 20, 20);
  p.setPen(Qt::NoPen);

  configFont(p, "Open Sans", 48, "Regular");
  drawText(p, rc.center().x(), 118, OnroadHud::tr("MAX"), is_cruise_set ? 200 : 100);
  if (is_cruise_set) {
    configFont(p, "Open Sans", 88, is_cruise_set ? "Bold" : "SemiBold");
    drawText(p, rc.center().x(), 212, maxSpeed, 255);
  } else {
    configFont(p, "Open Sans", 80, "SemiBold");
    drawText(p, rc.center().x(), 212, maxSpeed, 100);
  }
  }

  if (dpUiSpeed) {
  // current speed
  configFont(p, "Open Sans", 176, "Bold");
  drawText(p, rect().center().x(), 210, speed);
  configFont(p, "Open Sans", 66, "Regular");
  drawText(p, rect().center().x(), 290, speedUnit, 200);
  }
  // engage-ability icon
  if (engageable) {
    if (showDebugUI && showVTC) {
      drawVisionTurnControllerUI(p, rect().right() - 184 - bdr_s, int(bdr_s * 1.5), 184, vtcColor, vtcSpeed, 100);
    } else {
      if (dpUiEvent) {
      // engage-ability icon
      drawIcon(p, rect().right() - radius / 2 - bdr_s * 2, radius / 2 + int(bdr_s * 1.5),
               engage_img, bg_colors[status], 1.0);
      }
    }

    // Speed Limit Sign
    if (showSpeedLimit) {
      drawSpeedSign(p, speed_sgn_rc, speedLimit, slcSubText, slcSubTextSize, mapSourcedSpeedLimit, slcActive);
    }

    // Turn Speed Sign
    if (showTurnSpeedLimit) {
      rc.moveTop(speed_sgn_rc.bottom() + bdr_s);
      drawTrunSpeedSign(p, rc, turnSpeedLimit, tscSubText, curveSign, tscActive);
    }

  }
  if (dpUiFace) {
  // dm icon
  if (!hideDM) {
    drawIcon(p, radius / 2 + (bdr_s * 2), rect().bottom() - footer_h / 2,
             dm_img, QColor(0, 0, 0, 70), dmActive ? 1.0 : 0.2);
  }
  }
}

void OnroadHud::drawText(QPainter &p, int x, int y, const QString &text, int alpha) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void OnroadHud::drawCenteredText(QPainter &p, int x, int y, const QString &text, QColor color) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y});

  p.setPen(color);
  p.drawText(real_rect, Qt::AlignCenter, text);
}

void OnroadHud::drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity) {
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);
  p.setOpacity(opacity);
  p.drawPixmap(x - img_size / 2, y - img_size / 2, img);
}

void OnroadHud::drawVisionTurnControllerUI(QPainter &p, int x, int y, int size, const QColor &color,
                                           const QString &vision_speed, int alpha) {
  QRect rvtc(x, y, size, size);
  p.setPen(QPen(color, 10));
  p.setBrush(QColor(0, 0, 0, alpha));
  p.drawRoundedRect(rvtc, 20, 20);
  p.setPen(Qt::NoPen);

  configFont(p, "Open Sans", 56, "SemiBold");
  drawCenteredText(p, rvtc.center().x(), rvtc.center().y(), vision_speed, color);
}

void OnroadHud::drawCircle(QPainter &p, int x, int y, int r, QBrush bg) {
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(x - r, y - r, 2 * r, 2 * r);
}

void OnroadHud::drawSpeedSign(QPainter &p, QRect rc, const QString &speed_limit, const QString &sub_text,
                              int subtext_size, bool is_map_sourced, bool is_active) {
  const QColor ring_color = is_active ? QColor(255, 0, 0, 255) : QColor(0, 0, 0, 50);
  const QColor inner_color = QColor(255, 255, 255, is_active ? 255 : 85);
  const QColor text_color = QColor(0, 0, 0, is_active ? 255 : 85);

  const int x = rc.center().x();
  const int y = rc.center().y();
  const int r = rc.width() / 2.0f;

  drawCircle(p, x, y, r, ring_color);
  drawCircle(p, x, y, int(r * 0.8f), inner_color);

  configFont(p, "Open Sans", 89, "Bold");
  drawCenteredText(p, x, y, speed_limit, text_color);
  configFont(p, "Open Sans", subtext_size, "Bold");
  drawCenteredText(p, x, y + 55, sub_text, text_color);

  if (is_map_sourced) {
    p.setPen(Qt::NoPen);
    p.setOpacity(is_active ? 1.0 : 0.3);
    p.drawPixmap(x - subsign_img_size / 2, y - 55 - subsign_img_size / 2, map_img);
    p.setOpacity(1.0);
  }
}

void OnroadHud::drawTrunSpeedSign(QPainter &p, QRect rc, const QString &turn_speed, const QString &sub_text,
                                  int curv_sign, bool is_active) {
  const QColor border_color = is_active ? QColor(255, 0, 0, 255) : QColor(0, 0, 0, 50);
  const QColor inner_color = QColor(255, 255, 255, is_active ? 255 : 85);
  const QColor text_color = QColor(0, 0, 0, is_active ? 255 : 85);

  const int x = rc.center().x();
  const int y = rc.center().y();
  const int width = rc.width();

  const float stroke_w = 15.0;
  const float cS = stroke_w / 2.0 + 4.5;  // half width of the stroke on the corners of the triangle
  const float R = width / 2.0 - stroke_w / 2.0;
  const float A = 0.73205;
  const float h2 = 2.0 * R / (1.0 + A);
  const float h1 = A * h2;
  const float L = 4.0 * R / sqrt(3.0);

  // Draw the internal triangle, compensate for stroke width. Needed to improve rendering when in inactive
  // state due to stroke transparency being different from inner transparency.
  QPainterPath path;
  path.moveTo(x, y - R + cS);
  path.lineTo(x - L / 2.0 + cS, y + h1 + h2 - R - stroke_w / 2.0);
  path.lineTo(x + L / 2.0 - cS, y + h1 + h2 - R - stroke_w / 2.0);
  path.lineTo(x, y - R + cS);
  p.setPen(Qt::NoPen);
  p.setBrush(inner_color);
  p.drawPath(path);

  // Draw the stroke
  QPainterPath stroke_path;
  stroke_path.moveTo(x, y - R);
  stroke_path.lineTo(x - L / 2.0, y + h1 + h2 - R);
  stroke_path.lineTo(x + L / 2.0, y + h1 + h2 - R);
  stroke_path.lineTo(x, y - R);
  p.setPen(QPen(border_color, stroke_w, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.setBrush(Qt::NoBrush);
  p.drawPath(stroke_path);

  // Draw the turn sign
  if (curv_sign != 0) {
    p.setPen(Qt::NoPen);
    p.setOpacity(is_active ? 1.0 : 0.3);
    p.drawPixmap(int(x - (subsign_img_size / 2)), int(y - R + stroke_w + 30), curv_sign > 0 ? left_img : right_img);
    p.setOpacity(1.0);
  }

  // Draw the texts.
  configFont(p, "Open Sans", 67, "Bold");
  drawCenteredText(p, x, y + 25, turn_speed, text_color);
  configFont(p, "Open Sans", 22, "Bold");
  drawCenteredText(p, x, y + 65, sub_text, text_color);
}

// NvgWindow
void NvgWindow::initializeGL() {
  CameraViewWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);
}

void NvgWindow::updateFrameMat(int w, int h) {
  CameraViewWidget::updateFrameMat(w, h);

  UIState *s = &QUIState::ui_state;
  s->fb_w = w;
  s->fb_h = h;
  auto intrinsic_matrix = s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;
  float zoom = ZOOM / intrinsic_matrix.v[0];
  if (s->wide_camera) {
    zoom *= 0.5;
  }
  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(w / 2, h / 2 + y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
}

void NvgWindow::drawLaneLines(QPainter &painter, const UIScene &scene) {
  bool end_to_end = scene.end_to_end;
  if (QUIState::ui_state.scene.dpLaneLessModeCtrl) {
    if (QUIState::ui_state.scene.dpLaneLessMode == 1) { // e2e
      end_to_end = true;
    } else if (QUIState::ui_state.scene.dpLaneLessMode == 0) { // lane
      end_to_end = false;
    } else {
      // auto
      end_to_end = QUIState::ui_state.scene.dpLaneLessModeIsE2E;
    }
  }

  if (!end_to_end) {
    // lanelines
    for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
      painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, scene.lane_line_probs[i]));
      painter.drawPolygon(scene.lane_line_vertices[i].v, scene.lane_line_vertices[i].cnt);
    }
    // road edges
    for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
      painter.setBrush(QColor::fromRgbF(1.0, 0, 0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0)));
      painter.drawPolygon(scene.road_edge_vertices[i].v, scene.road_edge_vertices[i].cnt);
    }
  }
  // paint path
  QLinearGradient bg(0, height(), 0, height() / 4);
  bg.setColorAt(0, end_to_end ? redColor() : QColor(255, 255, 255));
  bg.setColorAt(1, end_to_end ? redColor(0) : QColor(255, 255, 255, 0));
  painter.setBrush(bg);
  painter.drawPolygon(scene.track_vertices.v, scene.track_vertices.cnt);
}

void NvgWindow::drawLead(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd, bool has_op_long) {
  const float speedBuff = 10.;
  const float leadBuff = 40.;
  const float d_rel = lead_data.getX()[0];
  const float v_rel = lead_data.getV()[0];

  float fillAlpha = 0;
  if (d_rel < leadBuff) {
    fillAlpha = 255 * (1.0 - (d_rel / leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255 * (-1 * (v_rel / speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  float y = std::fmin(height() - sz * .6, (float)vd.y());

  float g_xo = sz / 5;
  float g_yo = sz / 10;

  QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_xo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}};
  painter.setBrush(QColor(218, 202, 37, 255));
  painter.drawPolygon(glow, std::size(glow));

  // chevron
  QPointF chevron[] = {{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz}};
  painter.setBrush(has_op_long? redColor(fillAlpha) : QColor(180, 180, 180, fillAlpha));
  painter.drawPolygon(chevron, std::size(chevron));
}

void NvgWindow::drawColoredText(QPainter &p, int x, int y, const QString &text, QColor color = QColor(0xff, 0xff, 0xff, 0xc8)) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(color);
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void NvgWindow::drawBlinkers(QPainter &painter, const cereal::CarState::Reader &cs) {
  bool leftBlinker = cs.getLeftBlinker();
  bool rightBlinker = cs.getRightBlinker();
  if (!leftBlinker && !rightBlinker) {
    blinkerAlpha = 200;
    blinkerOffset = 0;
  } else {
    blinkerAlpha -= 5;
    blinkerOffset += 10;
    if (blinkerAlpha <= 100) {
      blinkerAlpha = 200;
      blinkerOffset = 0;
    }
  }
  if (leftBlinker) {
    int x = rect().center().x() - 200 - blinkerOffset;
    int y = bdr_s*2;

    QPainterPath path = QPainterPath();
    path.moveTo(x, y);
    path.lineTo(x, y + 250);
    path.lineTo(x - 100, y + 125);
    path.lineTo(x, y);
    painter.setPen(Qt::NoPen);
    painter.fillPath(path, QBrush(QColor(0, 0xff, 0, blinkerAlpha)));
  }
  if (rightBlinker) {
    int x = rect().center().x() + 200 + blinkerOffset;
    int y = bdr_s*2;

    QPainterPath path = QPainterPath();
    path.moveTo(x, y);
    path.lineTo(x, y + 250);
    path.lineTo(x + 100, y + 125);
    path.lineTo(x, y);
    painter.setPen(Qt::NoPen);
    painter.fillPath(path, QBrush(QColor(0, 0xff, 0, blinkerAlpha)));
  }
}
void NvgWindow::drawBlindSpots(QPainter &painter, const cereal::CarState::Reader &cs) {
  bool leftBlindSpot = cs.getLeftBlindspot();
  bool rightBlindSpot = cs.getRightBlindspot();
  if (leftBlindSpot || rightBlindSpot) {
    int blind_spot_width = (int)(bdr_s * 1.5);
    int blind_spot_height = height();
    painter.setBrush(QColor(0xff, 0xff, 0, 0xc8));
    painter.setPen(Qt::NoPen);

    if (leftBlindSpot) {
      QRect rc(0, 0, blind_spot_width, blind_spot_height);
      painter.drawRoundedRect(rc, 0, 0);
    }
    if (rightBlindSpot) {
      QRect rc(width() - blind_spot_width, 0, blind_spot_width, blind_spot_height);
      painter.drawRoundedRect(rc, 0, 0);
    }
  }
}
void NvgWindow::drawTopbar(QPainter &painter, const cereal::CarState::Reader &cs) {
  QRect rc(0, 0, width(), bdr_s + 10); // +10 give some bottom spacing
  if (cs.getBrakeLights()) {
    painter.setBrush(QColor(0xff, 0, 0, 0xff));
  } else {
    painter.setBrush(QColor(0, 0, 0, 0xff));
  }
  painter.drawRoundedRect(rc, 0, 0);
  painter.setPen(Qt::NoPen);
  QDateTime dateTime = dateTime.currentDateTime();

  configFont(painter, "Open Sans", 32, "Bold");
  drawColoredText(painter, rc.center().x(), rc.bottom() - 5, dateTime.toString("yyyy-MM-dd HH:mm:ss"), QColor(0xff, 0xff, 0xff, 0xc8));
}

// from fcw.py in 0.8.9
float NvgWindow::calc_ttc(float v_ego, float a_ego, float x_lead, float v_lead, float a_lead) {
  const float max_ttc = 5.0;
  float v_rel = v_ego - v_lead;
  float a_rel = a_ego - a_lead;

//    # assuming that closing gap ARel comes from lead vehicle decel,
//    # then limit ARel so that v_lead will get to zero in no sooner than t_decel.
//    # This helps underweighting ARel when v_lead is close to zero.
  const float t_decel = 2.0;
  a_rel = std::fmin(a_rel, v_lead / t_decel);

//    # delta of the quadratic equation to solve for ttc
  float delta = pow(abs(v_rel), 2) + 2 * x_lead * a_rel;

//    # assign an arbitrary high ttc value if there is no solution to ttc
  if (delta < 0.1 || (sqrt(delta) + v_rel < 0.1)) {
    return max_ttc;
  } else {
    return std::fmin(2 * x_lead / (sqrt(delta) + v_rel), max_ttc);
  }
}

void NvgWindow::drawSidebar(QPainter &painter, const cereal::ModelDataV2::Reader &lead_reader, const cereal::CarState::Reader &cs, const cereal::ControlsState::Reader &css, bool is_metric) {
  painter.setBrush(Qt::NoBrush);
  int y = bdr_s * 1.5 + 202; // max speed
  QColor text_color = QColor(0xff, 0xff, 0xff, 0xff);
  const int end_y = height() - bdr_s*2 - 20; // total available height - bdr_s*2 (map) - 20 (leave a gap)
  y += 50;
  QRect rc(rect().right() - dp_sidebar_width - bdr_s * 2, y, dp_sidebar_width, end_y - y);
  painter.setPen(QPen(text_color, 10));
  painter.drawRoundedRect(rc, 20, 20);
  painter.setPen(Qt::NoPen);

  y += 80;

  // actual steering
  float angleSteers = css.getAngleSteers();
  configFont(painter, "Open Sans", 54, "SemiBold");
  text_color = QColor(0xff, 0xff, 0xff, 0xff);
  drawColoredText(painter, rc.center().x(), y, QString::number(angleSteers, 'f', 1), text_color);
  y += 30;
  configFont(painter, "Open Sans", 24, "Bold");
  drawColoredText(painter, rc.center().x(), y, NvgWindow::tr("REAL STEER"));
  y += 100; // spacing

  // desire steering
  float angleSteersDes = css.getSteeringAngleDesiredDeg();
  configFont(painter, "Open Sans", 54, "SemiBold");
  text_color = QColor(0xff, 0xff, 0xff, 0xff);
  if (abs(angleSteersDes) >= 12) {
    text_color = QColor(0xff, 0x0, 0x0, 0xff);
  } else if (abs(angleSteersDes) >= 6) {
    text_color = QColor(0xff, 0xff, 0x0, 0xff);
  }
  drawColoredText(painter, rc.center().x(), y, QString::number(angleSteersDes, 'f', 1), text_color);
  y += 30;
  configFont(painter, "Open Sans", 24, "Bold");
  drawColoredText(painter, rc.center().x(), y, NvgWindow::tr("DESIR STEER"));
  y += 100; // spacing

  float v_ego = cs.getVEgo();
  float a_ego = cs.getAEgo();

  // find leads data
  float v_lead = 0;
  float a_lead = 0;
  float x_lead = 0;
  auto leads = lead_reader.getLeadsV3();
  if (leads[0].getProb() > .5) {
    v_lead = leads[0].getV()[0];
    a_lead = leads[0].getA()[0];
    x_lead = leads[0].getX()[0];
  } else if (leads[1].getProb() > .5 && (std::abs(leads[1].getX()[0] - leads[0].getX()[0]) > 3.0)) {
    v_lead = leads[1].getV()[0];
    a_lead = leads[1].getA()[0];
    x_lead = leads[1].getX()[0];
  }

  // relative distance
  configFont(painter, "Open Sans", 54, "SemiBold");
  drawColoredText(painter, rc.center().x(), y, x_lead == 0? NvgWindow::tr("N/A") : QString::number(x_lead, 'f', 1));
  y += 30;
  configFont(painter, "Open Sans", 24, "Bold");
  drawColoredText(painter, rc.center().x(), y, NvgWindow::tr("REL DIST"));
  y += 100; // spacing

  // relative speed
  float ttc = calc_ttc(v_ego, a_ego, x_lead, v_lead, a_lead);
  configFont(painter, "Open Sans", 54, "SemiBold");
  if (x_lead == 0) {
    text_color = QColor(0xff, 0xff, 0xff, 0xff);
  } else if (ttc <= 1) {
    text_color = QColor(0xff, 0x0, 0x0, 0xff);
  } else if (ttc < 2) {
    text_color = QColor(0xff, 0xff, 0x0, 0xff);
  } else {
    text_color = QColor(0xff, 0xff, 0xff, 0xff);
  }
  drawColoredText(painter, rc.center().x(), y, (x_lead == 0 || ttc == 5.0)? NvgWindow::tr("N/A") : QString::number(ttc, 'f', 1), text_color);
  y += 30;
  configFont(painter, "Open Sans", 24, "Bold");
  drawColoredText(painter, rc.center().x(), y, NvgWindow::tr("TTC"));
  y += 100; // spacing

  // engine rpm
  float engineRPM = cs.getEngineRPM();
  configFont(painter, "Open Sans", 52, "SemiBold");
  text_color = QColor(0xff, 0xff, 0xff, 0xff);
  if (engineRPM >= 6000) {
    text_color = QColor(0xff, 0x0, 0x0, 0xff);
  } else if (engineRPM >= 4000) {
    text_color = QColor(0xff, 0xff, 0x0, 0xff);
  }
  drawColoredText(painter, rc.center().x(), y, engineRPM <= 0 ? NvgWindow::tr("OFF") : QString::number((int)engineRPM), text_color);
  y += 30;
  configFont(painter, "Open Sans", 24, "Bold");
  drawColoredText(painter, rc.center().x(), y, NvgWindow::tr("ENG RPM"));
}

void NvgWindow::drawBottombar(QPainter &painter, const cereal::LiveMapData::Reader &lmd) {
  painter.setBrush(QColor(0, 0, 0, 0xff));
  painter.setPen(Qt::NoPen);
  QRect rc(0, height() - bdr_s*2, width(), bdr_s*2);
  painter.drawRoundedRect(rc, 0, 0);

  configFont(painter, "Open Sans", 48, "Bold");
  drawColoredText(painter, rc.center().x(), height() - bdr_s/2, QString::fromStdString(lmd.getCurrentRoadName()), QColor(0xff, 0xff, 0xff, 0xff));
}

void NvgWindow::paintBtn(QPainter &painter, int x, int y, QString main, QString sub) {
  const QColor text_color = QColor(0xff, 0xff, 0xff, 0xc8);
  QRect dlp_rc = QRect(x, y, dp_btn_wh, dp_btn_wh);

  painter.setBrush(Qt::NoBrush);
  painter.setPen(QPen(text_color, 10));
  painter.drawRoundedRect(dlp_rc, 20, 20);
  painter.setPen(Qt::NoPen);

  configFont(painter, "Open Sans", 48, "SemiBold");
  drawColoredText(painter, dlp_rc.center().x(), dlp_rc.center().y(), main, text_color);
  configFont(painter, "Open Sans", 24, "Bold");
  drawColoredText(painter, dlp_rc.center().x(), dlp_rc.bottom() - 20, sub, text_color);
}

void NvgWindow::drawBtns(QPainter &painter) {
  painter.setBrush(Qt::NoBrush);

  // btn init position, we work from right to left
  int x = rect().right() - dp_sidebar_width - bdr_s * 2;
  const int y = height() - bdr_s*2 - dp_btn_wh - 20;
  const int x_offset = dp_btn_wh + dp_btn_wh_gap;

  // fp
  //if (QUIState::ui_state.scene.dpFollowingProfileCtrl) {
  //  x -= x_offset;
  //  QString fp;
  //  switch (QUIState::ui_state.scene.dpFollowingProfile) {
  //    case 1:
  //      fp = NvgWindow::tr("1.2s");
  //      break;
  //    case 2:
  //      fp = NvgWindow::tr("1.4s");
  //      break;
  //    case 3:
  //      fp = NvgWindow::tr("1.8s");
  //      break;
  //    default:
  //      fp = NvgWindow::tr("1.0s");
  //  }
  //  paintBtn(painter, x, y, fp, NvgWindow::tr("FOLLOW"));
  //}

  // ap
  if (QUIState::ui_state.scene.dpAccelProfileCtrl) {
    x -= x_offset;
    QString ap;
    switch (QUIState::ui_state.scene.dpAccelProfile) {
      case 1:
        ap = NvgWindow::tr("NOR");
        break;
      case 2:
        ap = NvgWindow::tr("SPT");
        break;
      default:
        ap = NvgWindow::tr("ECO");
    }
    paintBtn(painter, x, y, ap, NvgWindow::tr("ACCEL"));
  }

  // dlp
  if (QUIState::ui_state.scene.dpLaneLessModeCtrl) {
    x -= x_offset;
    QString dlp;
    switch (QUIState::ui_state.scene.dpLaneLessMode) {
      case 1:
        dlp = NvgWindow::tr("E2E");
        break;
      case 2:
        dlp = NvgWindow::tr("AUTO");
        break;
      default:
        dlp = NvgWindow::tr("LANE");
    }
    paintBtn(painter, x, y, dlp, NvgWindow::tr("LANE"));
  }
}

void NvgWindow::paintGL() {
  CameraViewWidget::paintGL();

  UIState *s = &QUIState::ui_state;
  auto dp = (*s->sm)["dragonConf"].getDragonConf();
  if (s->scene.world_objects_visible) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(Qt::NoPen);

    if (dp.getDpUiLane())
      drawLaneLines(painter, s->scene);

    if (dp.getDpUiLead()) { //s->scene.longitudinal_control) {
      auto leads = (*s->sm)["modelV2"].getModelV2().getLeadsV3();
      if (leads[0].getProb() > .5) {
        drawLead(painter, leads[0], s->scene.lead_vertices[0], s->scene.longitudinal_control);
      }
      if (leads[1].getProb() > .5 && (std::abs(leads[1].getX()[0] - leads[0].getX()[0]) > 3.0)) {
        drawLead(painter, leads[1], s->scene.lead_vertices[1], s->scene.longitudinal_control);
      }
    }

    drawBlindSpots(painter, (*s->sm)["carState"].getCarState());
    if (dp.getDpUiBlinker())
      drawBlinkers(painter, (*s->sm)["carState"].getCarState());
    if (dp.getDpUiTop())
      drawTopbar(painter, (*s->sm)["carState"].getCarState());
    if (dp.getDpUiSide())
      drawSidebar(painter, (*s->sm)["modelV2"].getModelV2(), (*s->sm)["carState"].getCarState(), (*s->sm)["controlsState"].getControlsState(), s->scene.is_metric);
    if (dp.getDpMapd())
      drawBottombar(painter, (*s->sm)["liveMapData"].getLiveMapData());
    drawBtns(painter);
  }

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  if (dt > 66) {
    // warn on sub 15fps
    LOGW("slow frame time: %.2f", dt);
  }
  prev_draw_t = cur_draw_t;
}

void NvgWindow::showEvent(QShowEvent *event) {
  CameraViewWidget::showEvent(event);

  ui_update_params(&QUIState::ui_state);
  prev_draw_t = millis_since_boot();
}
