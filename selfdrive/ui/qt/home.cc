#include "selfdrive/ui/qt/home.h"

#include <QDateTime>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QVBoxLayout>

#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/drive_stats.h"
#include "selfdrive/ui/qt/widgets/prime.h"

// HomeWindow: the container for the offroad and onroad UIs

HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setMargin(0);
  main_layout->setSpacing(0);

  sidebar = new Sidebar(this);
  main_layout->addWidget(sidebar);
  QObject::connect(this, &HomeWindow::update, sidebar, &Sidebar::updateState);
  QObject::connect(sidebar, &Sidebar::openSettings, this, &HomeWindow::openSettings);

  slayout = new QStackedLayout();
  main_layout->addLayout(slayout);

  home = new OffroadHome();
  slayout->addWidget(home);

  onroad = new OnroadWindow(this);
  slayout->addWidget(onroad);

  QObject::connect(this, &HomeWindow::update, onroad, &OnroadWindow::updateStateSignal);
  QObject::connect(this, &HomeWindow::offroadTransitionSignal, onroad, &OnroadWindow::offroadTransitionSignal);

  driver_view = new DriverViewWindow(this);
  connect(driver_view, &DriverViewWindow::done, [=] {
    showDriverView(false);
  });
  slayout->addWidget(driver_view);
  setAttribute(Qt::WA_NoSystemBackground);
}

void HomeWindow::showSidebar(bool show) {
  sidebar->setVisible(show);
}

void HomeWindow::offroadTransition(bool offroad) {
  sidebar->setVisible(offroad);
  if (offroad) {
    slayout->setCurrentWidget(home);
  } else {
    slayout->setCurrentWidget(onroad);
  }
  emit offroadTransitionSignal(offroad);
}

void HomeWindow::showDriverView(bool show) {
  if (show) {
    emit closeSettings();
    slayout->setCurrentWidget(driver_view);
  } else {
    slayout->setCurrentWidget(home);
  }
  sidebar->setVisible(show == false);
}

void HomeWindow::mousePressEvent(QMouseEvent* e) {
  // Toggle speed limit control enabled
  SubMaster &sm = *(QUIState::ui_state.sm);
  auto longitudinal_plan = sm["longitudinalPlan"].getLongitudinalPlan();
  Rect speed_limit_touch_rect = QUIState::ui_state.scene.speed_limit_sign_touch_rect;
  Rect debug_tap_rect = {rect().center().x() - 200, rect().center().y() - 200, 400, 400};
  if (sidebar->isVisible()) {
    speed_limit_touch_rect.x += sidebar->width();
    debug_tap_rect.x += sidebar->width();
    debug_tap_rect.w -= sidebar->width();
  }

  if (onroad->isVisible() && longitudinal_plan.getSpeedLimit() > 0.0 &&
      speed_limit_touch_rect.ptInRect(e->x(), e->y())) {
    // If touching the speed limit sign area when visible
    QUIState::ui_state.scene.last_speed_limit_sign_tap = seconds_since_boot();
    QUIState::ui_state.scene.speed_limit_control_enabled = !QUIState::ui_state.scene.speed_limit_control_enabled;
    Params().putBool("SpeedLimitControl", QUIState::ui_state.scene.speed_limit_control_enabled);
  }

  // Issue a debug print.
  else if (onroad->isVisible() && debug_tap_rect.ptInRect(e->x(), e->y())) {
    char param_name[64] = {'\0'};
    time_t rawtime = time(NULL);
    struct tm timeinfo;
    localtime_r(&rawtime, &timeinfo);
    strftime(param_name, sizeof(param_name), "%Y-%m-%d--%H-%M-%S", &timeinfo);

    auto live_map_data = sm["liveMapData"].getLiveMapData();
    auto car_state = sm["carState"].getCarState();

    char turn_speeds_ahead_s[512] = "-\t-\t-\n";
    const auto speeds = live_map_data.getTurnSpeedLimitsAhead();
    const auto distances = live_map_data.getTurnSpeedLimitsAheadDistances();
    const auto signs = live_map_data.getTurnSpeedLimitsAheadSigns();
    int wrote = 0;

    for(int i = 0; i < speeds.size(); i++) {
      wrote += snprintf(turn_speeds_ahead_s + wrote, sizeof(turn_speeds_ahead_s) - wrote, "%.1f\t%.1f\t%d\n",
                        speeds[i] * 3.6, distances[i], signs[i]);
    }

    char s[1024];
    int size = snprintf(s, sizeof(s), 
      "Datetime: %s, vEgo: %.2f\n"
      "Pos, Bearing: (%.6f, %.6f), %.2f; GPSSpeed: %.1f\n"
      "sl: %.1f, valid: %d\n"
      "sl_ahead: %.1f, valid: %d, dist: %.1f\n"
      "tsl: %.1f, valid: %d, end dist: %.1f, sign: %d\n\n"
      "tsl ahead:\n"
      "VALUE\tDIST\tSIGN\n"
      "%s\n\n"
      "SPEED LIMIT CONTROLLER:\n"
      "sl: %.1f, state: %hu, isMap: %d\n\n"
      "TURN SPEED CONTROLLER:\n"
      "speed: %.1f, state: %hu\n\n"
      "VISIOn TURN CONTROLLER:\n"
      "speed: %.1f, state: %hu", 
      param_name, car_state.getVEgo() * 3.6,
      live_map_data.getLastGpsLatitude(), live_map_data.getLastGpsLongitude(), live_map_data.getLastGpsBearingDeg(), live_map_data.getLastGpsSpeed() * 3.6,
      live_map_data.getSpeedLimit() * 3.6, live_map_data.getSpeedLimitValid(),
      live_map_data.getSpeedLimitAhead() * 3.6, live_map_data.getSpeedLimitAheadValid(), live_map_data.getSpeedLimitAheadDistance(),
      live_map_data.getTurnSpeedLimit() * 3.6, live_map_data.getTurnSpeedLimitValid(), live_map_data.getTurnSpeedLimitEndDistance(), live_map_data.getTurnSpeedLimitSign(),
      turn_speeds_ahead_s,
      longitudinal_plan.getSpeedLimit() * 3.6, longitudinal_plan.getSpeedLimitControlState(), longitudinal_plan.getIsMapSpeedLimit(),
      longitudinal_plan.getTurnSpeed() * 3.6, longitudinal_plan.getTurnSpeedControlState(),
      longitudinal_plan.getVisionTurnSpeed() * 3.6, longitudinal_plan.getVisionTurnControllerState());

    Params().put(param_name, s, size < sizeof(s) ? size : sizeof(s));
    QUIState::ui_state.scene.display_debug_alert_frame = sm.frame;
  }

  // Handle sidebar collapsing
  else if (onroad->isVisible() && (!sidebar->isVisible() || e->x() > sidebar->width())) {

    // TODO: Handle this without exposing pointer to map widget
    // Hide map first if visible, then hide sidebar
    if (onroad->map != nullptr && onroad->map->isVisible()) {
      onroad->map->setVisible(false);
    } else if (!sidebar->isVisible()) {
      sidebar->setVisible(true);
    } else {
      sidebar->setVisible(false);

      if (onroad->map != nullptr) onroad->map->setVisible(true);
    }
  }
}

// OffroadHome: the offroad home page

OffroadHome::OffroadHome(QWidget* parent) : QFrame(parent) {
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(40, 40, 40, 45);

  // top header
  QHBoxLayout* header_layout = new QHBoxLayout();
  header_layout->setContentsMargins(15, 15, 15, 0);
  header_layout->setSpacing(16);

  date = new QLabel();
  header_layout->addWidget(date, 1, Qt::AlignHCenter | Qt::AlignLeft);

  update_notif = new QPushButton("UPDATE");
  update_notif->setVisible(false);
  update_notif->setStyleSheet("background-color: #364DEF;");
  QObject::connect(update_notif, &QPushButton::clicked, [=]() { center_layout->setCurrentIndex(1); });
  header_layout->addWidget(update_notif, 0, Qt::AlignHCenter | Qt::AlignRight);

  alert_notif = new QPushButton();
  alert_notif->setVisible(false);
  alert_notif->setStyleSheet("background-color: #E22C2C;");
  QObject::connect(alert_notif, &QPushButton::clicked, [=] { center_layout->setCurrentIndex(2); });
  header_layout->addWidget(alert_notif, 0, Qt::AlignHCenter | Qt::AlignRight);

  header_layout->addWidget(new QLabel(getBrandVersion()), 0, Qt::AlignHCenter | Qt::AlignRight);

  main_layout->addLayout(header_layout);

  // main content
  main_layout->addSpacing(25);
  center_layout = new QStackedLayout();

  QWidget* statsAndSetupWidget = new QWidget(this);
  QHBoxLayout* statsAndSetup = new QHBoxLayout(statsAndSetupWidget);
  statsAndSetup->setMargin(0);
  statsAndSetup->setSpacing(30);
  statsAndSetup->addWidget(new DriveStats, 1);
  statsAndSetup->addWidget(new SetupWidget);

  center_layout->addWidget(statsAndSetupWidget);

  // add update & alerts widgets
  update_widget = new UpdateAlert();
  QObject::connect(update_widget, &UpdateAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(update_widget);
  alerts_widget = new OffroadAlert();
  QObject::connect(alerts_widget, &OffroadAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(alerts_widget);

  main_layout->addLayout(center_layout, 1);

  // set up refresh timer
  timer = new QTimer(this);
  timer->callOnTimeout(this, &OffroadHome::refresh);

  setStyleSheet(R"(
    * {
     color: white;
    }
    OffroadHome {
      background-color: black;
    }
    OffroadHome > QPushButton {
      padding: 15px 30px;
      border-radius: 5px;
      font-size: 40px;
      font-weight: 500;
    }
    OffroadHome > QLabel {
      font-size: 55px;
    }
  )");
  refresh();
}

void OffroadHome::showEvent(QShowEvent *event) {
  timer->start(10 * 1000);
}

void OffroadHome::hideEvent(QHideEvent *event) {
  timer->stop();
}

void OffroadHome::refresh() {
  date->setText(QDateTime::currentDateTime().toString("dddd, MMMM d"));

  bool updateAvailable = update_widget->refresh();
  int alerts = alerts_widget->refresh();

  // pop-up new notification
  int idx = center_layout->currentIndex();
  if (!updateAvailable && !alerts) {
    idx = 0;
  } else if (updateAvailable && (!update_notif->isVisible() || (!alerts && idx == 2))) {
    idx = 1;
  } else if (alerts && (!alert_notif->isVisible() || (!updateAvailable && idx == 1))) {
    idx = 2;
  }
  center_layout->setCurrentIndex(idx);

  update_notif->setVisible(updateAvailable);
  alert_notif->setVisible(alerts);
  if (alerts) {
    alert_notif->setText(QString::number(alerts) + " ALERT" + (alerts > 1 ? "S" : ""));
  }
}
