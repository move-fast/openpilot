#pragma once

#include <QStackedLayout>
#include <QWidget>

#include "selfdrive/ui/qt/widgets/cameraview.h"
#include "selfdrive/ui/ui.h"


// ***** onroad widgets *****

class OnroadHud : public QWidget {
  Q_OBJECT
  Q_PROPERTY(QString speed MEMBER speed NOTIFY valueChanged);
  Q_PROPERTY(QString speedUnit MEMBER speedUnit NOTIFY valueChanged);
  Q_PROPERTY(QString maxSpeed MEMBER maxSpeed NOTIFY valueChanged);
  Q_PROPERTY(bool is_cruise_set MEMBER is_cruise_set NOTIFY valueChanged);
  Q_PROPERTY(bool engageable MEMBER engageable NOTIFY valueChanged);
  Q_PROPERTY(bool dmActive MEMBER dmActive NOTIFY valueChanged);
  Q_PROPERTY(bool hideDM MEMBER hideDM NOTIFY valueChanged);
  Q_PROPERTY(int status MEMBER status NOTIFY valueChanged);

  // dp
  // topbar
  Q_PROPERTY(QString currentDateTime MEMBER currentDateTime NOTIFY valueChanged);
  // bottombar
  Q_PROPERTY(QString mapd_info MEMBER mapd_info NOTIFY valueChanged);
  Q_PROPERTY(float angleSteers MEMBER angleSteers NOTIFY valueChanged);
  Q_PROPERTY(float angleSteersDes MEMBER angleSteersDes NOTIFY valueChanged);
  Q_PROPERTY(float relativeDist MEMBER relativeDist NOTIFY valueChanged);
  Q_PROPERTY(float relativeSpeed MEMBER relativeSpeed NOTIFY valueChanged);
  Q_PROPERTY(int engineRPM MEMBER engineRPM NOTIFY valueChanged);
  Q_PROPERTY(bool brakeLights MEMBER brakeLights NOTIFY valueChanged);
  Q_PROPERTY(bool leftBlindspot MEMBER leftBlindspot NOTIFY valueChanged);
  Q_PROPERTY(bool rightBlindspot MEMBER rightBlindspot NOTIFY valueChanged);
  Q_PROPERTY(bool steeringPressed MEMBER steeringPressed NOTIFY valueChanged);
  Q_PROPERTY(bool leftBlinker MEMBER leftBlinker NOTIFY valueChanged);
  Q_PROPERTY(bool rightBlinker MEMBER rightBlinker NOTIFY valueChanged);
  Q_PROPERTY(int blinkerOffset MEMBER blinkerOffset NOTIFY valueChanged);
  Q_PROPERTY(int blinkerAlpha MEMBER blinkerAlpha NOTIFY valueChanged);

public:
  explicit OnroadHud(QWidget *parent);
  void updateState(const UIState &s);

private:
  void drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity);
  void drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255);
  void paintEvent(QPaintEvent *event) override;
  void drawBlinkers();
  void drawBlinkerLeft();
  void drawBlinkerRight();
  void drawBlindSpots();
  void drawBlindSpotLeft();
  void drawBlindSpotRight();
  void drawTopbar();
  void drawSidebar();
  void drawBottombar();
  void drawColoredText(QPainter &p, int x, int y, const QString &text, QColor color);

  QPixmap engage_img;
  QPixmap dm_img;
  const int radius = 192;
  const int img_size = (radius / 2) * 1.5;
  QString speed;
  QString speedUnit;
  QString maxSpeed;
  bool is_cruise_set = false;
  bool engageable = false;
  bool dmActive = false;
  bool hideDM = false;
  int status = STATUS_DISENGAGED;

  // dp
  QString currentDateTime;
  bool mapd_enabled = false;
  QString mapd_info;
  float angleSteers = 0.0;
  float angleSteersDes = 0.0;
  float relativeDist = 0.0;
  float relativeSpeed = 0.0;
  int engineRPM = 0;
  bool brakeLights = false;

  bool leftBlindspot = false;
  bool rightBlindspot = false;
  bool steeringPressed = false;
  bool leftBlinker = false;
  bool rightBlinker = false;
  int blinkerOffset = 0;
  int blinkerAlpha = 200;


  // toggles
  bool dpUiSpeed = true;
  bool dpUiEvent = true;
  bool dpUiMaxSpeed = true;
  bool dpUiFace = true;
  bool dpUiLane = true;
  bool dpUiLead = true;
  bool dpUiSide = false;
  bool dpUiTop = false;
  bool dpUiBlinker = true;
  bool dpFollowingProfileCtrl = false;
  bool dpFollowingProfile = 0;
  bool dpAccelProfileCtrl = false;
  bool dpAccelProfile = 0;
  bool dpMapd = false;
  bool dpLaneLessModeCtrl = 0;
  bool dpLaneLessMode = false;

signals:
  void valueChanged();
};

class OnroadAlerts : public QWidget {
  Q_OBJECT

public:
  OnroadAlerts(QWidget *parent = 0) : QWidget(parent) {};
  void updateAlert(const Alert &a, const QColor &color);

protected:
  void paintEvent(QPaintEvent*) override;

private:
  QColor bg;
  Alert alert = {};
};

// container window for the NVG UI
class NvgWindow : public CameraViewWidget {
  Q_OBJECT

public:
  explicit NvgWindow(VisionStreamType type, QWidget* parent = 0) : CameraViewWidget("camerad", type, true, parent) {}

protected:
  void paintGL() override;
  void initializeGL() override;
  void showEvent(QShowEvent *event) override;
  void updateFrameMat(int w, int h) override;
  void drawLaneLines(QPainter &painter, const UIScene &scene);
  void drawLead(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd);
  inline QColor redColor(int alpha = 255) { return QColor(201, 34, 49, alpha); }
  double prev_draw_t = 0;
};

// container for all onroad widgets
class OnroadWindow : public QWidget {
  Q_OBJECT

public:
  OnroadWindow(QWidget* parent = 0);
  bool isMapVisible() const { return map && map->isVisible(); }

private:
  void paintEvent(QPaintEvent *event);
  void mousePressEvent(QMouseEvent* e) override;
  OnroadHud *hud;
  OnroadAlerts *alerts;
  NvgWindow *nvg;
  QColor bg = bg_colors[STATUS_DISENGAGED];
  QWidget *map = nullptr;
  QHBoxLayout* split;
  bool dp_allow_map = false;

signals:
  void updateStateSignal(const UIState &s);
  void offroadTransitionSignal(bool offroad);

private slots:
  void offroadTransition(bool offroad);
  void updateState(const UIState &s);
};
