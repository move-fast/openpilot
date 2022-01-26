#pragma once

#include <QStackedLayout>
#include <QWidget>

#include "selfdrive/ui/qt/widgets/cameraview.h"
#include "selfdrive/ui/ui.h"

// dp
const int dp_btn_wh = 160;
const int dp_btn_wh_gap = 30;
const int dp_sidebar_width = 192; // same as event icon radius
#include "selfdrive/common/params.h"

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
  Q_PROPERTY(bool dpUiSpeed MEMBER dpUiSpeed NOTIFY valueChanged);
  Q_PROPERTY(bool dpUiEvent MEMBER dpUiEvent NOTIFY valueChanged);
  Q_PROPERTY(bool dpUiFace MEMBER dpUiFace NOTIFY valueChanged);
  Q_PROPERTY(bool dpUiMaxSpeed MEMBER dpUiMaxSpeed NOTIFY valueChanged);
  Q_PROPERTY(bool showVTC MEMBER showVTC NOTIFY valueChanged);
  Q_PROPERTY(QString vtcSpeed MEMBER vtcSpeed NOTIFY valueChanged);
  Q_PROPERTY(QColor vtcColor MEMBER vtcColor NOTIFY valueChanged);
  Q_PROPERTY(bool showDebugUI MEMBER showDebugUI NOTIFY valueChanged);

  Q_PROPERTY(bool showSpeedLimit MEMBER showSpeedLimit NOTIFY valueChanged);
  Q_PROPERTY(QString speedLimit MEMBER speedLimit NOTIFY valueChanged);
  Q_PROPERTY(QString slcSubText MEMBER slcSubText NOTIFY valueChanged);
  Q_PROPERTY(float slcSubTextSize MEMBER slcSubTextSize NOTIFY valueChanged);
  Q_PROPERTY(bool mapSourcedSpeedLimit MEMBER mapSourcedSpeedLimit NOTIFY valueChanged);
  Q_PROPERTY(bool slcActive MEMBER slcActive NOTIFY valueChanged);

  Q_PROPERTY(bool showTurnSpeedLimit MEMBER showTurnSpeedLimit NOTIFY valueChanged);
  Q_PROPERTY(QString turnSpeedLimit MEMBER turnSpeedLimit NOTIFY valueChanged);
  Q_PROPERTY(QString tscSubText MEMBER tscSubText NOTIFY valueChanged);
  Q_PROPERTY(bool tscActive MEMBER tscActive NOTIFY valueChanged);
  Q_PROPERTY(int curveSign MEMBER curveSign NOTIFY valueChanged);

public:
  explicit OnroadHud(QWidget *parent);
  void updateState(const UIState &s);

private:
  void drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity);
  void drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255);
  void drawCenteredText(QPainter &p, int x, int y, const QString &text, QColor color);
  void drawVisionTurnControllerUI(QPainter &p, int x, int y, int size, const QColor &color, const QString &speed,
                                  int alpha);
  void drawCircle(QPainter &p, int x, int y, int r, QBrush bg);
  void drawSpeedSign(QPainter &p, QRect rc, const QString &speed, const QString &sub_text, int subtext_size,
                     bool is_map_sourced, bool is_active);
  void drawTrunSpeedSign(QPainter &p, QRect rc, const QString &speed, const QString &sub_text, int curv_sign,
                         bool is_active);
  void paintEvent(QPaintEvent *event) override;

  QPixmap engage_img;
  QPixmap dm_img;
  QPixmap map_img;
  QPixmap left_img;
  QPixmap right_img;
  const int radius = 192;
  const int img_size = (radius / 2) * 1.5;
  const int subsign_img_size = 35;
  QString speed;
  QString speedUnit;
  QString maxSpeed;
  bool is_cruise_set = false;
  bool engageable = false;
  bool dmActive = false;
  bool hideDM = false;
  int status = STATUS_DISENGAGED;

  // dp
  bool dpUiSpeed = true;
  bool dpUiEvent = true;
  bool dpUiFace = true;
  bool dpUiMaxSpeed = true;
  bool showVTC = false;
  QString vtcSpeed;
  QColor vtcColor;
  bool showDebugUI = false;

  QString roadName;

  bool showSpeedLimit = false;
  QString speedLimit;
  QString slcSubText;
  float slcSubTextSize = 0.0;
  bool mapSourcedSpeedLimit = false;
  bool slcActive = false;

  bool showTurnSpeedLimit = false;
  QString turnSpeedLimit;
  QString tscSubText;
  bool tscActive = false;
  int curveSign = 0;

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
  void drawLead(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd, bool has_op_long);
  inline QColor redColor(int alpha = 255) { return QColor(201, 34, 49, alpha); }
  double prev_draw_t = 0;

  // dp
  int blinkerOffset = 0;
  int blinkerAlpha = 200;
  float calc_ttc(float v_ego, float a_ego, float x_lead, float v_lead, float a_lead);
  void drawColoredText(QPainter &p, int x, int y, const QString &text, QColor color);
  void drawBlinkers(QPainter &painter, const cereal::CarState::Reader &cs);
  void drawBlindSpots(QPainter &painter, const cereal::CarState::Reader &cs);
  void drawTopbar(QPainter &painter, const cereal::CarState::Reader &cs);
  void drawSidebar(QPainter &painter, const cereal::ModelDataV2::Reader &lead_reader, const cereal::CarState::Reader &cs, const cereal::ControlsState::Reader &css, bool is_metric);
  void drawBottombar(QPainter &painter, const cereal::LiveMapData::Reader &lmd);
  void drawBtns(QPainter &painter);
  void paintBtn(QPainter &painter, int x, int y, QString main, QString sub);
};

// container for all onroad widgets
class OnroadWindow : public QWidget {
  Q_OBJECT

public:
  OnroadWindow(QWidget* parent = 0);
  bool isMapVisible() const { return map && map->isVisible(); }

private:
  void paintEvent(QPaintEvent *event);
  void btnTriggered(std::string param, int val, int min, int max);
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
