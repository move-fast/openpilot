#pragma once
#include <QVBoxLayout>
#include <QWidget>
#include <QLabel>

class DPGeneralPanel : public QWidget {
  Q_OBJECT
public:
  explicit DPGeneralPanel(QWidget* parent = nullptr);
  void showDisclaimer();

private:
  void autoShutdownWidget(QVBoxLayout * layout);
  void regWidget(QVBoxLayout * layout);
  QLabel *description;
};

class DPControlPanel : public QWidget {
  Q_OBJECT
public:
  explicit DPControlPanel(QWidget* parent = nullptr);

private:
  void laneChangeWidget(QVBoxLayout * layout);
  void longitudinalWidget(QVBoxLayout * layout);
  void steeringRatioWidget(QVBoxLayout * layout);
};

class DPUIPanel : public QWidget {
  Q_OBJECT
public:
  explicit DPUIPanel(QWidget* parent = nullptr);

private:
  void laneChangeWidget(QVBoxLayout * layout);
  void longitudinalWidget(QVBoxLayout * layout);
  void steeringRatioWidget(QVBoxLayout * layout);
};

class DPCarPanel : public QWidget {
  Q_OBJECT
public:
  explicit DPCarPanel(QWidget* parent = nullptr);

private:
  void carModelWidget(QVBoxLayout * layout);
  void toyotaSpeedOverrideWidget(QVBoxLayout * layout);
  void steeringRatioWidget(QVBoxLayout * layout);
};

class NavQRWidget : public QWidget {
  Q_OBJECT

public:
  explicit NavQRWidget(QWidget* parent = 0);
  void updateIpAddr(QString addr);

private:
  QLabel* qrCode;
  void showEvent(QShowEvent *event) override;
  QString ipAddr;
};

class DPMapsPanel : public QWidget {
  Q_OBJECT
public:
  explicit DPMapsPanel(QWidget* parent = nullptr);
  void updateMapsPanelIpAddr(QString addr);
private:
  void dpMapBox(QVBoxLayout * layout);
  void dpMapd(QVBoxLayout * layout);
  NavQRWidget* qrWidget;
  QString ipAddr;

signals:
  void updateIpAddr(QString addr);
};
