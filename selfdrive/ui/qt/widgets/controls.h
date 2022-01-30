#pragma once

#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>

#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/widgets/toggle.h"

// dp
#include <chrono>
#include <string>
namespace sc = std::chrono;
using std::to_string;
#include <QSpinBox>
#include <QDoubleSpinBox>

QFrame *horizontal_line(QWidget *parent = nullptr);

class ElidedLabel : public QLabel {
  Q_OBJECT

public:
  explicit ElidedLabel(QWidget *parent = 0);
  explicit ElidedLabel(const QString &text, QWidget *parent = 0);

signals:
  void clicked();

protected:
  void paintEvent(QPaintEvent *event) override;
  void resizeEvent(QResizeEvent* event) override;
  void mouseReleaseEvent(QMouseEvent *event) override { emit clicked(); }
  QString lastText_, elidedText_;
};


class AbstractControl : public QFrame {
  Q_OBJECT

public:
  void setDescription(const QString &desc) {
    if (description) description->setText(desc);
  }

  void setTitle(const QString &title) {
    title_label->setText(title);
  }

signals:
  void showDescription();

protected:
  AbstractControl(const QString &title, const QString &desc = "", const QString &icon = "", QWidget *parent = nullptr);
  void hideEvent(QHideEvent *e) override;

  QHBoxLayout *hlayout;
  QPushButton *title_label;
  QLabel *description = nullptr;
};

// widget to display a value
class LabelControl : public AbstractControl {
  Q_OBJECT

public:
  LabelControl(const QString &title, const QString &text = "", const QString &desc = "", QWidget *parent = nullptr) : AbstractControl(title, desc, "", parent) {
    label.setText(text);
    label.setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    hlayout->addWidget(&label);
  }
  void setText(const QString &text) { label.setText(text); }

private:
  ElidedLabel label;
};

// widget for a button with a label
class ButtonControl : public AbstractControl {
  Q_OBJECT

public:
  ButtonControl(const QString &title, const QString &text, const QString &desc = "", QWidget *parent = nullptr);
  inline void setText(const QString &text) { btn.setText(text); }
  inline QString text() const { return btn.text(); }

signals:
  void clicked();

public slots:
  void setEnabled(bool enabled) { btn.setEnabled(enabled); };

private:
  QPushButton btn;
};

class ToggleControl : public AbstractControl {
  Q_OBJECT

public:
  ToggleControl(const QString &title, const QString &desc = "", const QString &icon = "", const bool state = false, QWidget *parent = nullptr) : AbstractControl(title, desc, icon, parent) {
    toggle.setFixedSize(150, 100);
    if (state) {
      toggle.togglePosition();
    }
    hlayout->addWidget(&toggle);
    QObject::connect(&toggle, &Toggle::stateChanged, this, &ToggleControl::toggleFlipped);
  }

  void setEnabled(bool enabled) { toggle.setEnabled(enabled); }

signals:
  void toggleFlipped(bool state);

protected:
  Toggle toggle;
};

// widget to toggle params
class ParamControl : public ToggleControl {
  Q_OBJECT

public:
  ParamControl(const QString &param, const QString &title, const QString &desc, const QString &icon, QWidget *parent = nullptr) : ToggleControl(title, desc, icon, false, parent) {
    key = param.toStdString();
    QObject::connect(this, &ToggleControl::toggleFlipped, [=](bool state) {
      params.putBool(key, state);

      // touch dp_last_modified
      auto time = sc::system_clock::now(); // get the current time
      auto since_epoch = time.time_since_epoch(); // get the duration since epoch
      auto millis = sc::duration_cast<sc::milliseconds>(since_epoch);
      params.put("dp_last_modified", std::to_string(millis.count()));
    });
  }

  void showEvent(QShowEvent *event) override {
    if (params.getBool(key) != toggle.on) {
      toggle.togglePosition();
    }
  };

private:
  std::string key;
  Params params;
};

class ListWidget : public QWidget {
  Q_OBJECT
 public:
  explicit ListWidget(QWidget *parent = 0) : QWidget(parent), outer_layout(this) {
    outer_layout.setMargin(0);
    outer_layout.setSpacing(0);
    outer_layout.addLayout(&inner_layout);
    inner_layout.setMargin(0);
    inner_layout.setSpacing(25); // default spacing is 25
    outer_layout.addStretch();
  }
  inline void addItem(QWidget *w) { inner_layout.addWidget(w); }
  inline void addItem(QLayout *layout) { inner_layout.addLayout(layout); }
  inline void setSpacing(int spacing) { inner_layout.setSpacing(spacing); }

private:
  void paintEvent(QPaintEvent *) override {
    QPainter p(this);
    p.setPen(Qt::gray);
    for (int i = 0; i < inner_layout.count() - 1; ++i) {
      QRect r = inner_layout.itemAt(i)->geometry();
      int bottom = r.bottom() + inner_layout.spacing() / 2;
      p.drawLine(r.left() + 40, bottom, r.right() - 40, bottom);
    }
  }
  QVBoxLayout outer_layout;
  QVBoxLayout inner_layout;
};
// dp
class SpinBoxControl : public AbstractControl {
  Q_OBJECT

public:
  SpinBoxControl(const QString &title, const QString &desc = "", const QString &icon = "", const int val = 0, QWidget *parent = nullptr) : AbstractControl(title, desc, icon, parent) {
    spinbox.setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    spinbox.setAlignment(Qt::AlignCenter);
    spinbox.setStyleSheet(R"(
      QSpinBox {
        width: 300px;
        height: 80px;
        padding: 0px;
        border-radius:40px;
      }

      QSpinBox::up-button  {
        subcontrol-origin: margin;
        subcontrol-position: center right;
        image: url(qt/images/icon_plus.png);
        right: 1px;
        height: 80px;
        width: 80px;
      }

      QSpinBox::down-button  {
        subcontrol-origin: margin;
        subcontrol-position: center left;
        image: url(qt/images/icon_minus.png);
        left: 1px;
        height: 80px;
        width: 80px;
      }
    )");

    hlayout->addWidget(&spinbox);
    QObject::connect(&spinbox, QOverload<int>::of(&QSpinBox::valueChanged), this, &SpinBoxControl::valChanged);
  }

signals:
  void valChanged(int val);

protected:
  QSpinBox spinbox;
};

class ParamSpinBoxControl : public SpinBoxControl {
  Q_OBJECT

public:
  ParamSpinBoxControl(const QString &param, const QString &title, const QString &desc, const QString &icon, const int minimum = 0, const int maximum = 100, const int step = 1, const QString suffix = "", const QString minText = "", QWidget *parent = nullptr) : SpinBoxControl(title, desc, icon, 0, parent) {
    spinbox.setRange(minimum, maximum);
    spinbox.setSingleStep(step);
    spinbox.setSuffix(suffix);
    spinbox.setSpecialValueText(minText);

    std::string str_val = Params().get(param.toStdString().c_str());
    if (str_val != "") {
      int val = std::stoi(str_val);
      spinbox.setValue(val);
      QObject::connect(this, &SpinBoxControl::valChanged, [=](int val) {
        Params().put(param.toStdString(), std::to_string(val));

        // touch dp_last_modified
        auto time = sc::system_clock::now(); // get the current time
        auto since_epoch = time.time_since_epoch(); // get the duration since epoch
        auto millis = sc::duration_cast<sc::milliseconds>(since_epoch);
        Params().put("dp_last_modified", std::to_string(millis.count()));
      });
    }
  }
};

class DoubleSpinBoxControl : public AbstractControl {
  Q_OBJECT

public:
  DoubleSpinBoxControl(const QString &title, const QString &desc = "", const QString &icon = "", const double val = 0.0, QWidget *parent = nullptr) : AbstractControl(title, desc, icon, parent) {
    spinbox.setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    spinbox.setAlignment(Qt::AlignCenter);
    spinbox.setStyleSheet(R"(
      QDoubleSpinBox {
        width: 300px;
        height: 80px;
        padding: 0px;
        border-radius:40px;
      }

      QDoubleSpinBox::up-button  {
        subcontrol-origin: margin;
        subcontrol-position: center right;
        image: url(qt/images/icon_plus.png);
        right: 1px;
        height: 80px;
        width: 80px;
      }

      QDoubleSpinBox::down-button  {
        subcontrol-origin: margin;
        subcontrol-position: center left;
        image: url(qt/images/icon_minus.png);
        left: 1px;
        height: 80px;
        width: 80px;
      }
    )");

    hlayout->addWidget(&spinbox);
    QObject::connect(&spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &DoubleSpinBoxControl::valChanged);
  }

signals:
  void valChanged(double val);

protected:
  QDoubleSpinBox spinbox;
};

class ParamDoubleSpinBoxControl : public DoubleSpinBoxControl {
  Q_OBJECT

public:
  ParamDoubleSpinBoxControl(const QString &param, const QString &title, const QString &desc, const QString &icon, const double minimum = 0.0, const double maximum = 100.0, const double step = 1.0, const QString suffix = "", const QString minText = "", QWidget *parent = nullptr) : DoubleSpinBoxControl(title, desc, icon, 0.0, parent) {
    spinbox.setRange(minimum, maximum);
    spinbox.setSingleStep(step);
    spinbox.setSuffix(suffix);
    spinbox.setSpecialValueText(minText);

    std::string str_val = Params().get(param.toStdString().c_str());
    if (str_val != "") {
      double val = std::stod(str_val);
      spinbox.setValue(val);

      QObject::connect(this, &DoubleSpinBoxControl::valChanged, [=](double val) {
        Params().put(param.toStdString(), std::to_string(val));

        // touch dp_last_modified
        auto time = sc::system_clock::now(); // get the current time
        auto since_epoch = time.time_since_epoch(); // get the duration since epoch
        auto millis = sc::duration_cast<sc::milliseconds>(since_epoch);
        Params().put("dp_last_modified", std::to_string(millis.count()));
      });
    }
  }
  void setValue(float v) { spinbox.setValue(v); }
};