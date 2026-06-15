#include "GeoViewUiComposer.h"

#include "../GeoViewWidget.h"
#include "../features/GeoViewRouteFeature.h"
#include "../features/algorithm/pipeline/RoutePipelineDebug.h"
#include "../features/algorithm/pipeline/RoutePipelineTypes.h"

#include <memory>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QSlider>
#include <QTabWidget>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

QGroupBox* GeoViewUiComposer::createOptionsList(GeoViewWidget& view) {
  QGroupBox* groupBox = new QGroupBox(view.tr("Управление"));
  auto* rootLayout = new QVBoxLayout;
  rootLayout->setContentsMargins(8, 8, 8, 8);
  rootLayout->setSpacing(6);
  groupBox->setLayout(rootLayout);
  groupBox->setMinimumWidth(280);
  groupBox->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
  groupBox->setObjectName("controlRoot");
  groupBox->setStyleSheet(
      "#controlRoot { color: #F4F7FB; font-size: 14px; font-weight: 700; "
      "background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #202734, stop:1 #151A23); "
      "border: 1px solid #334155; border-radius: 14px; }"
      "#controlRoot::title { subcontrol-origin: margin; left: 14px; padding: 0 6px; }"
      "#controlRoot QPushButton { min-height: 36px; padding: 8px 12px; color: #F8FAFC; "
      "background-color: #2563EB; border: 1px solid #3B82F6; border-radius: 9px; font-weight: 700; }"
      "#controlRoot QPushButton:hover { background-color: #3B82F6; }"
      "#controlRoot QPushButton:pressed { background-color: #1D4ED8; }"
      "#controlRoot QPushButton:disabled { color: #8A95A6; background-color: #263244; "
      "border-color: #334155; }"
      "QGroupBox { color: #E7EDF7; font-size: 13px; font-weight: 700; "
      "background-color: rgba(255,255,255,0.04); border: 1px solid #334155; "
      "border-radius: 12px; margin-top: 10px; }"
      "QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; }");

  auto addSection = [&](const QString& title) {
    auto* box = new QGroupBox(title);
    auto* sectionLayout = new QVBoxLayout;
    sectionLayout->setContentsMargins(6, 8, 6, 6);
    sectionLayout->setSpacing(8);
    box->setLayout(sectionLayout);
    box->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    rootLayout->addWidget(box);
    return box;
  };
  auto addActionButton = [&](QVBoxLayout* sectionLayout, const QString& text) {
    auto* button = new QPushButton(text);
    button->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    sectionLayout->addWidget(button);
    return button;
  };
  auto* prepLayout = qobject_cast<QVBoxLayout*>(addSection(view.tr("1) Подготовка"))->layout());
  auto* routeLayout = qobject_cast<QVBoxLayout*>(addSection(view.tr("2) Маршрут"))->layout());
  auto* commandLayout = qobject_cast<QVBoxLayout*>(addSection(view.tr("3) Команды и экспорт"))->layout());
  auto* dangerLayout = qobject_cast<QVBoxLayout*>(addSection(view.tr("Сервис"))->layout());

  {
    QPushButton* button = addActionButton(
        routeLayout, view.tr("Указать начальную позицию\nробота на карте"));
    QObject::connect(button, &QPushButton::clicked, &view, [&view]() {
      QTimer::singleShot(250, &view, [&view]() {
        view.setInteractionMode(GeoViewWidget::InteractionMode::PlaceRobot);
        view.setUiStatus(
            view.tr("Кликните по карте, чтобы поставить или переместить робота (угол сохраняется)."),
            GeoViewWidget::UiStatusLevel::Info);
      });
    });
  }

  {
    QPushButton* button = addActionButton(prepLayout, view.tr("Добавить контур (GeoJson)"));
    QObject::connect(button, &QPushButton::clicked, &view, [&view]() {
      view.addContour();
    });
  }

  {
    QPushButton* button = addActionButton(prepLayout, view.tr("Задать контур на карте"));
    QObject::connect(button, &QPushButton::clicked, &view, [&view]() {
      view.startDrawContourOnMap();
    });
  }

  {
    QPushButton* button = addActionButton(prepLayout, view.tr("Отменить задание контура"));
    button->setVisible(false);
    view.mCancelDrawContourButton = button;
    QObject::connect(button, &QPushButton::clicked, &view, [&view]() {
      view.cancelDrawContour();
      view.setUiStatus(view.tr("Задание контура отменено"), GeoViewWidget::UiStatusLevel::Info);
    });
  }

  {
    QPushButton* button = addActionButton(prepLayout, view.tr("Нарисовать область выреза"));
    QObject::connect(button, &QPushButton::clicked, &view, [&view]() {
      view.startCutoutPolygonMode();
    });
  }

  {
    QPushButton* button = addActionButton(prepLayout, view.tr("Убрать последний вырез"));
    QObject::connect(button, &QPushButton::clicked, &view, [&view]() {
      view.undoLastCutout();
    });
  }

  {
    QPushButton* button = addActionButton(routeLayout, view.tr("Построение маршрута"));
    QObject::connect(button, &QPushButton::clicked, &view,
                     [&view]() { view.mSidePanelStack->setCurrentIndex(1); });
  }

  {
    QPushButton* button = addActionButton(commandLayout, view.tr("Построить команды"));
    QObject::connect(button, &QPushButton::clicked, &view, [&view]() {
      view.createRoute();
      view.showRobotCommandsJson();
    });
  }

  {
    QPushButton* button = addActionButton(routeLayout, view.tr("Указать маршрут вручную"));
    QObject::connect(button, &QPushButton::clicked, &view, [&view, button]() {
      view.toggleManualRouteMode();
      const bool manualEnabled =
          (view.interactionMode() == GeoViewWidget::InteractionMode::ManualRoute);
      button->setText(manualEnabled ? view.tr("Завершить построение маршрута")
                                    : view.tr("Указать маршрут вручную"));
    });
  }

  {
    QPushButton* button = addActionButton(dangerLayout, view.tr("Удалить контур"));
    QObject::connect(button, &QPushButton::clicked, &view, [&view]() {
      view.removeContour();
      view.setUiStatus(view.tr("Контур удален"), GeoViewWidget::UiStatusLevel::Info);
    });
  }

  {
    QPushButton* button = addActionButton(dangerLayout, view.tr("Очистить все"));
    QObject::connect(button, &QPushButton::clicked, &view, &GeoViewWidget::clearAll);
  }

  {
    QPushButton* button = addActionButton(
        commandLayout, view.tr("Сгенерировать json\nдля симуляции в Gazebo"));
    QObject::connect(button, &QPushButton::clicked, &view, [&view]() {
      const QJsonDocument jsonDoc = view.generateGazeboJson();
      if (jsonDoc.isEmpty())
        return;

      const QString filename = QFileDialog::getSaveFileName(
          &view, view.tr("Сохранить JSON"), QString(), view.tr("JSON Files (*.json)"));
      if (filename.isEmpty())
        return;
      QFile file(filename);
      if (file.open(QIODevice::WriteOnly)) {
        file.write(jsonDoc.toJson());
        file.close();
        view.setUiStatus(view.tr("JSON для Gazebo сохранен"), GeoViewWidget::UiStatusLevel::Success);
      } else {
        QMessageBox::warning(&view, view.tr("Ошибка"), view.tr("Не удалось сохранить файл"));
        view.setUiStatus(view.tr("Не удалось сохранить JSON"), GeoViewWidget::UiStatusLevel::Error);
      }
    });
  }

  {
    QPushButton* button = addActionButton(
        commandLayout, view.tr("Сохранить waypoints\nдля Gazebo (JSON)"));
    QObject::connect(button, &QPushButton::clicked, &view, [&view]() {
      const QJsonDocument jsonDoc = view.generateGazeboWaypointsJson();
      if (jsonDoc.isEmpty())
        return;

      const QString filename = QFileDialog::getSaveFileName(
          &view, view.tr("Сохранить waypoints"),
          QStringLiteral("gazebo_waypoints.json"),
          view.tr("JSON Files (*.json)"));
      if (filename.isEmpty())
        return;
      QFile file(filename);
      if (file.open(QIODevice::WriteOnly)) {
        file.write(jsonDoc.toJson(QJsonDocument::Indented));
        file.close();
        view.setUiStatus(view.tr("Waypoints для Gazebo сохранены"), GeoViewWidget::UiStatusLevel::Success);
      } else {
        QMessageBox::warning(&view, view.tr("Ошибка"), view.tr("Не удалось сохранить файл"));
        view.setUiStatus(view.tr("Не удалось сохранить waypoints"), GeoViewWidget::UiStatusLevel::Error);
      }
    });
  }

  groupBox->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);

  return groupBox;
}

QGroupBox* GeoViewUiComposer::createInfoList(GeoViewWidget& view) {
  QGroupBox* groupBox = new QGroupBox(view.tr("Info"));
  auto* infoLay = new QVBoxLayout(groupBox);
  infoLay->setContentsMargins(10, 12, 10, 10);
  infoLay->setSpacing(8);
  groupBox->setMinimumWidth(280);
  groupBox->setMinimumHeight(120);
  groupBox->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  groupBox->setObjectName("infoCard");
  groupBox->setStyleSheet(
      "#infoCard { color: #F4F7FB; font-size: 14px; font-weight: 700; "
      "background-color: #151C2A; border: 1px solid #2A3A52; border-radius: 12px; }"
      "#infoCard::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; }"
      "QListWidget { background-color: #0F172A; color: #E5EBF4; border: 1px solid #263244; "
      "border-radius: 8px; padding: 4px; }");

  view.mInfoWidget = new QListWidget();
  view.mInfoWidget->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
  view.mInfoWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  view.mInfoWidget->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  view.mInfoWidget->setMinimumHeight(80);
  view.mInfoWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  view.mStatusLabel = new QLabel(view.tr("Готово к работе"));
  view.mStatusLabel->setWordWrap(true);
  view.mStatusLabel->setMinimumHeight(26);
  infoLay->addWidget(view.mStatusLabel, 0);
  infoLay->addWidget(view.mInfoWidget, 1);
  view.updateInfoList();
  return groupBox;
}

QWidget* GeoViewUiComposer::createRouteParamsWidget(GeoViewWidget& view) {
  QGroupBox* box = new QGroupBox(view.tr("Параметры маршрута"));
  QVBoxLayout* l = new QVBoxLayout(box);
  l->setContentsMargins(12, 12, 12, 12);
  l->setSpacing(10);
  box->setMinimumWidth(280);
  box->setMinimumHeight(400);
  box->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  box->setObjectName("paramsCard");
  box->setStyleSheet(
      "#paramsCard { color: #F4F7FB; font-size: 14px; font-weight: 700; "
      "background-color: #151C2A; border: 1px solid #2A3A52; border-radius: 12px; }"
      "#paramsCard::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; }"
      "QLabel { color: #CBD5E1; font-size: 13px; }"
      "QPushButton { min-height: 36px; padding: 6px 14px; color: #F8FAFC; "
      "background-color: #2563EB; border: 1px solid #3B82F6; border-radius: 9px; font-weight: 700; }"
      "QPushButton:hover { background-color: #3B82F6; }"
      "QPushButton:pressed { background-color: #1D4ED8; }"
      "QDoubleSpinBox, QComboBox { min-height: 34px; background-color: #0F172A; color: #E5EBF4; "
      "border: 1px solid #334155; border-radius: 8px; padding: 3px 8px; }"
      "QDoubleSpinBox:focus, QComboBox:focus { border: 1px solid #60A5FA; }"
      "QCheckBox { color: #CBD5E1; spacing: 8px; }"
      "QToolButton { min-height: 32px; color: #E7ECF3; background-color: #1E293B; "
      "border: 1px solid #334155; border-radius: 8px; text-align: left; padding: 5px 10px; }"
      "QToolButton:hover { background-color: #27364D; }");

  const QSizePolicy growH(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);

  auto addFieldBlock = [&](const QString& title, QWidget* widget) {
    auto* titleLbl = new QLabel(title);
    titleLbl->setWordWrap(true);
    titleLbl->setContentsMargins(0, 6, 0, 4);
    l->addWidget(titleLbl);
    widget->setSizePolicy(growH);
    l->addWidget(widget);
  };

  QPushButton* selectStartBtn = new QPushButton(view.tr("Выбрать на карте"));
  addFieldBlock(view.tr("Стартовая точка на контуре"), selectStartBtn);
  QObject::connect(selectStartBtn, &QPushButton::clicked, &view, [&view]() {
    QTimer::singleShot(250, &view, [&view]() {
      view.setInteractionMode(GeoViewWidget::InteractionMode::SelectStartPoint);
      view.setUiStatus(view.tr("Выберите стартовую точку на контуре"), GeoViewWidget::UiStatusLevel::Info);
    });
  });

  QPushButton* selectEndBtn = new QPushButton(view.tr("Выбрать на карте"));
  addFieldBlock(view.tr("Конечная точка маршрута"), selectEndBtn);
  QObject::connect(selectEndBtn, &QPushButton::clicked, &view, [&view]() {
    QTimer::singleShot(250, &view, [&view]() {
      view.setInteractionMode(GeoViewWidget::InteractionMode::SelectEndPoint);
      view.setUiStatus(view.tr("Выберите конечную точку на карте"), GeoViewWidget::UiStatusLevel::Info);
    });
  });

  QDoubleSpinBox* angle = new QDoubleSpinBox();
  angle->setRange(0, 180);
  angle->setSingleStep(5);
  angle->setValue(90.0);
  addFieldBlock(view.tr("Угол наклона маршрута (°)"), angle);

  QDoubleSpinBox* step = new QDoubleSpinBox();
  step->setRange(0.1, 100);
  step->setValue(4.0);
  addFieldBlock(view.tr("Шаг между линиями (м)"), step);

  QDoubleSpinBox* contourOffset = new QDoubleSpinBox();
  contourOffset->setRange(0, 50);
  contourOffset->setValue(2.0);
  addFieldBlock(view.tr("Отступ от границы (м)"), contourOffset);

  QCheckBox* basicCoverageMode = new QCheckBox(
      view.tr("Упрощённый алгоритм\n(параллельное покрытие)"));
  basicCoverageMode->setChecked(false);
  basicCoverageMode->setSizePolicy(growH);
  basicCoverageMode->setToolTip(
      view.tr("Boustrophedon по inset-контуру (тот же угол, шаг и отступ, что у основного "
               "алгоритма). Вырезы не учитываются — при наличии вырезов используйте основной "
               "пайплайн."));
  l->addWidget(basicCoverageMode);

  QCheckBox* debugMode =
      new QCheckBox(view.tr("Debug режим маршрута\n(визуализация по стадиям)"));
  debugMode->setChecked(false);
  debugMode->setSizePolicy(growH);
  l->addWidget(debugMode);

  QPushButton* build = new QPushButton(view.tr("Построить маршрут"));
  build->setSizePolicy(growH);
  l->addWidget(build);

  // ----- Stage tabs (один блок на стадию) -----
  auto* tabs = new QTabWidget(box);
  tabs->setObjectName("stageTabs");
  tabs->setStyleSheet(
      "QTabWidget::pane { border: 1px solid #3A4352; border-radius: 6px; "
      "background: #1E232B; }"
      "QTabBar::tab { padding: 4px 8px; color: #CDD6E4; "
      "background: #2D3542; border: 1px solid #3A4352; border-bottom: none; "
      "border-top-left-radius: 4px; border-top-right-radius: 4px; }"
      "QTabBar::tab:selected { background: #3A4658; color: #FFFFFF; }"
      "QTabBar::tab:hover { background: #46556B; }"
      "QPlainTextEdit { background: #161A20; color: #C8D2E0; "
      "border: 1px solid #2A323D; border-radius: 4px; }"
      "QListWidget { background: #161A20; color: #C8D2E0; "
      "border: 1px solid #2A323D; border-radius: 4px; }");
  tabs->setTabPosition(QTabWidget::North);
  tabs->setMinimumHeight(320);
  tabs->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  tabs->tabBar()->setUsesScrollButtons(true);
  tabs->tabBar()->setExpanding(false);
  l->addWidget(tabs);

  QObject::connect(basicCoverageMode, &QCheckBox::toggled, &view,
                   [debugMode, tabs](bool on) {
    debugMode->setEnabled(!on);
    tabs->setEnabled(!on);
    if (on && debugMode->isChecked()) debugMode->setChecked(false);
  });

  struct StageTab {
    RouteAlgo::PipelineStage stage;
    QString title;
    QLabel* status = nullptr;
    QLabel* summary = nullptr;
    QListWidget* metrics = nullptr;
    QPlainTextEdit* log = nullptr;
    QSlider* stepSlider = nullptr;
    QLabel* stepLabel = nullptr;
    QPushButton* stepPrev = nullptr;
    QPushButton* stepNext = nullptr;
    QPushButton* stepReset = nullptr;
    QPushButton* stepPlay = nullptr;
    QTimer* playTimer = nullptr;
    QTimer* redrawDebounce = nullptr;
  };
  // Sequence in algorithm order; the user reads top-to-bottom and reaches
  // Approach last (which is the natural "operational" view).
  const QVector<QPair<RouteAlgo::PipelineStage, QString>> stageDefs = {
      {RouteAlgo::PipelineStage::Prepare,  view.tr("1. Подготовка")},
      {RouteAlgo::PipelineStage::Sweep,    view.tr("2. Полосы")},
      {RouteAlgo::PipelineStage::Filter,   view.tr("3. Фильтр")},
      {RouteAlgo::PipelineStage::Graph,    view.tr("4. Граф")},
      {RouteAlgo::PipelineStage::Merge,    view.tr("5. Слияние")},
      {RouteAlgo::PipelineStage::Routes,   view.tr("6. Цепочки")},
      {RouteAlgo::PipelineStage::Group,    view.tr("7. Острова")},
      {RouteAlgo::PipelineStage::Stitch,   view.tr("8. Сшивка")},
      {RouteAlgo::PipelineStage::Approach, view.tr("9. Маршрут")},
  };
  // shared_ptr makes the vector ref-counted across all lambda captures, so we
  // don't have to chase Qt parent ownership separately.
  auto stageTabs = std::make_shared<QVector<StageTab>>();
  stageTabs->reserve(stageDefs.size());
  for (const auto& def : stageDefs) {
    StageTab t;
    t.stage = def.first;
    t.title = def.second;
    auto* page = new QWidget(tabs);
    auto* pl = new QVBoxLayout(page);
    pl->setContentsMargins(8, 8, 8, 8);
    pl->setSpacing(6);
    t.status = new QLabel(view.tr("Стадия не выполнена"));
    t.status->setStyleSheet("font-weight: 600;");
    t.summary = new QLabel(QString());
    t.summary->setWordWrap(true);
    t.metrics = new QListWidget();
    t.metrics->setMaximumHeight(110);
    t.log = new QPlainTextEdit();
    t.log->setReadOnly(true);
    t.log->setMaximumBlockCount(500);

    // Per-stage playback controls.
    auto* playRow = new QHBoxLayout();
    t.stepPrev = new QPushButton(view.tr("◀"));
    t.stepNext = new QPushButton(view.tr("▶"));
    t.stepReset = new QPushButton(view.tr("⟲"));
    t.stepPlay = new QPushButton(view.tr("▶▶"));
    t.stepPlay->setCheckable(true);
    for (auto* b : {t.stepPrev, t.stepNext, t.stepReset, t.stepPlay}) {
      b->setMaximumWidth(48);
      b->setToolTip(view.tr("Шаг внутри стадии"));
    }
    t.stepLabel = new QLabel(view.tr("шаг: 0/0"));
    t.stepSlider = new QSlider(Qt::Horizontal);
    t.stepSlider->setRange(0, 0);
    t.stepSlider->setValue(0);
    t.playTimer = new QTimer(page);
    t.playTimer->setInterval(80);  // ~12 fps animation
    t.redrawDebounce = new QTimer(page);
    t.redrawDebounce->setSingleShot(true);
    t.redrawDebounce->setInterval(60);
    playRow->addWidget(t.stepPrev);
    playRow->addWidget(t.stepNext);
    playRow->addWidget(t.stepPlay);
    playRow->addWidget(t.stepReset);
    playRow->addWidget(t.stepLabel);

    pl->addWidget(t.status);
    pl->addWidget(t.summary);
    pl->addWidget(t.stepSlider);
    pl->addLayout(playRow);
    pl->addWidget(new QLabel(view.tr("Метрики:")));
    pl->addWidget(t.metrics);
    pl->addWidget(new QLabel(view.tr("Лог:")));
    pl->addWidget(t.log, 1);
    tabs->addTab(page, t.title);
    stageTabs->append(t);
  }

  // Slider/buttons callbacks: each one redraws using the current step value.
  auto redrawStage = [&view, debugMode](const StageTab& t) {
    if (!debugMode->isChecked()) return;
    view.setRouteVisualizationStage(t.stage, t.stepSlider->value());
  };
  auto updateStepLabel = [&view](StageTab& t) {
    t.stepLabel->setText(view.tr("шаг: %1/%2").arg(t.stepSlider->value()).arg(t.stepSlider->maximum()));
  };
  for (auto& t : *stageTabs) {
    QObject::connect(t.redrawDebounce, &QTimer::timeout, &view,
                     [stageTabs, redrawStage, slot = t.stage]() {
      for (const auto& tab : *stageTabs) {
        if (tab.stage == slot) {
          redrawStage(tab);
          break;
        }
      }
    });
    QObject::connect(t.stepSlider, &QSlider::valueChanged, &view,
                     [stageTabs, updateStepLabel, slot = t.stage](int) {
      for (auto& tab : *stageTabs) {
        if (tab.stage == slot) {
          updateStepLabel(tab);
          if (tab.redrawDebounce) tab.redrawDebounce->start();
          break;
        }
      }
    });
    QObject::connect(t.stepPrev, &QPushButton::clicked, &view,
                     [stageTabs, slot = t.stage]() {
      for (auto& tab : *stageTabs) {
        if (tab.stage == slot) {
          tab.stepSlider->setValue(std::max(0, tab.stepSlider->value() - 1));
          break;
        }
      }
    });
    QObject::connect(t.stepNext, &QPushButton::clicked, &view,
                     [stageTabs, slot = t.stage]() {
      for (auto& tab : *stageTabs) {
        if (tab.stage == slot) {
          tab.stepSlider->setValue(
              std::min(tab.stepSlider->maximum(), tab.stepSlider->value() + 1));
          break;
        }
      }
    });
    QObject::connect(t.stepReset, &QPushButton::clicked, &view,
                     [stageTabs, slot = t.stage]() {
      for (auto& tab : *stageTabs) {
        if (tab.stage == slot) {
          if (tab.stepPlay->isChecked()) tab.stepPlay->setChecked(false);
          tab.stepSlider->setValue(0);
          break;
        }
      }
    });
    QObject::connect(t.stepPlay, &QPushButton::toggled, &view,
                     [stageTabs, slot = t.stage](bool on) {
      for (auto& tab : *stageTabs) {
        if (tab.stage != slot) continue;
        if (on) {
          if (tab.stepSlider->value() >= tab.stepSlider->maximum()) {
            tab.stepSlider->setValue(0);
          }
          tab.playTimer->start();
        } else {
          tab.playTimer->stop();
        }
        break;
      }
    });
    QObject::connect(t.playTimer, &QTimer::timeout, &view,
                     [stageTabs, slot = t.stage]() {
      for (auto& tab : *stageTabs) {
        if (tab.stage != slot) continue;
        const int next = tab.stepSlider->value() + 1;
        if (next > tab.stepSlider->maximum()) {
          tab.playTimer->stop();
          tab.stepPlay->setChecked(false);
        } else {
          tab.stepSlider->setValue(next);
        }
        break;
      }
    });
  }

  auto refreshTabs = [&view, stageTabs]() {
    const auto& stages = GeoViewRouteFeature::lastPipelineStages();
    for (auto& tab : *stageTabs) {
      const RouteAlgo::StageInfo* info = nullptr;
      for (const auto& s : stages) {
        if (s.stage == tab.stage) {
          info = &s;
          break;
        }
      }
      if (!info) {
        tab.status->setText(view.tr("Стадия не выполнялась"));
        tab.status->setStyleSheet("color: #889; font-weight: 600;");
        tab.summary->setText(QString());
        if (tab.metrics) tab.metrics->clear();
        if (tab.log) tab.log->clear();
        continue;
      }
      QString color;
      QString statusText;
      switch (info->status) {
        case RouteAlgo::StageStatus::Ok:
          color = "#76E08A";
          statusText = view.tr("OK");
          break;
        case RouteAlgo::StageStatus::Failed:
          color = "#FF7070";
          statusText = view.tr("ОШИБКА");
          break;
        case RouteAlgo::StageStatus::Running:
          color = "#FFC04A";
          statusText = view.tr("выполняется");
          break;
        case RouteAlgo::StageStatus::Skipped:
          color = "#9AA4B5";
          statusText = view.tr("пропущена");
          break;
        case RouteAlgo::StageStatus::Pending:
          color = "#9AA4B5";
          statusText = view.tr("ожидание");
          break;
      }
      tab.status->setText(view.tr("[%1] %2 — %3 мс")
                              .arg(info->name, statusText)
                              .arg(static_cast<long long>(info->elapsedMs)));
      tab.status->setStyleSheet(QStringLiteral("color: %1; font-weight: 600;").arg(color));
      tab.summary->setText(info->message);
      tab.metrics->clear();
      for (const auto& kv : info->metrics) {
        tab.metrics->addItem(QStringLiteral("%1: %2").arg(kv.first, kv.second));
      }
      tab.log->setPlainText(info->log.join('\n'));
      // Refresh slider range based on what the visualizer can step over.
      const int maxStep = GeoViewRouteFeature::stageStepCount(tab.stage);
      const bool wasAtMax = tab.stepSlider->value() == tab.stepSlider->maximum() ||
                            tab.stepSlider->maximum() == 0;
      tab.stepSlider->blockSignals(true);
      tab.stepSlider->setRange(0, maxStep);
      tab.stepSlider->setValue(wasAtMax ? maxStep : std::min(tab.stepSlider->value(), maxStep));
      tab.stepSlider->blockSignals(false);
      tab.stepLabel->setText(
          view.tr("шаг: %1/%2").arg(tab.stepSlider->value()).arg(maxStep));
      const bool stepEnabled =
          (info->status == RouteAlgo::StageStatus::Ok && maxStep > 1);
      tab.stepSlider->setEnabled(stepEnabled);
      tab.stepPrev->setEnabled(stepEnabled);
      tab.stepNext->setEnabled(stepEnabled);
      tab.stepReset->setEnabled(stepEnabled);
      tab.stepPlay->setEnabled(stepEnabled);
      if (!stepEnabled && tab.stepPlay->isChecked()) {
        tab.stepPlay->setChecked(false);
      }
    }
  };

  // Switching tabs triggers re-rendering of that stage on the map (only when
  // the current build was invoked in debug mode — operational view never
  // overrides itself).
  QObject::connect(tabs, &QTabWidget::currentChanged, &view,
                   [stageTabs, &view, debugMode](int idx) {
    if (idx < 0 || idx >= static_cast<int>(stageTabs->size())) return;
    if (!debugMode->isChecked()) return;
    const auto& t = stageTabs->at(idx);
    view.setRouteVisualizationStage(t.stage, t.stepSlider->value());
  });

  QObject::connect(build, &QPushButton::clicked, &view,
                   [&view, step, angle, contourOffset, basicCoverageMode, debugMode, tabs, stageTabs,
                    refreshTabs]() {
    if (!view.mContour || view.mContour->points().size() < 3) {
      view.setUiStatus(view.tr("Построение невозможно: контур не задан"),
                       GeoViewWidget::UiStatusLevel::Warning);
      QMessageBox::warning(&view, view.tr("Нет контура"),
                           view.tr("Сначала добавьте корректный контур."));
      return;
    }
    if (step->value() <= 0.0) {
      view.setUiStatus(view.tr("Построение невозможно: шаг должен быть > 0"),
                       GeoViewWidget::UiStatusLevel::Warning);
      return;
    }

    if (basicCoverageMode->isChecked()) {
      if (!view.mCutoutPolygons.isEmpty()) {
        view.setUiStatus(view.tr("Упрощённый алгоритм не учитывает вырезы — маршрут может "
                                 "пересекать запретные зоны. Отключите упрощённый режим."),
                         GeoViewWidget::UiStatusLevel::Warning);
      }
      const bool hasEndPoint = view.mState.manualEndPoint.latitude() != 0.0 ||
                              view.mState.manualEndPoint.longitude() != 0.0;
      constexpr bool kDefaultRightSide = true;
      const bool ok = GeoViewRouteFeature::buildBasicParallelCoverageRoute(
          view, step->value(), angle->value(), contourOffset->value(),
          view.mState.manualStartPoint, view.mState.manualEndPoint, hasEndPoint,
          kDefaultRightSide);
      if (!ok) {
        view.setUiStatus(view.tr("Упрощённый алгоритм: маршрут не построен"),
                         GeoViewWidget::UiStatusLevel::Warning);
        QMessageBox::warning(&view, view.tr("Ошибка"),
                             view.tr("Не удалось построить покрытие. Проверьте контур, шаг и угол."));
        return;
      }
      const QString detail = view.tr("Упрощённый алгоритм: угол %1°, %2 точек")
                                 .arg(angle->value())
                                 .arg(view.mState.routePoints.size());
      view.setUiStatus(detail, GeoViewWidget::UiStatusLevel::Success);
      GeoViewRouteFeature::restoreRouteAnchorMarkers(view, view.mState.manualStartPoint,
                                                     view.mState.manualEndPoint, hasEndPoint);
      view.mSidePanelStack->setCurrentIndex(0);
      return;
    }

    const int currentTabIdx =
        std::clamp(tabs->currentIndex(), 0, static_cast<int>(stageTabs->size()) - 1);
    const RouteAlgo::PipelineStage stageToShow = debugMode->isChecked()
        ? stageTabs->at(currentTabIdx).stage
        : RouteAlgo::PipelineStage::Approach;
    constexpr bool kDefaultRightSide = true;
    for (auto& tab : *stageTabs) {
      if (tab.stepPlay && tab.stepPlay->isChecked()) tab.stepPlay->setChecked(false);
      if (tab.playTimer) tab.playTimer->stop();
    }
    view.buildRouteWithAngleForCustomRoute(step->value(), angle->value(), contourOffset->value(),
                                           view.mState.manualStartPoint, view.mState.manualEndPoint,
                                           kDefaultRightSide, debugMode->isChecked(), stageToShow);
    refreshTabs();
    const QString reason = GeoViewRouteFeature::lastBuildStatus();
    const bool ok = GeoViewRouteFeature::lastBuildSuccessful();
    if (ok && debugMode->isChecked()) {
      for (int i = 0; i < static_cast<int>(stageTabs->size()); ++i) {
        if (stageTabs->at(i).stage == RouteAlgo::PipelineStage::Approach) {
          tabs->setCurrentIndex(i);
          view.setRouteVisualizationStage(RouteAlgo::PipelineStage::Approach);
          break;
        }
      }
    }
    const int outerParts = GeoViewRouteFeature::lastPrepareOuterContourCount();
    QString msg = reason;
    GeoViewWidget::UiStatusLevel level =
        ok ? GeoViewWidget::UiStatusLevel::Success : GeoViewWidget::UiStatusLevel::Warning;
    const int unreachable = GeoViewRouteFeature::lastUnreachableIslandCount();
    if (unreachable > 0) {
      msg += QLatin1Char('\n');
      msg += view.tr("Внимание: %1 недостижимых островов — маршрут построен только по достижимым.")
                 .arg(unreachable);
      level = GeoViewWidget::UiStatusLevel::Warning;
    }
    if (outerParts > 1) {
      msg += QLatin1Char('\n');
      msg += view.tr("Внимание: мультиконтур — %1 независимых рабочих областей после отступа.")
                 .arg(outerParts);
      level = GeoViewWidget::UiStatusLevel::Warning;
    }
    view.setUiStatus(msg, level);
    if (!debugMode->isChecked()) {
      view.mSidePanelStack->setCurrentIndex(0);
    }
  });

  auto* scroll = new QScrollArea();
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scroll->setWidget(box);
  scroll->setStyleSheet("QScrollArea { background: transparent; border: none; }");
  return scroll;
}
