#include <QApplication>
#include <QCommandLineParser>
#include <QMainWindow>
#include <QStyleFactory>
#include "GeoViewWidget.h"


int main(int argc, char* argv[]) {
  QApplication app(argc, argv);
  app.setApplicationName("Map");
  app.setStyle(QStyleFactory::create("Fusion"));

  QCommandLineParser parser;
  parser.addHelpOption();
  parser.addVersionOption();
  parser.process(app);

  // Top-level QMainWindow avoids Windows title-bar hit issues after maximize when
  // the heavy UI (QGraphicsView map) lives in the central widget.
  QMainWindow mainWindow;
  mainWindow.setWindowFlags(Qt::Window | Qt::WindowMinimizeButtonHint |
                            Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint);
  auto* mapWidget = new GeoViewWidget(&mainWindow);
  mainWindow.setCentralWidget(mapWidget);
  mainWindow.setWindowTitle(mapWidget->windowTitle());
  mainWindow.resize(1280, 720);
  mainWindow.show();
  return app.exec();
}
