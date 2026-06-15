#pragma once
#include <QGeoView/QGVGlobal.h>
#include <QDialog>
#include <QVector>


class ContourPreviewDialog : public QDialog {
  Q_OBJECT
 public:
  explicit ContourPreviewDialog(const QVector<QGV::GeoPos>& points,
                                const QVector<QGV::GeoPos>& intersections,
                                QWidget* parent = nullptr);

 protected:
  void paintEvent(QPaintEvent* event) override;

 private:
  QVector<QGV::GeoPos> m_points;
  QVector<QGV::GeoPos> m_intersections;
};
