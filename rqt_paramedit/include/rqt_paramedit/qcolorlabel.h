#ifndef QCOLORLABEL_H
#define QCOLORLABEL_H

#include <QLabel>
#include <QObject>
#include <QTimer>

class QColorLabel : public QLabel
{
public:
  using QLabel::QLabel;
  void changeBackgroundColor(const QColor& color);
  void changeBackgroundColorWithTimer(const QColor& color = Qt::green, int timeout = 1000);
};

#endif  // QCOLORLABEL_H
