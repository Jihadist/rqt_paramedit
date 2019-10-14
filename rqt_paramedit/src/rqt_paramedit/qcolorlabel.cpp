#include "rqt_paramedit/qcolorlabel.h"

void QColorLabel::changeBackgroundColor(const QColor& color)
{
  QPalette pal = palette();
  pal.setColor(QPalette::Window, color);
  setPalette(pal);
}

void QColorLabel::changeBackgroundColorWithTimer(const QColor& color, int timeout)
{
  QColor defaultColor = palette().color(QPalette::Window);
  changeBackgroundColor(color);
  setText("Success");
  QTimer::singleShot(timeout, [this, defaultColor]() {
    changeBackgroundColor(defaultColor);
    clear();
  });
}
