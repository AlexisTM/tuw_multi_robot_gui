#ifndef GOODDIALOG_H
#define GOODDIALOG_H

#include <iostream>
#include <QDialog>
#include <QString>

namespace Ui
{
class GoodDialog;
}

class GoodDialog : public QDialog
{
  Q_OBJECT

public:
  explicit GoodDialog(QWidget* parent = 0);
  ~GoodDialog();

  QString getGoodName();
  float getPositionX();
  float getPositionY();
  float getPositionZ();

  void setGoodName(QString);
  void setPositionX(float);
  void setPositionY(float);
  void setPositionZ(float);

private:
  Ui::GoodDialog* ui_;
};

#endif  // GOODDIALOG_H
