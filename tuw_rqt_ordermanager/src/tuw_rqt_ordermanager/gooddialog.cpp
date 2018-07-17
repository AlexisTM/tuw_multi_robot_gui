#include "tuw_rqt_ordermanager/gooddialog.h"
#include "tuw_rqt_ordermanager/ui_gooddialog.h"

GoodDialog::GoodDialog(QWidget* parent) : QDialog(parent), ui_(new Ui::GoodDialog)
{
  ui_->setupUi(this);

  connect(ui_->buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
  connect(ui_->buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
}

GoodDialog::~GoodDialog()
{
  delete ui_;
}

QString GoodDialog::getGoodName()
{
  return this->ui_->le_good_name->text();
}
float GoodDialog::getPositionX()
{
  return this->ui_->le_pos_x->text().toFloat();
}
float GoodDialog::getPositionY()
{
  return this->ui_->le_pos_y->text().toFloat();
}
float GoodDialog::getPositionZ()
{
  return this->ui_->le_pos_z->text().toFloat();
}

void GoodDialog::setGoodName(QString name)
{
  this->ui_->le_good_name->setText(name);
}
void GoodDialog::setPositionX(float x)
{
  this->ui_->le_pos_x->setText(QString::number(x));
}
void GoodDialog::setPositionY(float y)
{
  this->ui_->le_pos_y->setText(QString::number(y));
}
void GoodDialog::setPositionZ(float z)
{
  this->ui_->le_pos_z->setText(QString::number(z));
}
