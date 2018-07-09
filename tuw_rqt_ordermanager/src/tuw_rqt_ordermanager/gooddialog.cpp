#include "tuw_rqt_ordermanager/gooddialog.h"
#include "tuw_rqt_ordermanager/ui_gooddialog.h"

GoodDialog::GoodDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GoodDialog)
{
    ui->setupUi(this);

    connect(ui->buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    connect(ui->buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
}

GoodDialog::~GoodDialog()
{
    delete ui;
}

QString GoodDialog::getGoodName()
{
    return this->ui->le_good_name->text();
}
float GoodDialog::getPositionX()
{
    return this->ui->le_pos_x->text().toFloat();
}
float GoodDialog::getPositionY()
{
    return this->ui->le_pos_y->text().toFloat();
}
float GoodDialog::getPositionZ()
{
    return this->ui->le_pos_z->text().toFloat();
}

void GoodDialog::setGoodName(QString name)
{
    this->ui->le_good_name->setText(name);
}
void GoodDialog::setPositionX(float x)
{
    this->ui->le_pos_x->setText(QString::number(x));
}
void GoodDialog::setPositionY(float y)
{
    this->ui->le_pos_y->setText(QString::number(y));
}
void GoodDialog::setPositionZ(float z)
{
    this->ui->le_pos_z->setText(QString::number(z));
}
