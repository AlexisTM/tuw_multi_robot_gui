#include "tuw_rqt_ordermanager/robotdialog.h"
#include "tuw_rqt_ordermanager/ui_robotdialog.h"

RobotDialog::RobotDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RobotDialog)
{
    ui->setupUi(this);

    connect(ui->buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    connect(ui->buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
}

RobotDialog::~RobotDialog()
{
    delete ui;
}

QString RobotDialog::getRobotName()
{
    return this->ui->le_robot_name->text();
}
float RobotDialog::getPositionX()
{
    return this->ui->le_pos_x->text().toFloat();
}
float RobotDialog::getPositionY()
{
    return this->ui->le_pos_y->text().toFloat();
}
float RobotDialog::getPositionZ()
{
    return this->ui->le_pos_z->text().toFloat();
}

void RobotDialog::setRobotName(QString name)
{
    this->ui->le_robot_name->setText(name);
}
void RobotDialog::setPositionX(float x)
{
    this->ui->le_pos_x->setText(QString::number(x));
}
void RobotDialog::setPositionY(float y)
{
    this->ui->le_pos_y->setText(QString::number(y));
}
void RobotDialog::setPositionZ(float z)
{
    this->ui->le_pos_z->setText(QString::number(z));
}
