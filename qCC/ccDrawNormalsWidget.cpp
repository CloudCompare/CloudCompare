#include "ccDrawNormalsWidget.h"
#include "ui_ccDrawNormalsWidget.h"

#include <QSettings>

ccDrawNormalsWidget::ccDrawNormalsWidget(ccPointCloud *cloud, QWidget *parent) :
	QWidget(parent),
	ui(new Ui::ccDrawNormalsWidget),
	m_cloud(cloud)
{
	ui->setupUi(this);

	ui->label_cloudName->setText(m_cloud->getName());

	buttonGroup = new QButtonGroup();
	buttonGroup->addButton(this->ui->pushButton_yellow);
	buttonGroup->addButton(this->ui->pushButton_red);
	buttonGroup->addButton(this->ui->pushButton_green);
	buttonGroup->addButton(this->ui->pushButton_blue);
	buttonGroup->addButton(this->ui->pushButton_black);

	connect(this->ui->doubleSpinBox_normalLength, SIGNAL(valueChanged(double)), this, SLOT(normalLengthValueChanged(double)));
	connect(this->ui->pushButton_yellow,	&QPushButton::clicked, this, &ccDrawNormalsWidget::setNormalColor);
	connect(this->ui->pushButton_red,		&QPushButton::clicked, this, &ccDrawNormalsWidget::setNormalColor);
	connect(this->ui->pushButton_green,		&QPushButton::clicked, this, &ccDrawNormalsWidget::setNormalColor);
	connect(this->ui->pushButton_blue,		&QPushButton::clicked, this, &ccDrawNormalsWidget::setNormalColor);
	connect(this->ui->pushButton_black,		&QPushButton::clicked, this, &ccDrawNormalsWidget::setNormalColor);

	readSettings();

	normalLengthValueChanged(ui->doubleSpinBox_normalLength->value());
	setNormalColor();

	setWindowFlag(Qt::WindowStaysOnTopHint);
	setWindowFlag(Qt::Window);
	setWindowTitle("Draw normals");
	show();
}

ccDrawNormalsWidget::~ccDrawNormalsWidget()
{
	writeSettings();
	delete ui;
}

void ccDrawNormalsWidget::readSettings()
{
	QSettings settings("OSUR/CloudCompare", "drawNormals");

	double normalLength = settings.value("normalLength", 1.).toDouble();
	normalColor color = static_cast<normalColor>(settings.value("normalColor", YELLOW).toUInt());

	ui->doubleSpinBox_normalLength->setValue(normalLength);

	switch (color) {
	case RED:
		this->ui->pushButton_red->setChecked(true);
		break;
	case GREEN:
		this->ui->pushButton_red->setChecked(true);
		break;
	case BLUE:
		this->ui->pushButton_red->setChecked(true);
		break;
	case BLACK:
		this->ui->pushButton_red->setChecked(true);
		break;
	default:
		this->ui->pushButton_yellow->setChecked(true);
		break;
	}
}

void ccDrawNormalsWidget::writeSettings()
{
	QSettings settings("OSUR/CloudCompare", "drawNormals");

	settings.setValue("normalLength", ui->doubleSpinBox_normalLength->value());

	if (this->ui->pushButton_red->isChecked())
		settings.setValue("normalColor", RED);
	else if (this->ui->pushButton_green->isChecked())
		settings.setValue("normalColor", GREEN);
	else if (this->ui->pushButton_blue->isChecked())
		settings.setValue("normalColor", BLUE);
	else if (this->ui->pushButton_black->isChecked())
		settings.setValue("normalColor", BLACK);
	else
		settings.setValue("normalColor", YELLOW);
}

void ccDrawNormalsWidget::normalLengthValueChanged(double value)
{
	m_cloud->setNormalLength(value);
}

void ccDrawNormalsWidget::setNormalColor()
{
	if (this->ui->pushButton_red->isChecked())
		m_cloud->setNormalColor(QColorConstants::Red);
	else if (this->ui->pushButton_green->isChecked())
		m_cloud->setNormalColor(QColorConstants::Green);
	else if (this->ui->pushButton_blue->isChecked())
		m_cloud->setNormalColor(QColorConstants::Blue);
	else if (this->ui->pushButton_black->isChecked())
		m_cloud->setNormalColor(QColorConstants::Black);
	else
		m_cloud->setNormalColor(QColorConstants::Yellow);
}
