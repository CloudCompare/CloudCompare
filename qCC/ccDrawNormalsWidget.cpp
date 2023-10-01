#include "ccDrawNormalsWidget.h"
#include "ui_ccDrawNormalsWidget.h"

#include <QSettings>

ccDrawNormalsWidget::ccDrawNormalsWidget(ccPointCloud* cloud, QWidget* parent)
	: QWidget(parent)
	, ui(new Ui::ccDrawNormalsWidget)
	, m_cloud(cloud)
{
	ui->setupUi(this);

	ui->label_cloudName->setText(m_cloud->getName());

	buttonGroup = new QButtonGroup();
	buttonGroup->addButton(this->ui->pushButtonYellow);
	buttonGroup->addButton(this->ui->pushButtonRed);
	buttonGroup->addButton(this->ui->pushButtonGreen);
	buttonGroup->addButton(this->ui->pushButtonBlue);
	buttonGroup->addButton(this->ui->pushButtonBlack);

	connect(this->ui->doubleSpinBox_normalLength, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ccDrawNormalsWidget::normalLengthValueChanged);
	connect(this->ui->pushButtonYellow,		&QPushButton::clicked, this, &ccDrawNormalsWidget::setNormalColor);
	connect(this->ui->pushButtonRed,		&QPushButton::clicked, this, &ccDrawNormalsWidget::setNormalColor);
	connect(this->ui->pushButtonGreen,		&QPushButton::clicked, this, &ccDrawNormalsWidget::setNormalColor);
	connect(this->ui->pushButtonBlue,		&QPushButton::clicked, this, &ccDrawNormalsWidget::setNormalColor);
	connect(this->ui->pushButtonBlack,		&QPushButton::clicked, this, &ccDrawNormalsWidget::setNormalColor);

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
	QSettings settings;
	settings.beginGroup("PointCloudNormals");

	double normalLength = settings.value("normalLength", 1.).toDouble();
	normalColor color = static_cast<normalColor>(settings.value("normalColor", YELLOW).toUInt());

	ui->doubleSpinBox_normalLength->setValue(normalLength);

	switch (color)
	{
	case RED:
		this->ui->pushButtonRed->setChecked(true);
		break;
	case GREEN:
		this->ui->pushButtonGreen->setChecked(true);
		break;
	case BLUE:
		this->ui->pushButtonBlue->setChecked(true);
		break;
	case BLACK:
		this->ui->pushButtonBlack->setChecked(true);
		break;
	default:
		this->ui->pushButtonYellow->setChecked(true);
		break;
	}

	settings.endGroup();
}

void ccDrawNormalsWidget::writeSettings()
{
	QSettings settings;
	settings.beginGroup("PointCloudNormals");

	settings.setValue("normalLength", ui->doubleSpinBox_normalLength->value());

	if (this->ui->pushButtonRed->isChecked())
		settings.setValue("normalColor", RED);
	else if (this->ui->pushButtonGreen->isChecked())
		settings.setValue("normalColor", GREEN);
	else if (this->ui->pushButtonBlue->isChecked())
		settings.setValue("normalColor", BLUE);
	else if (this->ui->pushButtonBlack->isChecked())
		settings.setValue("normalColor", BLACK);
	else
		settings.setValue("normalColor", YELLOW);

	settings.endGroup();
}

void ccDrawNormalsWidget::normalLengthValueChanged(double value)
{
	m_cloud->setNormalLength(value);
	m_cloud->redrawDisplay();
}

void ccDrawNormalsWidget::setNormalColor()
{
	if (this->ui->pushButtonRed->isChecked())
		m_cloud->setNormalLineColor(ccColor::red);
	else if (this->ui->pushButtonGreen->isChecked())
		m_cloud->setNormalLineColor(ccColor::green);
	else if (this->ui->pushButtonBlue->isChecked())
		m_cloud->setNormalLineColor(ccColor::blue);
	else if (this->ui->pushButtonBlack->isChecked())
		m_cloud->setNormalLineColor(ccColor::black);
	else
		m_cloud->setNormalLineColor(ccColor::yellow);

	if (m_cloud->getDisplay())
		m_cloud->redrawDisplay();
}
