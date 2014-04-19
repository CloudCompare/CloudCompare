#include <float.h>

#include "ccShiftAndScaleCloudDlg.h"

//Qt
#include <QPushButton>

//semi-persistent settings
struct LastInfo
{
	bool hasInfo;
	CCVector3d shift;
	double scale;

	LastInfo()
		: hasInfo(false)
		, shift(0,0,0)
		, scale(0)
	{}
};
static LastInfo s_lastInfo;

ccShiftAndScaleCloudDlg::ccShiftAndScaleCloudDlg(const double* P,
												 double diagonal,
												 QWidget* parent/*=0*/)
	: QDialog(parent)
	, m_applyAll(false)
{
	setupUi(this);
	showWarning(false);
	if (P)
		firstPointCoordsLabel->setText(QString("(%1;%2;%3)").arg(P[0],0,'f',2).arg(P[1],0,'f',2).arg(P[2],0,'f',2));
	else
		firstPointFrame->hide();

	if (diagonal > 0.0)
		cloudDiagonalLabel->setText(QString::number(diagonal,'f',2));
	else
		scaleInfoFrame->setVisible(false);

	shiftX->setRange(-DBL_MAX,DBL_MAX);
	shiftY->setRange(-DBL_MAX,DBL_MAX);
	shiftZ->setRange(-DBL_MAX,DBL_MAX);

	//we can only load the previously stored info... if we have it!
	useLastToolButton->setVisible(s_lastInfo.hasInfo);

	connect(useLastToolButton,	SIGNAL(clicked()),					this,	SLOT(useLastInfo()));
	connect(buttonBox,			SIGNAL(clicked(QAbstractButton*)),	this,	SLOT(onClick(QAbstractButton*)));
}

void ccShiftAndScaleCloudDlg::showScaleItems(bool state)
{
	scaleInfoFrame->setVisible(state);
	scaleFrame->setVisible(state);
}

void ccShiftAndScaleCloudDlg::setShift(const CCVector3d& shift)
{
	shiftX->setValue(shift.x);
	shiftY->setValue(shift.y);
	shiftZ->setValue(shift.z);
}

CCVector3d ccShiftAndScaleCloudDlg::getShift() const
{
	return CCVector3d( shiftX->value(), shiftY->value(), shiftZ->value() );
}

void ccShiftAndScaleCloudDlg::setScale(double scale)
{
	scaleSpinBox->setValue(scale);
}

double ccShiftAndScaleCloudDlg::getScale() const
{
	return scaleSpinBox->value();
}

void ccShiftAndScaleCloudDlg::showWarning(bool state)
{
	warningLabel->setVisible(state);
}

void ccShiftAndScaleCloudDlg::onClick(QAbstractButton* button)
{
	bool saveInfo = false;
	m_applyAll = false;

	if (button == buttonBox->button(QDialogButtonBox::Yes))
	{
		saveInfo = true;
	}
	else if (button == buttonBox->button(QDialogButtonBox::YesToAll))
	{
		saveInfo = true;
		m_applyAll = true;
	}

	if (saveInfo)
	{
		s_lastInfo.hasInfo = true;
		s_lastInfo.shift = getShift();
		s_lastInfo.scale = getScale();
	}
}

void ccShiftAndScaleCloudDlg::useLastInfo()
{
	if (s_lastInfo.hasInfo)
	{
		setShift(s_lastInfo.shift);
		if (scaleSpinBox->isVisible())
			setScale(s_lastInfo.scale);
	}
}
