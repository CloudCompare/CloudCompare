#include "ccShiftAndScaleCloudDlg.h"

//Qt
#include <QPushButton>

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
		cloudDiagonalLabel->setVisible(false);

	connect(buttonBox, SIGNAL(clicked(QAbstractButton*)), this, SLOT(onClick(QAbstractButton*)));
}

void ccShiftAndScaleCloudDlg::showScaleItems(bool state)
{
	scaleInfoFrame->setVisible(state);
	scaleFrame->setVisible(state);
}

void ccShiftAndScaleCloudDlg::setShift(double x, double y, double z)
{
	shiftX->setValue(x);
	shiftY->setValue(y);
	shiftZ->setValue(z);
}

void ccShiftAndScaleCloudDlg::getShift(double& x, double& y, double& z) const
{
	x = shiftX->value();
	y = shiftY->value();
	z = shiftZ->value();
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
	m_applyAll = (button == buttonBox->button(QDialogButtonBox::YesToAll));
}
