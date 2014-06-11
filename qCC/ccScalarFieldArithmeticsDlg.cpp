//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccScalarFieldArithmeticsDlg.h"

//Qt
#include <QPushButton>
#include <QMessageBox>

//qCC_db
#include <ccPointCloud.h>
#include <ccScalarField.h>

//system
#include <assert.h>
#ifdef _MSC_VER
#include <windows.h>
#endif

ccScalarFieldArithmeticsDlg::ccScalarFieldArithmeticsDlg(	ccPointCloud* cloud,
															QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::SFArithmeticsDlg()
{
	assert(cloud);

	setupUi(this);
	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

	unsigned sfCount = cloud ? cloud->getNumberOfScalarFields() : 0;
	if (sfCount < 1)
	{
		sf1ComboBox->setEnabled(false);
		sf2ComboBox->setEnabled(false);
		buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
	}
	else
	{
		QStringList sfLabels;
		for (unsigned i=0; i<sfCount; ++i)
			sfLabels << QString(cloud->getScalarFieldName(i));

		sf1ComboBox->addItems(sfLabels);
		sf1ComboBox->setCurrentIndex(0);
		sf2ComboBox->addItems(sfLabels);
		sf2ComboBox->setCurrentIndex(std::min<unsigned>(1,sfCount-1));
	}

	connect(operationComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onCurrentIndexChanged(int)));
}

void ccScalarFieldArithmeticsDlg::onCurrentIndexChanged(int index)
{
	sf2ComboBox->setEnabled(index <= DIVIDE); //only the 4 first operations are between two SFs
}

int ccScalarFieldArithmeticsDlg::getSF1Index()
{
	return sf1ComboBox->currentIndex();
}

int ccScalarFieldArithmeticsDlg::getSF2Index()
{
	return sf2ComboBox->currentIndex();
}

ccScalarFieldArithmeticsDlg::Operation ccScalarFieldArithmeticsDlg::getOperation() const
{
	switch (operationComboBox->currentIndex())
	{
	case 0:
		return PLUS;
	case 1:
		return MINUS;
	case 2:
		return MULTIPLY;
	case 3:
		return DIVIDE;
	case 4:
		return SQRT;
	case 5:
		return POW2;
	case 6:
		return POW3;
	case 7:
		return EXP;
	case 8:
		return LOG;
	case 9:
		return LOG10;
	case 10:
		return COS;
	case 11:
		return SIN;
	case 12:
		return TAN;
	case 13:
		return ACOS;
	case 14:
		return ASIN;
	case 15:
		return ATAN;
	default:
		assert(false);
		break;
	}

	return INVALID;
}

QString ccScalarFieldArithmeticsDlg::getOperationName(QString sf1, QString sf2/*=QString()*/) const
{
	switch (getOperation())
	{
	case PLUS:
		return QString("%1 + %2").arg(sf1).arg(sf2);
	case MINUS:
		return QString("%1 - %2").arg(sf1).arg(sf2);
	case MULTIPLY:
		return QString("%1 * %2").arg(sf1).arg(sf2);
	case DIVIDE:
		return QString("%1 / %2").arg(sf1).arg(sf2);
	case SQRT:
		return QString("sqrt(%1)").arg(sf1);
	case POW2:
		return QString("pow2(%1)").arg(sf1);
	case POW3:
		return QString("pow3(%1)").arg(sf1);
	case EXP:
		return QString("exp(%1)").arg(sf1);
	case LOG:
		return QString("log(%1)").arg(sf1);
	case LOG10:
		return QString("log10(%1)").arg(sf1);
	case COS:
		return QString("cos(%1)").arg(sf1);
	case SIN:
		return QString("sin(%1)").arg(sf1);
	case TAN:
		return QString("tan(%1)").arg(sf1);
	case ACOS:
		return QString("acos(%1)").arg(sf1);
	case ASIN:
		return QString("asin(%1)").arg(sf1);
	case ATAN:
		return QString("atan(%1)").arg(sf1);
	default:
		assert(false);
		break;
	}

	return QString();
}

bool ccScalarFieldArithmeticsDlg::apply(ccPointCloud* cloud)
{
	if (!cloud || !cloud->hasScalarFields())
	{
		ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] No input cloud, or cloud has no SF?!");
		assert(false);
		return false;
	}

	Operation op = getOperation();
	if (op == INVALID)
	{
		ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Invalid/unhandled operation selected");
		assert(false);
		return false;
	}

	unsigned sfCount = cloud->getNumberOfScalarFields();
	if (op <= DIVIDE && sfCount < 2)
	{
		ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Cloud has not enough SFs to apply the selected operation!");
		assert(false);
		return false;
	}

	int sf1Idx = getSF1Index();
	int sf2Idx = (op <= DIVIDE ? getSF2Index() : -1);
	CCLib::ScalarField* sf1 = cloud->getScalarField(sf1Idx);
	CCLib::ScalarField* sf2 = (sf2Idx >= 0 ? cloud->getScalarField(sf2Idx) : 0);

	if (!sf1 || (op <= DIVIDE && !sf2))
	{
		ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Invalid SF selection!");
		assert(false);
		return false;
	}

	//generate new sf name based on the operation
	QString sf1Name(sf1->getName());
	QString sf2Name;
	if (sf2)
	{
		//for operations involving two SFs, we don't expand the SF names (as the resulting SF name would be tool long!)
		sf1Name = QString("(SF#%1").arg(sf1Idx);
		sf2Name = QString("(SF#%1").arg(sf2Idx);
	}
	QString sfName = getOperationName(sf1Name,sf2Name);

	int sfIdx = cloud->getScalarFieldIndexByName(qPrintable(sfName));
	if (sfIdx >= 0)
	{
		if (sfIdx == sf1Idx || sfIdx == sf2Idx)
		{
			ccLog::Warning(QString("[ccScalarFieldArithmeticsDlg::apply] Resulting scalar field would have the same name as one of the operand (%1)! Rename it first...").arg(sfName));
			return false;
		}
		if (QMessageBox::warning(	this,
									"Same scalar field name",
									"Resulting scalar field already exists! Overwrite it?",
									QMessageBox::Ok | QMessageBox::Cancel,
									QMessageBox::Ok ) != QMessageBox::Ok)
		{
			return false;
		}

		cloud->deleteScalarField(sfIdx);
	}

	sfIdx = cloud->addScalarField(qPrintable(sfName));
	if (sfIdx < 0)
	{
		ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Failed to create destination SF! (not enough memory?)");
		return false;
	}
	CCLib::ScalarField* sfDest = cloud->getScalarField(sfIdx);

	unsigned valCount = sf1->currentSize();
	assert(!sf2 || valCount == sf2->currentSize());

	//resize destination SF
	if (!sfDest->resize(valCount))
	{
		ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Not enough memory!");
		cloud->deleteScalarField(sfIdx);
		sfDest = 0;
		return false;
	}
	assert(valCount == sfDest->currentSize());

	for (unsigned i=0; i<valCount; ++i)
	{
		ScalarType val = NAN_VALUE;

		//we must handle 'invalid' values
		const ScalarType& val1 = sf1->getValue(i);
		if (ccScalarField::ValidValue(val1))
		{
			switch (op)
			{
			case PLUS:
				{
					const ScalarType& val2 = sf2->getValue(i);
					if (ccScalarField::ValidValue(val2))
						val = val1 + val2;
				}
				break;
			case MINUS:
				{
					const ScalarType& val2 = sf2->getValue(i);
					if (ccScalarField::ValidValue(val2))
						val = val1 - val2;
				}
				break;
			case MULTIPLY:
				{
					const ScalarType& val2 = sf2->getValue(i);
					if (ccScalarField::ValidValue(val2))
						val = val1 * val2;
				}
				break;
			case DIVIDE:
				{
					const ScalarType& val2 = sf2->getValue(i);
					if (ccScalarField::ValidValue(val2))
						val = val1 / val2;
				}
				break;
			case SQRT:
				if (val1 >= 0)
					val = sqrt(val1);
				break;
			case POW2:
				val = val1*val1;
				break;
			case POW3:
				val = val1*val1*val1;
				break;
			case EXP:
				val = exp(val1);
				break;
			case LOG:
				if (val1 >= 0)
					val = log(val1);
				break;
			case LOG10:
				if (val1 >= 0)
					val = log10(val1);
				break;
			case COS:
				val = cos(val1);
				break;
			case SIN:
				val = sin(val1);
				break;
			case TAN:
				val = tan(val1);
				break;
			case ACOS:
				if (val1 >= -1 && val1 <= 1.0)
					val = acos(val1);
				break;
			case ASIN:
				if (val1 >= -1 && val1 <= 1.0)
					val = asin(val1);
				break;
			case ATAN:
				val = atan(val1);
				break;
			default:
				assert(false);
				break;
			}
		}

		sfDest->setValue(i,val);
	}

	sfDest->computeMinAndMax();
	cloud->setCurrentDisplayedScalarField(sfIdx);

	return true;
}
