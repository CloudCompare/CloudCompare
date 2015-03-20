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
#include <math.h>

//number of valid operations
static const unsigned s_opCount = 18;
//operation names
static const char s_opNames[s_opCount][12] = {"+", "-", "*", "/", "sqrt", "pow2", "pow3", "exp", "log", "log10", "cos", "sin", "tan", "acos", "asin", "atan", "int", "inverse" };

//semi persitent
static int s_previouslySelectedOperationIndex = 1;

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
	operationComboBox->setCurrentIndex(s_previouslySelectedOperationIndex);
}

void ccScalarFieldArithmeticsDlg::onCurrentIndexChanged(int index)
{
	sf2ComboBox->setEnabled(index <= DIVIDE); //only the 4 first operations are between two SFs
	s_previouslySelectedOperationIndex = index;
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
	int opIndex = operationComboBox->currentIndex();
	if (opIndex < s_opCount)
		return static_cast<ccScalarFieldArithmeticsDlg::Operation>(opIndex);

	assert(false);
	return INVALID;
}

QString ccScalarFieldArithmeticsDlg::getOperationName(QString sf1, QString sf2/*=QString()*/) const
{
	Operation op = getOperation();
	return GetOperationName(op,sf1,sf2);
}

ccScalarFieldArithmeticsDlg::Operation ccScalarFieldArithmeticsDlg::GetOperationByName(QString name)
{
	name = name.toUpper();

	//test all known names...
	for (unsigned i=0; i<s_opCount; ++i)
		if (name == QString(s_opNames[i]).toUpper())
			return static_cast<ccScalarFieldArithmeticsDlg::Operation>(i);

	return INVALID;
}

QString ccScalarFieldArithmeticsDlg::GetOperationName(Operation op, QString sf1, QString sf2/*=QString()*/)
{
	switch (op)
	{
	case PLUS:
		return QString("%1 + %2").arg(sf1).arg(sf2);
	case MINUS:
		return QString("%1 - %2").arg(sf1).arg(sf2);
	case MULTIPLY:
		return QString("%1 * %2").arg(sf1).arg(sf2);
	case DIVIDE:
		return QString("%1 / %2").arg(sf1).arg(sf2);
	default:
		if (op != INVALID)
			return QString("%1(%2)").arg(s_opNames[op]).arg(sf1);
		else
			assert(false);
		break;
	}

	return QString();
}

bool ccScalarFieldArithmeticsDlg::apply(ccPointCloud* cloud)
{
	Operation op = getOperation();
	int sf1Idx = getSF1Index();
	int sf2Idx = getSF2Index();

	return Apply(cloud,op,sf1Idx,sf2Idx,this);
}

bool ccScalarFieldArithmeticsDlg::Apply(ccPointCloud* cloud, Operation op, int sf1Idx, int sf2Idx/*=-1*/, QWidget* parent/*=0*/)
{
	assert(cloud);

	if (!cloud || !cloud->hasScalarFields())
	{
		ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] No input cloud, or cloud has no SF?!");
		assert(false);
		return false;
	}

	if (op == INVALID)
	{
		ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Invalid/unhandled operation");
		assert(false);
		return false;
	}

	unsigned sfCount = cloud->getNumberOfScalarFields();
	if (std::max(sf1Idx,sf2Idx) >= static_cast<int>(sfCount))
	{
		ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Cloud has not enough SFs to apply the selected operation!");
		assert(false);
		return false;
	}
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
		sf1Name = QString("(SF#%1)").arg(sf1Idx);
		sf2Name = QString("(SF#%1)").arg(sf2Idx);
	}
	QString sfName = GetOperationName(op,sf1Name,sf2Name);

	int sfIdx = cloud->getScalarFieldIndexByName(qPrintable(sfName));
	if (sfIdx >= 0)
	{
		if (sfIdx == sf1Idx || sfIdx == sf2Idx)
		{
			ccLog::Warning(QString("[ccScalarFieldArithmeticsDlg::apply] Resulting scalar field would have the same name as one of the operand (%1)! Rename it first...").arg(sfName));
			return false;
		}
		if (parent && QMessageBox::warning(	parent,
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
			case INT:
				val = static_cast<ScalarType>(static_cast<int>(val1)); //integer part ('round' doesn't seem to be available on MSVC?!)
				break;
			case INVERSE:
				val = fabs(val1) < ZERO_TOLERANCE ? NAN_VALUE : static_cast<ScalarType>(1.0/val1);
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
