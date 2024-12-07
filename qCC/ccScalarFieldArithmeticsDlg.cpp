//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccScalarFieldArithmeticsDlg.h"
#include "ui_sfArithmeticsDlg.h"

//Qt
#include <QMessageBox>
#include <QPushButton>

//qCC_db
#include <ccPointCloud.h>
#include <ccScalarField.h>

//system
#include <cassert>
#ifdef _MSC_VER
#include <windows.h>
#endif
#include <cmath>

//number of valid operations
constexpr unsigned s_opCount = 22;
//operation names
constexpr char s_opNames[s_opCount][8] {"add", "sub", "mult", "div", "min", "max", "sqrt", "pow2", "pow3", "exp", "log", "log10", "cos", "sin", "tan", "acos", "asin", "atan", "int", "inverse", "set", "abs" };

//semi persitent
static int s_previouslySelectedOperationIndex = 1;
static bool s_applyInPlace = false;
static double s_previousConstValue = 1.0;

ccScalarFieldArithmeticsDlg::ccScalarFieldArithmeticsDlg(	ccPointCloud* cloud,
															QWidget* parent/*=nullptr*/)
	: QDialog(parent, Qt::Tool)
	, m_ui( new Ui::SFArithmeticsDlg )
{
	assert(cloud);

	m_ui->setupUi(this);

	QStringList sfLabels;
	unsigned sfCount = cloud ? cloud->getNumberOfScalarFields() : 0;
	if (sfCount < 1)
	{
		m_ui->sf1ComboBox->setEnabled(false);
		m_ui->sf2ComboBox->setEnabled(false);
		m_ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
	}
	else
	{
		for (unsigned i = 0; i < sfCount; ++i)
		{
			sfLabels << QString::fromStdString(cloud->getScalarFieldName(i));
		}

		m_ui->sf1ComboBox->addItems(sfLabels);
		m_ui->sf1ComboBox->setCurrentIndex(0);

		sfLabels << "[Constant value]";
		m_ui->sf2ComboBox->addItems(sfLabels);
		m_ui->sf2ComboBox->setCurrentIndex(std::min<unsigned>(1, sfCount - 1));
	}

	//connect signals/slots
	connect(m_ui->operationComboBox,	qOverload<int>(&QComboBox::currentIndexChanged), this, &ccScalarFieldArithmeticsDlg::onOperationIndexChanged);
	connect(m_ui->sf2ComboBox,			qOverload<int>(&QComboBox::currentIndexChanged), this, &ccScalarFieldArithmeticsDlg::onSF2IndexChanged);
	
	m_ui->operationComboBox->setCurrentIndex(s_previouslySelectedOperationIndex);
	m_ui->constantDoubleSpinBox->setValue(s_previousConstValue);
	m_ui->updateSF1CheckBox->setChecked(s_applyInPlace);
}

ccScalarFieldArithmeticsDlg::~ccScalarFieldArithmeticsDlg()
{
	delete m_ui;
	m_ui = nullptr;
}

void ccScalarFieldArithmeticsDlg::onOperationIndexChanged(int index)
{
	if (index == Operation::SET)
	{
		//force the last element of the SF2 field (= always the 'constant' field)
		m_ui->sf2ComboBox->setCurrentIndex(m_ui->sf2ComboBox->count() - 1);
		m_ui->sf2ComboBox->setEnabled(false);
		m_ui->updateSF1CheckBox->setChecked(true);
		m_ui->updateSF1CheckBox->setEnabled(false);
	}
	else
	{
		m_ui->sf2ComboBox->setEnabled(index <= MAX); //only the 6 first operations are	applied with 2 SFs
		m_ui->updateSF1CheckBox->setEnabled(true);
	}
}

void ccScalarFieldArithmeticsDlg::onSF2IndexChanged(int index)
{
	//the last element is always the 'constant' field
	m_ui->constantDoubleSpinBox->setEnabled(m_ui->sf2ComboBox->currentIndex() + 1 == m_ui->sf2ComboBox->count());
}

int ccScalarFieldArithmeticsDlg::getSF1Index()
{
	return m_ui->sf1ComboBox->currentIndex();
}

int ccScalarFieldArithmeticsDlg::getSF2Index()
{
	return m_ui->sf2ComboBox->currentIndex();
}

ccScalarFieldArithmeticsDlg::Operation ccScalarFieldArithmeticsDlg::getOperation() const
{
	int opIndex = m_ui->operationComboBox->currentIndex();
	if (opIndex < s_opCount)
	{
		return static_cast<ccScalarFieldArithmeticsDlg::Operation>(opIndex);
	}
	else
	{
		assert(false);
		return INVALID;
	}
}

ccScalarFieldArithmeticsDlg::Operation ccScalarFieldArithmeticsDlg::GetOperationByName(const QString& name)
{
	auto lowerName = name.toLower();

	//test all known names...
	for (unsigned i = 0; i < s_opCount; ++i)
	{
		if (lowerName == QString( s_opNames[i] ))
		{
			return static_cast<ccScalarFieldArithmeticsDlg::Operation>(i);
		}
	}

	return INVALID;
}

QString ccScalarFieldArithmeticsDlg::GetOperationName(Operation op, const QString& sf1, const QString& sf2/*=QString()*/)
{
	switch (op)
	{
	case PLUS:
		return QString("%1 + %2").arg(sf1, sf2);
	case MINUS:
		return QString("%1 - %2").arg(sf1, sf2);
	case MULTIPLY:
		return QString("%1 * %2").arg(sf1, sf2);
	case DIVIDE:
		return QString("%1 / %2").arg(sf1, sf2);
	case MIN:
		return QString("min(%1, %2)").arg(sf1, sf2);
	case MAX:
		return QString("max(%1, %2)").arg(sf1, sf2);
	case SET:
		return sf1;
	default:
		if (op != INVALID)
			return QString("%1(%2)").arg(s_opNames[op], sf1);
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

	//save persistent parameters
	s_previouslySelectedOperationIndex = m_ui->operationComboBox->currentIndex();
	s_previousConstValue = m_ui->constantDoubleSpinBox->value();
	s_applyInPlace = m_ui->updateSF1CheckBox->isChecked();

	SF2 sf2Desc;
	sf2Desc.isConstantValue = m_ui->constantDoubleSpinBox->isEnabled() || (sf1Idx == Operation::SET);
	sf2Desc.constantValue = m_ui->constantDoubleSpinBox->value();
	sf2Desc.sfIndex = sf2Desc.isConstantValue ? -1 : sf2Idx;

	return Apply(cloud, op, sf1Idx, s_applyInPlace, &sf2Desc, this);
}

bool ccScalarFieldArithmeticsDlg::Apply(ccPointCloud* cloud,
										Operation op,
										int sf1Idx,
										bool inplace,
										SF2* sf2Desc/*=nullptr*/,
										QWidget* parent/*=nullptr*/)
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
	CCCoreLib::ScalarField* sf1 = nullptr;
	{
		if (sf1Idx >= static_cast<int>(sfCount))
		{
			ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Invalid SF1 index!");
			assert(false);
			return false;
		}
		sf1 = cloud->getScalarField(sf1Idx);
		assert(sf1);
	}
	double sf1PreviousOffset = sf1->getOffset(); // remember the offset, as we may have to change it BEFORE reading the values

	CCCoreLib::ScalarField* sf2 = nullptr;
	if (op <= MAX)
	{
		if (!sf2Desc || (!sf2Desc->isConstantValue && sf2Desc->sfIndex >= static_cast<int>(sfCount)))
		{
			ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Invalid SF2 index/descriptor!");
			assert(false);
			return false;
		}

		if (sf2Desc->isConstantValue)
		{
			if (op == DIVIDE && sf2Desc->constantValue == 0)
			{
				ccLog::Error("Invalid constant value (can't divide by zero)");
				return false;
			}
		}
		sf2 = (!sf2Desc->isConstantValue && sf2Desc->sfIndex >= 0 ? cloud->getScalarField(sf2Desc->sfIndex) : nullptr);
	}

	//output SF
	int sfIdx = -1;
	if (!inplace)
	{
		//generate new sf name based on the operation
		QString sf1Name = QString::fromStdString(sf1->getName());
		QString sf2Name;
		if (sf2)
		{
			sf2Name = QString::fromStdString(sf2->getName());
			QString sfName = GetOperationName(op,sf1Name,sf2Name);
			if (sfName.length() > 24)
			{
				assert(sf2Desc);
				//if the resulting SF name is too long, we use shortcuts instead
				sf1Name = QString("(SF#%1)").arg(sf1Idx);
				sf2Name = QString("(SF#%1)").arg(sf2Desc->sfIndex);
			}
		}
		else if (sf2Desc && sf2Desc->isConstantValue)
		{
			sf2Name = QString("%1").arg(sf2Desc->constantValue);
		}

		QString sfName = GetOperationName(op, sf1Name, sf2Name);

		sfIdx = cloud->getScalarFieldIndexByName(sfName.toStdString());
		if (sfIdx >= 0)
		{
			if (sfIdx == sf1Idx || (sf2Desc && sfIdx == sf2Desc->sfIndex))
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

		sfIdx = cloud->addScalarField(sfName.toStdString());
		if (sfIdx < 0)
		{
			ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Failed to create destination SF! (not enough memory?)");
			return false;
		}
	}
	else // we keep the original scalar field
	{
		// we might want to update the offset as best as we can
		switch (op)
		{
		case PLUS:
		{
			if (sf2Desc && sf2Desc->isConstantValue)
			{
				// shortcut
				sf1->setOffset(sf1->getOffset() + sf2Desc->constantValue);
				sf1->computeMinAndMax();
				cloud->setCurrentDisplayedScalarField(sf1Idx);

				ccLog::PrintVerbose(QString("SF %1 offset has been changed: from %2 to %3")
					.arg(QString::fromStdString(sf1->getName()))
					.arg(sf1PreviousOffset)
					.arg(sf1->getOffset()));
				return true;
			}
			else
			{
				sf1->setOffset(sf1->getOffset() + sf2->getOffset());
			}
			break;
		}
		case MINUS:
		{
			if (sf2Desc && sf2Desc->isConstantValue)
			{
				// shortcut
				sf1->setOffset(sf1->getOffset() - sf2Desc->constantValue);
				sf1->computeMinAndMax();
				cloud->setCurrentDisplayedScalarField(sf1Idx);

				ccLog::PrintVerbose(QString("SF %1 offset has been changed: from %2 to %3")
					.arg(QString::fromStdString(sf1->getName()))
					.arg(sf1PreviousOffset)
					.arg(sf1->getOffset()));
				return true;
			}
			else
			{
				sf1->setOffset(sf1->getOffset() - sf2->getOffset());
			}
			break;
		}
		case MULTIPLY:
		{
			if (sf2Desc && sf2Desc->isConstantValue)
			{
				sf1->setOffset(sf1->getOffset() * sf2Desc->constantValue); // it's safe was long as we use sf1PreviousOffset to read the values later
			}
			else
			{
				sf1->setOffset(sf1->getOffset() * sf2->getOffset()); // it's safe was long as we use sf1PreviousOffset to read the values later
			}
			break;
		}
		case DIVIDE:
		{
			if (sf2Desc && sf2Desc->isConstantValue)
			{
				if (sf2Desc->constantValue != 0)
				{
					sf1->setOffset(sf1->getOffset() / sf2Desc->constantValue); // it's safe was long as we use sf1PreviousOffset to read the values later
				}
			}
			else
			{
				// not obvious...
			}
			break;
		}
		case SQRT:
		{
			sf1->setOffset(sqrt(std::max(0.0, sf1->getOffset()))); // it's safe was long as we use sf1PreviousOffset to read the values later
			break;
		}
		case POW2:
		{
			sf1->setOffset(sf1->getOffset() * sf1->getOffset()); // it's safe was long as we use sf1PreviousOffset to read the values later
			break;
		}
		case POW3:
		{
			sf1->setOffset(sf1->getOffset() * sf1->getOffset() * sf1->getOffset()); // it's safe was long as we use sf1PreviousOffset to read the values later
			break;
		}
		case EXP:
		{
			// hard to predict anything...
			break;
		}
		case LOG:
		{
			if (sf1->getOffset() > 1.0)
			{
				sf1->setOffset(log(sf1->getOffset()));
			}
			else
			{
				sf1->setOffset(0.0);
			}
			break;
		}
		case LOG10:
		case COS:
		case SIN:
		case TAN:
		case ACOS:
		case ASIN:
		case ATAN:
		{
			sf1->setOffset(0.0); // it's safe was long as we use sf1PreviousOffset to read the values later
			break;
		}
		case INT:
		{
			// no need to change the offset
			break;
		}
		case INVERSE:
		{
			// if all values were close to 0, the result could be a very large number,
			// but we'll assume that in most of the cases they will be >> 0
			if (sf1->getMin() > 0.0 && sf1->getMax() < 1.0)
			{
				sf1->setOffset(1.0 / sf1->getMax());
			}
			else if (sf1->getMin() > -1.0 && sf1->getMax() < 0.0)
			{
				sf1->setOffset(1.0 / sf1->getMin());
			}
			else
			{
				sf1->setOffset(0.0);
			}
			break;
		}
		case SET:
		{
			if (sf2Desc && sf2Desc->isConstantValue)
			{
				sf1->setOffset(sf2Desc->constantValue); // it's safe was long as we use sf1PreviousOffset to read the values later
			}
			else
			{
				assert(false);
			}
			break;
		}
		case ABS:
		{
			sf1->setOffset(std::abs(sf1->getOffset())); // it's safe was long as we use sf1PreviousOffset to read the values later
			break;
		}
		default:
		{
			// don't modify the offset, hopefully this should be enough...
			break;
		}
		}

		if (sf1->getOffset() != sf1PreviousOffset)
		{
			ccLog::PrintVerbose(QString("SF %1 offset has been changed: from %2 to %3")
				.arg(QString::fromStdString(sf1->getName()))
				.arg(sf1PreviousOffset)
				.arg(sf1->getOffset()));
		}

		sfIdx = sf1Idx;
	}
	CCCoreLib::ScalarField* sfDest = cloud->getScalarField(sfIdx);
	assert(sfDest);

	unsigned valCount = sf1->currentSize();
	assert(!sf2 || valCount == sf2->currentSize());

	//resize destination SF
	if (!sfDest->resizeSafe(valCount))
	{
		ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Not enough memory!");
		cloud->deleteScalarField(sfIdx);
		sfDest = nullptr;
		return false;
	}
	assert(valCount == sfDest->currentSize());

	for (unsigned i = 0; i < valCount; ++i)
	{
		double val = std::numeric_limits<double>::quiet_NaN();

		//we must handle 'invalid' values
		double val1 = sf1PreviousOffset + sf1->getLocalValue(i);
		if (ccScalarField::ValidValue(val1))
		{
			switch (op)
			{
			case PLUS:
				{
					assert(sf2Desc);
					if (sf2Desc->isConstantValue)
					{
						val = val1 + sf2Desc->constantValue;
					}
					else
					{
						assert(sf2);
						ScalarType val2 = sf2->getValue(i);
						if (ccScalarField::ValidValue(val2))
						{
							val = val1 + val2;
						}
					}
				}
				break;
			case MINUS:
				{
					assert(sf2Desc);
					if (sf2Desc->isConstantValue)
					{
						val = val1 - sf2Desc->constantValue;
					}
					else
					{
						assert(sf2);
						ScalarType val2 = sf2->getValue(i);
						if (ccScalarField::ValidValue(val2))
						{
							val = val1 - val2;
						}
					}
				}
				break;
			case MULTIPLY:
				{
					assert(sf2Desc);
					if (sf2Desc->isConstantValue)
					{
						val = val1 * sf2Desc->constantValue;
					}
					else
					{
						assert(sf2);
						const ScalarType& val2 = sf2->getValue(i);
						if (ccScalarField::ValidValue(val2))
							val = val1 * val2;
					}
				}
				break;
			case DIVIDE:
				{
					assert(sf2Desc);
					if (sf2Desc->isConstantValue)
					{
						val = val1 / static_cast<ScalarType>(sf2Desc->constantValue);
					}
					else
					{
						assert(sf2);
						const ScalarType& val2 = sf2->getValue(i);
						if (ccScalarField::ValidValue(val2) && CCCoreLib::GreaterThanEpsilon(std::abs(val2) ) )
						{
							val = val1 / val2;
						}
					}
				}
				break;
			case SQRT:
				if (val1 >= 0)
				{
					val = std::sqrt(val1);
				}
				break;
			case POW2:
				val = val1 * val1;
				break;
			case POW3:
				val = val1 * val1 * val1;
				break;
			case EXP:
				val = std::exp(val1);
				break;
			case LOG:
				if (val1 >= 0)
				{
					val = std::log(val1);
				}
				break;
			case LOG10:
				if (val1 >= 0)
				{
					val = std::log10(val1);
				}
				break;
			case COS:
				val = std::cos(val1);
				break;
			case SIN:
				val = std::sin(val1);
				break;
			case TAN:
				val = std::tan(val1);
				break;
			case ACOS:
				if (val1 >= -1.0 && val1 <= 1.0)
				{
					val = std::acos(val1);
				}
				break;
			case ASIN:
				if (val1 >= -1.0 && val1 <= 1.0)
				{
					val = std::asin(val1);
				}
				break;
			case ATAN:
				val = std::atan(val1);
				break;
			case INT:
				val = std::round(val1); //integer part
				break;
			case INVERSE:
				val = CCCoreLib::LessThanEpsilon(std::abs(val1)) ? std::numeric_limits<double>::quiet_NaN() : (1.0 / val1);
				break;
			case SET:
				assert(sf2Desc);
				val = sf2Desc->constantValue;
				break;
			case ABS:
				val = std::abs(val1);
				break;
			case MIN:
			case MAX:
			{
				ScalarType val2 = 0;
				assert(sf2Desc);
				if (sf2Desc->isConstantValue)
				{
					val2 = sf2Desc->constantValue;
				}
				else
				{
					assert(sf2);
					val2 = sf2->getValue(i);
				}
				if (ccScalarField::ValidValue(val2))
				{
					val = op == MIN ? std::min(val1, val2) : std::max(val1, val2);
				}
			}
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
