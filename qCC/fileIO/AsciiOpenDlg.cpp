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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2224                                                              $
//$LastChangedDate:: 2012-07-25 19:13:23 +0200 (mer., 25 juil. 2012)       $
//**************************************************************************
//
#include <QMessageBox>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QLineEdit>
#include <QSpinBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QToolButton>
#include <QPushButton>

#include <stdio.h>
#include <assert.h>

#include <CCMiscTools.h>

#include "AsciiOpenDlg.h"
#include "FileIOFilter.h"

AsciiOpenDlg::AsciiOpenDlg(QString filename, QWidget* parent)
	: QDialog(parent)
	, Ui::AsciiOpenDialog()
	, m_skippedLines(0)
	, m_separator(' ')
	, m_averageLineSize(-1.0)
	, m_filename(filename)
	, m_invalidColumns(true)
	, m_columnsCount(0)
{
    setupUi(this);

    //spinBoxSkipLines->setValue(0);

    connect(buttonBox, SIGNAL(accepted()), this, SLOT(testBeforeAccept()));
    connect(lineEditSeparator, SIGNAL(textChanged(const QString &)), this, SLOT(updateTable(const QString &)));
    connect(spinBoxSkipLines, SIGNAL(valueChanged(int)), this, SLOT(setSkippedLines(int)));

    //shortcut buttons
    connect(toolButtonShortcutESP, SIGNAL(clicked()), this, SLOT(shortcutButtonPressed()));
    connect(toolButtonShortcutTAB, SIGNAL(clicked()), this, SLOT(shortcutButtonPressed()));
    connect(toolButtonShortcutComma, SIGNAL(clicked()), this, SLOT(shortcutButtonPressed()));
    connect(toolButtonShortcutDotcomma, SIGNAL(clicked()), this, SLOT(shortcutButtonPressed()));

    lineEditFileName->setText(m_filename);

	QList<QChar> separators;
	separators << QChar(' ');
	separators << QChar('\t');
	separators << QChar(',');
	separators << QChar(';');

	//We try all default separators...
	for (int i=0;i<separators.size();++i)
	{
		lineEditSeparator->setText(separators[i]); //this calls 'updateTable'
		//...until we find one that gives us at least 3 colums
		if (getColumnsCount()>2)
			break;
	}
}

AsciiOpenDlg::~AsciiOpenDlg()
{
}

void AsciiOpenDlg::setSkippedLines(int linesCount)
{
    if (linesCount<0)
        return;

    m_skippedLines = (unsigned)linesCount;

    updateTable(m_separator);
}

static const unsigned MAX_COLUMNS = 256;				//maximum number of columns that can be handled
static const unsigned LINES_READ_FOR_STATS = 200;		//number of lines read for stats
static const unsigned DISPLAYED_LINES = 20;				//number of displayed lines

void AsciiOpenDlg::updateTable(const QString &separator)
{
	tableWidget->setEnabled(false);
	extractSFNamesFrom1stLineCheckBox->setEnabled(false);
	m_headerLine.clear();

	if (m_filename.isEmpty())
	{
		tableWidget->clear();
        return;
	}

    if (separator.length()<1)
    {
        asciiCodeLabel->setText("Enter a valid character!");
        buttonBox->setEnabled(false);
		tableWidget->clear();
		m_invalidColumns = true;
        return;
    }

	//we open the file in ASCII mode
	FILE* pFile = fopen(qPrintable(m_filename),"rt");
	if (!pFile)
	{
		tableWidget->clear();
		m_invalidColumns = true;
        return;
	}

    //buffer
	char line[MAX_ASCII_FILE_LINE_LENGTH];      //last read line

    //we skip first lines (if needed)
	unsigned i;
    for (i=0;i<m_skippedLines;++i)
	{
        if (fgets(line, MAX_ASCII_FILE_LINE_LENGTH, pFile))
		{
			//we keep track of the first line
			if (i==0)
				m_headerLine = QString(line);
		}
		else
		{
			fclose(pFile);
			tableWidget->clear();
			m_invalidColumns = true;
            return;
		}
	}

    //new separator
    m_separator = separator[0];
    asciiCodeLabel->setText(QString("(ASCII code: %1)").arg(m_separator.unicode()));
	//if the old setup has less than 3 columns, we forget it
	if (m_columnsCount<3)
	{
		tableWidget->clear();
		m_columnsCount=0;
	}
    tableWidget->setRowCount(DISPLAYED_LINES+1);    //+1 for first line shifting

	unsigned lineCount = 0;			//number of lines read
	unsigned totalChars = 0;        //total read characters (for stats)
	unsigned columnsCount = 0;		//max columns count per line

	std::vector<bool> valueIsNumber;	//identifies columns with numbers only [mandatory]
	std::vector<bool> valueIsBelowOne;	//identifies columns with values between -1 and 1 only
	std::vector<bool> valueIsInteger;	//identifies columns with integer values only
	std::vector<bool> valueIsBelow255;	//identifies columns with integer values between 0 and 255 only

	while ((lineCount<LINES_READ_FOR_STATS) && fgets(line, MAX_ASCII_FILE_LINE_LENGTH, pFile))
	{
        //we convert char buffer to a QString object
		QString str(line);

        //we recognize "//" as comment
		if (line[0]!='/' || line[1]!='/')
		{
			QStringList parts = str.trimmed().split(m_separator,QString::SkipEmptyParts);
            unsigned partsCount = (unsigned)parts.size();
            if (partsCount>MAX_COLUMNS)
                partsCount=MAX_COLUMNS;

            if (lineCount<DISPLAYED_LINES)
            {
                //do we need to add one or several new columns?
                if (partsCount>columnsCount)
                {
					//we also extend vectors
					for (i=columnsCount;i<partsCount;++i)
					{
						valueIsNumber.push_back(true);
						valueIsBelowOne.push_back(true);
						valueIsBelow255.push_back(true);
						valueIsInteger.push_back(true);
					}

					tableWidget->setColumnCount(partsCount);
                    columnsCount=partsCount;
                }

                //we fill row with extracted parts
                for (i=0;i<partsCount;++i)
                {
                    QTableWidgetItem *newItem = new QTableWidgetItem(parts[i]);

					//test values
					bool isANumber = false;
					double value = parts[i].toDouble(&isANumber);
					if (!isANumber)
					{
						valueIsNumber[i]	= false;
						valueIsBelowOne[i]	= false;
						valueIsInteger[i]	= false;
						valueIsBelow255[i]	= false;
						newItem->setBackground(QBrush(QColor(255,160,160)));
					}
					else
					{
						double intPart, fracPart;
						fracPart = modf(value,&intPart);

						valueIsBelowOne[i]	= valueIsBelowOne[i] && (fabs(value)<=1.0);
						valueIsInteger[i]	= valueIsInteger[i] && (fracPart == 0.0);
						valueIsBelow255[i]	= valueIsBelow255[i] && (valueIsInteger[i] && (intPart >= 0.0 && value<=255.0));
					}

					tableWidget->setItem(lineCount+1, i, newItem); //+1 for first line shifting
                }
            }

            totalChars += (str.size()+1); //+1 for return char at eol

            ++lineCount;
		}
	}

	fclose(pFile);
	pFile=0;

	if (lineCount==0 || columnsCount==0)
	{
		m_averageLineSize = -1.0;
		tableWidget->clear();
		m_invalidColumns = true;
        return;
	}

	//average line size
	m_averageLineSize = double(totalChars)/double(lineCount);

    //we add a type selector for each column
	QStringList propsText;
	for (i=0; i<ASCII_OPEN_DLG_TYPES_NUMBER; i++)
        propsText << QString(ASCII_OPEN_DLG_TYPES_NAMES[i]);

	//remove unnecessary columns
	while (columnsCount<m_columnsCount)
		tableWidget->removeColumn(--m_columnsCount);
	for (i=lineCount+1;i<=DISPLAYED_LINES;++i)
		tableWidget->removeRow(i);

    int columnWidth = (tableWidget->width()*9) / (columnsCount*10);
    columnWidth = ccMax(columnWidth,80);

	//Icons
	const QIcon xIcon(QString::fromUtf8(":/CC/Types/images/types/x_coordinate.png"));
	const QIcon yIcon(QString::fromUtf8(":/CC/Types/images/types/y_coordinate.png"));
	const QIcon zIcon(QString::fromUtf8(":/CC/Types/images/types/z_coordinate.png"));
	const QIcon NormIcon(QString::fromUtf8(":/CC/Types/images/types/normal.png"));
	const QIcon RGBIcon(QString::fromUtf8(":/CC/Types/images/types/rgb_color.png"));
	const QIcon GreyIcon(QString::fromUtf8(":/CC/Types/images/types/gray_color.png"));
	const QIcon ScalarIcon(QString::fromUtf8(":/CC/Types/images/types/scalar_field.png"));
	const QIcon PositiveScalarIcon(QString::fromUtf8(":/CC/Types/images/types/positive_scalar_field.png"));

	unsigned assignedXYZ = 0;
	unsigned assignedNorm = 0;
	unsigned assignedRGB = 0;
	for (i=0;i<columnsCount;i++)
	{
		QComboBox* columnHeader = static_cast<QComboBox*>(tableWidget->cellWidget(0,i));
		QComboBox* _columnHeader = columnHeader;
		if (!columnHeader)
		{
			columnHeader = new QComboBox();
			columnHeader->addItems(propsText);
			columnHeader->setMaxVisibleItems(ASCII_OPEN_DLG_TYPES_NUMBER);
			columnHeader->setCurrentIndex(0);
			columnHeader->setItemIcon(ASCII_OPEN_DLG_X,xIcon);
			columnHeader->setItemIcon(ASCII_OPEN_DLG_Y,yIcon);
			columnHeader->setItemIcon(ASCII_OPEN_DLG_Z,zIcon);
			columnHeader->setItemIcon(ASCII_OPEN_DLG_NX,NormIcon);
			columnHeader->setItemIcon(ASCII_OPEN_DLG_NY,NormIcon);
			columnHeader->setItemIcon(ASCII_OPEN_DLG_NZ,NormIcon);
			columnHeader->setItemIcon(ASCII_OPEN_DLG_R,RGBIcon);
			columnHeader->setItemIcon(ASCII_OPEN_DLG_G,RGBIcon);
			columnHeader->setItemIcon(ASCII_OPEN_DLG_B,RGBIcon);
			columnHeader->setItemIcon(ASCII_OPEN_DLG_Grey,GreyIcon);
			columnHeader->setItemIcon(ASCII_OPEN_DLG_Scalar,ScalarIcon);
			columnHeader->setItemIcon(ASCII_OPEN_DLG_Positive_Scalar,PositiveScalarIcon);

			connect(columnHeader, SIGNAL(currentIndexChanged(int)), this, SLOT(columnsTypeHasChanged(int)));
		}

		if (valueIsNumber[i])
		{
			//first time? let's try to assign each column a type
			if ((m_invalidColumns || m_columnsCount==0) && columnsCount>1)
			{
				columnHeader->blockSignals(true);
				//by default, we assume that the first columns are always X,Y and Z
				if (assignedXYZ<3)
				{
					//in rare cases, the first column is an index
					if (assignedXYZ==0 && valueIsInteger[i] && (i+1<columnsCount) && !valueIsInteger[i+1])
					{
						//we skip it
					}
					else
					{
						++assignedXYZ;
						columnHeader->setCurrentIndex(assignedXYZ);
					}
				}
				else
				{
					//looks like RGB?
					if (valueIsBelow255[i] && assignedRGB<3 && (i+2-assignedRGB < columnsCount)
						&& (assignedRGB > 0 || (valueIsBelow255[i+1] && valueIsBelow255[i+2]))) //make sure that next values are also ok!
					{
						columnHeader->setCurrentIndex(ASCII_OPEN_DLG_R+assignedRGB);
						++assignedRGB;
					}
					else if (valueIsBelowOne[i] && assignedNorm<3 && (i+2-assignedNorm < columnsCount)
						&& (assignedNorm > 0 || (valueIsBelowOne[i+1] && valueIsBelowOne[i+2]))) //make sure that next values are also ok!
					{
						columnHeader->setCurrentIndex(ASCII_OPEN_DLG_NX+assignedNorm);
						++assignedNorm;
					}
					else
					{
						//maybe it's a scalar?
						columnHeader->setCurrentIndex(ASCII_OPEN_DLG_Scalar);
					}
				}
				columnHeader->blockSignals(false);
			}
		}

		if (!_columnHeader)
			tableWidget->setCellWidget(0,i,columnHeader);
		tableWidget->setColumnWidth(i,columnWidth);
	}
	m_columnsCount=columnsCount;

	//can we consider the first ignored line as a header?
	if (!m_headerLine.isEmpty())
	{
		QStringList parts = m_headerLine.trimmed().split(m_separator,QString::SkipEmptyParts);
		extractSFNamesFrom1stLineCheckBox->setEnabled((unsigned)parts.size() == m_columnsCount);
	}

	//check for invalid columns
	m_invalidColumns = false;
	for (i=0;i<m_columnsCount;i++)
	{
		m_invalidColumns |= (!valueIsNumber[i]);
		//tableWidget->cellWidget(0,i)->setEnabled(valueIsNumber[i]);
	}

	tableWidget->setEnabled(true);
	buttonBox->setEnabled(true);
	buttonBox->button(QDialogButtonBox::Ok)->setEnabled(!m_invalidColumns);
}

bool AsciiOpenDlg::CheckOpenSequence(const AsciiOpenDlg::Sequence& sequence, QString& errorMessage)
{
	//two requirements:
	//- at least 2 coordinates must be defined
	//- appart from SFs, only one column assignment per property
	unsigned i,counters[ASCII_OPEN_DLG_TYPES_NUMBER];
	memset(counters,0,sizeof(unsigned)*ASCII_OPEN_DLG_TYPES_NUMBER);

	for (i=0;i<sequence.size();i++)
		++counters[sequence[i].type];

	for (i=1;i<ASCII_OPEN_DLG_Scalar;i++)
		if (counters[i]>1)
		{
			errorMessage = QString("'%1' defined at least twice!").arg(ASCII_OPEN_DLG_TYPES_NAMES[i]);
			return false;
		}

	unsigned char coordIsDefined[3]={counters[ASCII_OPEN_DLG_X]>0
									, counters[ASCII_OPEN_DLG_Y]>0
									, counters[ASCII_OPEN_DLG_Z]>0};

	if (coordIsDefined[0] + coordIsDefined[1] + coordIsDefined[2]<2)
	{
		errorMessage = "At least 2 vertex coordinates should be defined!";
		return false;
	}

	return true;
}

void AsciiOpenDlg::testBeforeAccept()
{
	QString errorMessage;
	if (!CheckOpenSequence(getOpenSequence(),errorMessage))
	{
        QMessageBox::warning(0, "Error", errorMessage);
	}
	else
	{
		accept();
	}
}

AsciiOpenDlg::Sequence AsciiOpenDlg::getOpenSequence()
{
    Sequence seq;

    if (m_columnsCount>0)
    {
		//shall we extract headerParts?
		QStringList headerParts;
		if (!m_headerLine.isEmpty()
			&& extractSFNamesFrom1stLineCheckBox->isEnabled()
			&& extractSFNamesFrom1stLineCheckBox->isChecked())
		{
			headerParts = m_headerLine.trimmed().split(m_separator,QString::SkipEmptyParts);
		}

        seq.reserve(m_columnsCount-1);
        for (unsigned i=0;i<m_columnsCount;i++)
        {
			const QComboBox* combo = static_cast<QComboBox*>(tableWidget->cellWidget(0,i));
			assert(combo);
			seq.push_back(SequenceItem((CC_ASCII_OPEN_DLG_TYPES)combo->currentIndex(),headerParts.size()>(int)i ? headerParts[i] : QString()));
        }
    }

    return seq;
}

void AsciiOpenDlg::columnsTypeHasChanged(int index)
{
    if (!m_columnsCount)
        return;

    //we get the signal sender
    QObject* obj = sender();
    if (!obj)
        return;

    //it should be a QComboBox (could we test this?)
    QComboBox* changedCombo = static_cast<QComboBox*>(obj);
	assert(changedCombo);

    //now we look which column's combobox it is
	for (unsigned i=0;i<m_columnsCount;i++)
	{
		QComboBox* combo = static_cast<QComboBox*>(tableWidget->cellWidget(0,i));
	    //we found the right element
        if (changedCombo == combo)
        {
            if (index == int(ASCII_OPEN_DLG_X) ||
                index == int(ASCII_OPEN_DLG_NX) ||
                index == int(ASCII_OPEN_DLG_R))
            {
				//Auto select the next columns type
                if (i+2<m_columnsCount)
                {
					QComboBox* nextCombo = static_cast<QComboBox*>(tableWidget->cellWidget(0,i+1));
					QComboBox* nextNextCombo = static_cast<QComboBox*>(tableWidget->cellWidget(0,i+2));
                    //if the two next columns have no assigned type, we set them auto.
                    if (nextCombo->currentIndex()==int(ASCII_OPEN_DLG_None)
                        && nextNextCombo->currentIndex()==int(ASCII_OPEN_DLG_None))
                    {
						nextCombo->blockSignals(true);
						nextNextCombo->blockSignals(true);

                        if (index == int(ASCII_OPEN_DLG_X))
                        {
                            nextCombo->setCurrentIndex(ASCII_OPEN_DLG_Y);
                            nextNextCombo->setCurrentIndex(ASCII_OPEN_DLG_Z);
                        }
                        else if (index == int(ASCII_OPEN_DLG_NX))
                        {
                            nextCombo->setCurrentIndex(ASCII_OPEN_DLG_NY);
                            nextNextCombo->setCurrentIndex(ASCII_OPEN_DLG_NZ);
                        }
                        else if (index == int(ASCII_OPEN_DLG_R))
                        {
                            nextCombo->setCurrentIndex(ASCII_OPEN_DLG_G);
                            nextNextCombo->setCurrentIndex(ASCII_OPEN_DLG_B);
                        }
                    }

					nextCombo->blockSignals(false);
					nextNextCombo->blockSignals(false);
                }
            }
        }
		else if (index< ASCII_OPEN_DLG_Scalar) //check that the other combo as the same index (appart from SF)
		{
			if (combo->currentIndex() == index)
			{
				combo->blockSignals(true);
				combo->setCurrentIndex((int)ASCII_OPEN_DLG_None);
				combo->blockSignals(false);
			}
		}
	}
}

void AsciiOpenDlg::shortcutButtonPressed()
{
    if (!m_columnsCount)
        return;

    //we get the signal sender
    QObject* obj = sender();
    if (!obj)
        return;

    //it should be a QToolButton (could we test this?)
    QToolButton* shortcutButton = static_cast<QToolButton*>(obj);

    uchar newSeparator=0;
    if (shortcutButton==toolButtonShortcutESP)
        newSeparator=char(32);
    else if (shortcutButton==toolButtonShortcutTAB)
        newSeparator=char(9);
    else if (shortcutButton==toolButtonShortcutComma)
        newSeparator=char(44);
    else if (shortcutButton==toolButtonShortcutDotcomma)
        newSeparator=char(59);

    if (newSeparator>0 && getSeparator()!=newSeparator)
        lineEditSeparator->setText(QChar(newSeparator));
}

