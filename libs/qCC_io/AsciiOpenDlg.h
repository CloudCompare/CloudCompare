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

#ifndef CC_ASCII_OPEN_DIALOG_HEADER
#define CC_ASCII_OPEN_DIALOG_HEADER

#include <algorithm>

//Qt
#include <QString>
#include <QDialog>

//system
#include <vector>

enum CC_ASCII_OPEN_DLG_TYPES {	ASCII_OPEN_DLG_None			= 0,
								ASCII_OPEN_DLG_X			= 1,
								ASCII_OPEN_DLG_Y			= 2,
								ASCII_OPEN_DLG_Z			= 3,
								ASCII_OPEN_DLG_NX			= 4,
								ASCII_OPEN_DLG_NY			= 5,
								ASCII_OPEN_DLG_NZ			= 6,
								ASCII_OPEN_DLG_R			= 7,
								ASCII_OPEN_DLG_G			= 8,
								ASCII_OPEN_DLG_B			= 9,
								ASCII_OPEN_DLG_Grey			= 10,
								ASCII_OPEN_DLG_Scalar		= 11,
								ASCII_OPEN_DLG_RGB32i		= 12, //RGBA as a single 32 bits integer (PCL style)
								ASCII_OPEN_DLG_RGB32f		= 13, //RGBA as a single 32 bits float (PCL style)
};

//! Default ASCII header columns
class AsciiHeaderColumns
{
public:
	static QString X()  { return "X"; }
	static QString Y()  { return "Y"; }
	static QString Z()  { return "Z"; }
	static QString Nx() { return "Nx"; }
	static QString Ny() { return "Ny"; }
	static QString Nz() { return "Nz"; }
	static QString R()  { return "R"; }
	static QString G()  { return "G"; }
	static QString B()  { return "B"; }

	static QString Grey()   { return "Intensity"; }
	static QString Scalar() { return "SF"; }
	static QString RGB32i() { return "RGB32i"; }
	static QString RGB32f() { return "RGB32f"; }
};

const unsigned ASCII_OPEN_DLG_TYPES_NUMBER = 14;
const char ASCII_OPEN_DLG_TYPES_NAMES[ASCII_OPEN_DLG_TYPES_NUMBER][24] = {	"Ignore",
																			"coord. X",
																			"coord. Y",
																			"coord. Z",
																			"Nx",
																			"Ny",
																			"Nz",
																			"Red",
																			"Green",
																			"Blue",
																			"Grey",
																			"Scalar",
																			"RGBAi",
																			"RGBAf",
																			};

class QComboBox;
class Ui_AsciiOpenDialog;

//! Dialog for configuration of ASCII files opening sequence
class AsciiOpenDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	/** \param parent parent widget
	**/
	AsciiOpenDlg(QWidget* parent = 0);

	//! Default destructor
	virtual ~AsciiOpenDlg();

	//! Sets filename
	/** \param filename filename
	**/
	void setFilename(QString filename);

	//! ASCII open sequence item
	struct SequenceItem
	{
		CC_ASCII_OPEN_DLG_TYPES type;
		QString header;

		//! Default constructor
		SequenceItem()
			: type(ASCII_OPEN_DLG_None)
			, header()
		{}

		//! Constructor from parameters
		SequenceItem(CC_ASCII_OPEN_DLG_TYPES _type, const QString& _header)
			: type(_type)
			, header(_header)
		{}
	};

	//! ASCII open sequence
	typedef std::vector<SequenceItem> Sequence;

	//! Returns the whole "opening" sequence as set by the user
	Sequence getOpenSequence() const;

	//! Returns number of lines to skip
	unsigned getSkippedLinesCount() const { return m_skippedLines; }

	//! Returns user selected separator
	uchar getSeparator() const { return m_separator.cell(); }

	//! Returns roughly estimated average line size (in bytes)
	double getAverageLineSize() const { return m_averageLineSize; }

	//! Returns columns count per line
	unsigned getColumnsCount() const { return m_columnsCount; }

	//! Returns the max number of points per cloud
	unsigned getMaxCloudSize() const;

	//! Returns whether the current sequence is 'safe'
	/** A safe sequence is safe if it matches the header (if any)
		or if the file has less than 6 columns.
	**/
	bool safeSequence() const;

	//! Checks the "opening" sequence as set by the user
	/** \return validity (+ error message if not)
	**/
	static bool CheckOpenSequence(const Sequence& sequence, QString& errorMessage);

public slots:
	//! Slot called when separator changes
	void onSeparatorChange(const QString& separator);
	//! Forces the table to update itself
	void updateTable();
	//! Sets the number of lines to skip
	void setSkippedLines(int linesCount);

protected slots:
	void testBeforeAccept();
	void columnsTypeHasChanged(int index);
	void shortcutButtonPressed();
	void checkSelectedColumnsValidity();

protected:

	//! Tries to guess the best separator automagically
	void autoFindBestSeparator();

	//associated UI
	Ui_AsciiOpenDialog* m_ui;

	unsigned m_skippedLines;
	QChar m_separator;
	double m_averageLineSize;
	QString m_filename;
	QString m_headerLine;
	//! Identifies columns with numbers only [mandatory]
	std::vector<bool> m_columnsValidty;

	//QComboBox* m_columnsType;
	unsigned m_columnsCount;

	// Resizes dialog width to fit all displayed table columns
	void resizeWidthToFitTableColumns();
};

#endif //CC_ASCII_OPEN_DIALOG_HEADER
