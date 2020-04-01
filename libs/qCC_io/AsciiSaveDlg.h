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

#ifndef CC_ASCII_SAVE_DIALOG_HEADER
#define CC_ASCII_SAVE_DIALOG_HEADER

//local
#include "qCC_io.h"

//Qt
#include <QDialog>

class Ui_AsciiSaveDialog;

//! Dialog for configuration of ASCII files saving sequence
class QCC_IO_LIB_API AsciiSaveDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit AsciiSaveDlg(QWidget* parent = 0);

	//! Destructor
	virtual ~AsciiSaveDlg();

	//! Returns whether columns names should be be saved in header
	bool saveColumnsNamesHeader() const;
	//! Sets whether columns names should be be saved in header
	void enableSaveColumnsNamesHeader(bool state);

	//! Returns whether the number of points should be saved in header
	bool savePointCountHeader() const;
	//! Sets whether the number of points should be saved in header
	void enableSavePointCountHeader(bool state);

	//! Returns separator
	unsigned char getSeparator() const;

	//! Set separator (index)
	/** 0 = space
		1 = semicolon
		2 = comma
		3 = tab
	**/
	void setSeparatorIndex(int index);

	//! Returns coordinates precision
	int coordsPrecision() const;
	//! Sets coordinates precision
	void setCoordsPrecision(int prec);

	//! Returns SF precision
	int sfPrecision() const;
	//! Sets SF precision
	void setSfPrecision(int prec);

	//! Returns whether SF(s) and color should be swapped
	bool swapColorAndSF() const;
	//! Sets whether SF(s) and color should be swapped
	void enableSwapColorAndSF(bool state);

	//! Sets whether this dialog should appear or not by default
	void setAutoShow(bool state) { m_autoShow = state; }
	//! Returns whether this dialog automatically appears or not
	bool autoShow() const { return m_autoShow; }

	//! Sets whether to save colors as float values (instead of unsigned bytes)
	void setSaveFloatColors(bool state);
	//! Returns whether to save colors as float values (instead of unsigned bytes)
	bool saveFloatColors() const;

	//! Sets whether to save the alpha (transparency) channel
	void setSaveAlphaChannel(bool state);
	//! Returns whether to save the alpha (transparency) channel
	bool saveAlphaChannel() const;

protected slots:

	//! Saves dialog state to persistent settings
	void acceptAndSaveSettings();

protected:

	//! Associated UI
	Ui_AsciiSaveDialog* m_ui;

	//! Inits dialog state from persistent settings
	void initFromPersistentSettings();

	//! Whether this dialog should be automatically shown or not
	bool m_autoShow;

};

#endif //CC_ASCII_SAVE_DIALOG_HEADER

