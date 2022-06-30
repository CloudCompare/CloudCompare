#pragma once

//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qMeshBoolean                    #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "ui_meshBooleanDlg.h"

//! Dialog for qCork plugin
class ccMeshBooleanDialog : public QDialog, public Ui::MeshBooleanDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccMeshBooleanDialog(QWidget* parent = nullptr);

	//! Supported CSG operations
	enum CSG_OPERATION { UNION, INTERSECT, DIFF, SYM_DIFF };

	//! Set meshes names
	void setNames(const QString& A, const QString& B);

	//! Returns the selected operation
	CSG_OPERATION getSelectedOperation() const { return m_selectedOperation; }

	//! Returns whether mesh order has been swappped or not
	bool isSwapped() const { return m_isSwapped; }

protected Q_SLOTS:

	void unionSelected();
	void intersectSelected();
	void diffSelected();
	void symDiffSelected();
	void swap();

protected:

	CSG_OPERATION m_selectedOperation;
	bool m_isSwapped;
};
