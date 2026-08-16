#pragma once
// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

// qCC_db
#include <ccHObject.h>
#include <ccScalarField.h>

// Qt
#include <QDialog>

class ccScalarField;
class ccPointCloud;

namespace Ui
{
	class ScalarFieldsManagerDlg;
}

enum SFAttributes
{
	NAME,
	MINVAL,
	MAXVAL,
	MEAN,
	STD
};

//! Dialog to edit/create scalar fields
class ccScalarFieldsManagerDialog : public QDialog
{
	Q_OBJECT

  public:
	//! Default constructor
	ccScalarFieldsManagerDialog(const ccHObject::Container& selectedEntities,
	                            QWidget*                    parent = nullptr);

	//! Destructor
	~ccScalarFieldsManagerDialog() override;

	//! Sets active point cloud
	void setActivePointCloud(ccPointCloud* pc);
	void setSelectedEntities(const ccHObject::Container& entities);

  protected:
	void onEntityChanged(int index);

	void buildTable();
	void updateDisplay();

	void deleteSF();
	void renameSF(int sfIdx, const QString& newName);
	void addConstantSF();
	void showHistogram();
	void doArithmetic();

	void appendSFToTable(int sfIdx);

  protected:
	//! Active point cloud
	ccPointCloud*              m_pointCloud;
	unsigned                   m_sfCount;
	std::vector<ccPointCloud*> m_availableClouds;

	Ui::ScalarFieldsManagerDlg* m_ui;
};
