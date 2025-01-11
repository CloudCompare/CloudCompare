#pragma once

//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qCloudLayers                    #
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
//#                     COPYRIGHT: WigginsTech 2022                        #
//#                                                                        #
//##########################################################################

// local
#include <ui_ccCloudLayersDlg.h>
#include "ccAsprsModel.h"
#include "ccCloudLayersHelper.h"

//CC
#include <ccOverlayDialog.h>

class ccPointCloud;
class ccMouseCircle;

class ccCloudLayersDlg : public ccOverlayDialog, public Ui::ccCloudLayersDlg
{
	Q_OBJECT

public:
	//! Default constructor
	explicit ccCloudLayersDlg(ccMainAppInterface* app, QWidget* parent = nullptr);

	//! Destructor
	virtual ~ccCloudLayersDlg();

	//! inherited from ccOverlayDialog
	bool start() override;
	void stop(bool accepted) override;
	
	bool setPointCloud(ccPointCloud* cloud);
	
private:
	void resetUI();
	void initTableView();

	void saveSettings();
	void loadSettings();

	bool eventFilter(QObject* obj, QEvent* event) override;
	void reject() override;

private Q_SLOTS:

	//! add new asprs item
	void addClicked();

	//! delete select(ed) asprs item(s)
	void deleteClicked();

	//! draw mouse circle
	void startClicked();

	//! stop drawing mouse cirlce
	void pauseClicked();

	//! apply changes and close dialog
	void applyClicked();

	//! restore changes and close dialog
	void closeClicked() { reject(); }
	
	void scalarFieldIndexChanged(int index);
	void inputClassIndexChanged(int index);
	void outputClassIndexChanged(int index);

	//! asprs model signals
	void codeChanged(ccAsprsModel::AsprsItem item, int oldCode);
	void colorChanged(ccAsprsModel::AsprsItem item);
	void classNameChanged(int row, QString newName);

	//! show color picker dialog
	void tableViewDoubleClicked(const QModelIndex &index);

	//! update input and output comboboxes
	void updateInputOutput();
	void swapInputOutput();

	void mouseMoved(int x, int y, Qt::MouseButtons buttons);

private:
	ccMainAppInterface* m_app;
	ccAsprsModel m_asprsModel;
	ccCloudLayersHelper* m_helper;
	ccMouseCircle* m_mouseCircle;
};

