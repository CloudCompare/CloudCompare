//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qVoxFall                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 3 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                 COPYRIGHT: THE UNIVERSITY OF NEWCASTLE                 #
//#                                                                        #
//##########################################################################

#ifndef Q_VOXFALL_DIALOG_HEADER
#define Q_VOXFALL_DIALOG_HEADER

#include <ui_qVoxFallDialog.h>

//Qt
#include <QSettings>

class ccMainAppInterface;
class ccMesh;

//! VOXFALL plugin's main dialog
class qVoxFallDialog : public QDialog, public Ui::VoxFallDialog
{
	Q_OBJECT

public:

	//! Default constructor
	qVoxFallDialog(ccMesh* mesh1, ccMesh* mesh2, ccMainAppInterface* app);

	//! Returns mesh #1
	ccMesh* getMesh1() const { return m_mesh1; }
	//! Returns mesh #2
	ccMesh* getMesh2() const { return m_mesh2; }

	//! Returns voxel size
	double getVoxelSize() const;
	//! Returns slope azimuth
	double getAzimuth() const;
	//! Returns whether the blocks will be exported as meshes
	bool getExportMeshesActivation() const;
	//! Labels the blocks as loss or gain clusters
	bool getLossGainActivation() const;

	//! Returns the max number of threads to use
	int getMaxThreadCount() const;

	void qVoxFallDialog::loadParamsFromPersistentSettings();
	void qVoxFallDialog::loadParamsFrom(const QSettings& settings);
	void qVoxFallDialog::saveParamsToPersistentSettings();
	void qVoxFallDialog::saveParamsTo(QSettings& settings);

protected:

	void swapMeshes();
	void setMesh1Visibility(bool);
	void setMesh2Visibility(bool);

protected: //methods

	//! Sets meshes
	void setMeshes(ccMesh* mesh1, ccMesh* mesh2);

protected: //members

	ccMainAppInterface* m_app;

	ccMesh* m_mesh1;
	ccMesh* m_mesh2;
};

#endif //Q_VOXFALL_DIALOG_HEADER
