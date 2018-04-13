//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qHoughNormals                   #
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

#ifndef QHOUGH_NORMALS_PLUGIN_HEADER
#define QHOUGH_NORMALS_PLUGIN_HEADER

#include "ccStdPluginInterface.h"

//! Wrapper to the 'normals_Hough' library (https://github.com/aboulch/normals_Hough)
/** "Deep Learning for Robust Normal Estimation in Unstructured Point Clouds"
	by Alexandre Boulch and Renaud Marlet, Symposium of Geometry Processing 2016, Computer Graphics Forum
**/
class qHoughNormals : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qHoughNormals" FILE "info.json")

public:

	//! Default constructor
	explicit qHoughNormals(QObject* parent = nullptr);

	virtual ~qHoughNormals() = default;
	
	//inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual QList<QAction *> getActions() override;

protected slots:

	//! Slot called when associated ation is triggered
	void doAction();

protected:

	//! Associated action
	QAction* m_action;
};

#endif //QHOUGH_NORMALS_PLUGIN_HEADER
