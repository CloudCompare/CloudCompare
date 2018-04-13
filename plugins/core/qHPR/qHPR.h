//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qHPR                        #
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

#ifndef Q_HPR_PLUGIN_HEADER
#define Q_HPR_PLUGIN_HEADER

#include "ccStdPluginInterface.h"

//CCLib
#include <ReferenceCloud.h>

//! Wrapper to the "Hidden Point Removal" algorithm for approximating points visibility in an N dimensional point cloud, as seen from a given viewpoint
/** "Direct Visibility of Point Sets", Sagi Katz, Ayellet Tal, and Ronen Basri. 
	SIGGRAPH 2007
	http://www.mathworks.com/matlabcentral/fileexchange/16581-hidden-point-removal
**/
class qHPR : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qHPR" FILE "info.json")

public:

	//! Default constructor
	explicit qHPR(QObject* parent = nullptr);
	
	virtual ~qHPR() = default;

	//inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual QList<QAction *> getActions() override;

protected slots:

	//! Slot called when associated ation is triggered
	void doAction();

protected:

	//! Katz et al. algorithm
	CCLib::ReferenceCloud* removeHiddenPoints(CCLib::GenericIndexedCloudPersist* theCloud, const CCVector3d& viewPoint, double fParam);

	//! Associated action
	QAction* m_action;
};

#endif
