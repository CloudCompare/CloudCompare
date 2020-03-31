//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qFacets                       #
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
//#                      COPYRIGHT: Thomas Dewez, BRGM                     #
//#                                                                        #
//##########################################################################

#ifndef QFACET_PLUGIN_HEADER
#define QFACET_PLUGIN_HEADER

//Local
#include "cellsFusionDlg.h"

//Qt
#include <QObject>

//qCC
#include "ccStdPluginInterface.h"

//CCLib
#include <AutoSegmentationTools.h>
#include <ReferenceCloud.h>

//System
#include <unordered_set>

class QAction;
class ccHObject;
class ccPointCloud;
class ccPolyline;
class ccFacet;

class StereogramDialog;

//! Facet detection plugin (BRGM)
/** BRGM: BUREAU DE RECHERCHES GEOLOGIQUES ET MINIERES - http://www.brgm.fr/
**/
class qFacets : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qFacets" FILE "info.json")

public:

	//! Default constructor
	qFacets(QObject* parent = nullptr);
	
	//! Destructor
	virtual ~qFacets() = default;

	//inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual QList<QAction *> getActions() override;

protected slots:

	//! Fuses the cells of a kd-tree to produces planar facets
	void fuseKdTreeCells();

	//! Uses Fast Marching to detect planar facets
	void extractFacetsWithFM();

	//! Exports facets (as shapefiles)
	void exportFacets();

	//! Exports statistics on a set of facets
	void exportFacetsInfo();

	//! Classifies facets by orientation
	void classifyFacetsByAngle();

	//! Displays the selected entity stereogram
	void showStereogram();

protected:

	//! Uses the given algorithm to detect planar facets
	void extractFacets(CellsFusionDlg::Algorithm algo);

	//! Creates facets from components
	ccHObject* createFacets(ccPointCloud* cloud,
							CCLib::ReferenceCloudContainer& components,
							unsigned minPointsPerComponent,
							double maxEdgeLength,
							bool randomColors,
							bool& error);

	//! Set of facets (pointers)
	typedef std::unordered_set<ccFacet*> FacetSet;

	//! Returns all the facets in the current selection
	void getFacetsInCurrentSelection(FacetSet& facets) const;

	//! Classifies facets by orientation
	void classifyFacetsByAngle(	ccHObject* group,
								double angleStep_deg,
								double maxDist);

	//! Associated action
	QAction* m_doFuseKdTreeCells;
	//! Associated action
	QAction* m_fastMarchingExtraction;
	//! Associated action
	QAction* m_doExportFacets;
	//! Associated action
	QAction* m_doExportFacetsInfo;
	//! Associated action
	QAction* m_doClassifyFacetsByAngle;
	//! Associated action
	QAction* m_doShowStereogram;
	
	StereogramDialog* m_stereogramDialog;
};

#endif //QFACET_PLUGIN_HEADER
