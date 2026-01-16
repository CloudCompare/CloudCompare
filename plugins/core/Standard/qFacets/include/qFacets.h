#pragma once

// ##########################################################################
// #                                                                        #
// #                     CLOUDCOMPARE PLUGIN: qFacets                       #
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
// #                      COPYRIGHT: Thomas Dewez, BRGM                     #
// #                                                                        #
// ##########################################################################

// Local
#include "cellsFusionDlg.h"

// Qt
#include <QObject>

// qCC
#include "ccStdPluginInterface.h"

// CCCoreLib
#include <AutoSegmentationTools.h>
#include <DistanceComputationTools.h>
#include <ReferenceCloud.h>
#include <ccPointCloud.h>

// System
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
	Q_INTERFACES(ccPluginInterface ccStdPluginInterface)

	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qFacets" FILE "../info.json")

  public:
	//! Default constructor
	qFacets(QObject* parent = nullptr);

	//! Destructor
	~qFacets() override = default;

	// inherited from ccStdPluginInterface
	void            onNewSelection(const ccHObject::Container& selectedEntities) override;
	QList<QAction*> getActions() override;
	void            registerCommands(ccCommandLineInterface* cmd) override;

	struct FacetsParams
	{
		bool                      extractFacets;
		CellsFusionDlg::Algorithm algo;
		// ALGO_KD_TREE only
		double kdTreeFusionMaxAngleDeg;
		double kdTreeFusionMaxRelativeDistance;
		// ALGO_FAST_MARCHING only
		unsigned octreeLevel;
		bool     useRetroProjectionError;
		// both ALGO_KD_TREE and ALGO_FAST_MARCHING
		double                                              errorMaxPerFacet;
		double                                              maxEdgeLength;
		unsigned                                            minPointsPerFacet;
		CCCoreLib::DistanceComputationTools::ERROR_MEASURES errorMeasure;

		bool   classifyFacetsByAngle;
		double classifAngleStep;
		double classifMaxDist;

		bool    exportFacets; // Export to shape file
		QString shapeFilename;
		bool    useNativeOrientation; // also used for csv if coordsInCsv=ture
		bool    useGlobalOrientation; // also used for csv if coordsInCsv=ture
		bool    useCustomOrientation; // also used for csv if coordsInCsv=ture
		float   nX;                   // also used for csv if coordsInCsv=ture
		float   nY;                   // also used for csv if coordsInCsv=ture
		float   nZ;                   // also used for csv if coordsInCsv=ture

		bool    exportFacetsInfo; // Export to csv
		QString csvFilename;
		bool    coordsInCsv;

		FacetsParams()
		    : extractFacets(false)
		    , algo(CellsFusionDlg::Algorithm::ALGO_KD_TREE)
		    , kdTreeFusionMaxAngleDeg(20.0f)
		    , kdTreeFusionMaxRelativeDistance(1.0f)
		    , octreeLevel(8)
		    , useRetroProjectionError(false)
		    , errorMaxPerFacet(0.2f)
		    , minPointsPerFacet(10)
		    , maxEdgeLength(1.0f)
		    , errorMeasure(CCCoreLib::DistanceComputationTools::MAX_DIST_99_PERCENT)
		    , classifyFacetsByAngle(false)
		    , classifAngleStep(30.0f)
		    , classifMaxDist(1.0f)
		    , exportFacets(false)
		    , shapeFilename("facets.shp")
		    , useNativeOrientation(true)
		    , useGlobalOrientation(false)
		    , useCustomOrientation(false)
		    , nX(0.0f)
		    , nY(0.0f)
		    , nZ(1.0f)
		    , exportFacetsInfo(false)
		    , csvFilename("facets.csv")
		    , coordsInCsv(false)
		{
		}
	};

	//! Set of facets (pointers)
	typedef std::unordered_set<ccFacet*> FacetSet;

	static bool ExecuteExportFacetsInfo(const FacetSet& facets,
	                                    const QString   filename,
	                                    bool            coordsInCSV          = false,
	                                    bool            useNativeOrientation = true,
	                                    bool            useGlobalOrientation = false,
	                                    bool            useCustomOrientation = false,
	                                    double          nX                   = 0.0f,
	                                    double          nY                   = 0.0f,
	                                    double          nZ                   = 1.0f,
	                                    bool            silent               = false);

	static bool ExecuteExportFacets(const FacetSet& facets,
	                                const QString   filename,
	                                bool            useNativeOrientation = true,
	                                bool            useGlobalOrientation = false,
	                                bool            useCustomOrientation = false,
	                                double          nX                   = 0.0f,
	                                double          nY                   = 0.0f,
	                                double          nZ                   = 1.0f,
	                                bool            silent               = false);

	static QString PolylineCoordsToWKT_POLYGONZ(const ccPolyline* polyline,
	                                            unsigned int      precision = 3);

	static ccGLMatrix CalcOriRotMat(const FacetSet& facets,
	                                bool            useNativeOrientation = true,
	                                bool            useGlobalOrientation = false,
	                                bool            useCustomOrientation = false,
	                                double          nX                   = 0.0f,
	                                double          nY                   = 0.0f,
	                                double          nZ                   = 1.0f);

  protected:
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
	static ccHObject* CreateFacets(ccPointCloud*                       cloud,
	                               CCCoreLib::ReferenceCloudContainer& components,
	                               unsigned                            minPointsPerComponent,
	                               double                              maxEdgeLength,
	                               bool                                randomColors,
	                               bool&                               error,
	                               CCCoreLib::GenericProgressCallback* progress = nullptr);

  public:
	//! Core logic for facet extraction from a point cloud
	static ccHObject* ExecuteFacetExtraction(ccPointCloud*                       pc,
	                                         const FacetsParams&                 params,
	                                         bool&                               errorDuringFacetCreation,
	                                         CCCoreLib::GenericProgressCallback* progress = nullptr);

  protected:
	//! Returns all the facets in the current selection
	void getFacetsInCurrentSelection(FacetSet& facets) const;

	//! Classifies facets by orientation
	void classifyFacetsByAngle(ccHObject* group,
	                           double     angleStep_deg,
	                           double     maxDist);

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

	//! Sterogram dialog
	StereogramDialog* m_stereogramDialog;
};
