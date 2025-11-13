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

//CCCoreLib
#include <AutoSegmentationTools.h>
#include <ReferenceCloud.h>
#include <DistanceComputationTools.h>
#include <ccPointCloud.h>


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
	Q_INTERFACES( ccPluginInterface ccStdPluginInterface )

	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.qFacets" FILE "../info.json" )

public:

	//! Default constructor
	qFacets(QObject* parent = nullptr);

	//! Destructor
	virtual ~qFacets() = default;

	//inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual QList<QAction *> getActions() override;
	virtual void registerCommands(ccCommandLineInterface* cmd) override;
	
	


	struct FacetsParams
	{
		bool EXTRACT_FACETS;
		CellsFusionDlg::Algorithm ALGO;		
		//ALGO_KD_TREE
		double KD_TREE_FUSION_MAX_ANGLE_DEG;
		double KD_TREE_FUSION_MAX_RELATIVE_DISTANCE;
		//ALGO_FAST_MARCHING
		unsigned OCTREE_LEVEL;
		bool USE_RETRO_PROJECTION_ERROR;
		//both ALGO_KD_TREE ALGO_FAST_MARCHING and 
		double ERROR_MAX_PER_FACET;
		
		double MAX_EDGE_LENGTH;
		unsigned MIN_POINTS_PER_FACET; 
		//int ERROR_MEASURE_TYPE;
		CCCoreLib::DistanceComputationTools::ERROR_MEASURES ERROR_MEASURE;
		
		
		bool CLASSIFY_FACETS_BY_ANGLE; 
		double CLASSIF_ANGLE_STEP;
		double CLASSIF_MAX_DIST;
		
		bool EXPORT_FACETS; //Export to shape file
		QString SHAPE_FILENAME;
		bool USE_NATIVE_ORIENTATION;
		bool USE_GLOBAL_ORIENTATION;
		bool USE_CUSTOM_ORIENTATION;
		float NX;
		float NY;
		float NZ;

		bool EXPORT_FACETS_INFO; //exportFacetsInfo
		QString CSV_FILENAME;
		bool COORDS_IN_CSV;


		FacetsParams() : EXTRACT_FACETS(false)
			, ALGO(CellsFusionDlg::Algorithm::ALGO_KD_TREE)
			, KD_TREE_FUSION_MAX_ANGLE_DEG(20.0f)
			, KD_TREE_FUSION_MAX_RELATIVE_DISTANCE(1.0f)
			, OCTREE_LEVEL(8)
			, USE_RETRO_PROJECTION_ERROR(false)
			, ERROR_MAX_PER_FACET(0.2f)
			, MIN_POINTS_PER_FACET(10)
			, MAX_EDGE_LENGTH(1.0f)			
			, ERROR_MEASURE(CCCoreLib::DistanceComputationTools::MAX_DIST_99_PERCENT)						
			, CLASSIFY_FACETS_BY_ANGLE(false)
			, CLASSIF_ANGLE_STEP(30.0f)
			, CLASSIF_MAX_DIST(1.0f)
			, EXPORT_FACETS(false)
			, SHAPE_FILENAME("facets.shp")
			, USE_NATIVE_ORIENTATION(true)
			, USE_GLOBAL_ORIENTATION(false)
			, USE_CUSTOM_ORIENTATION(false)
			, NX(0.0f)
			, NY(0.0f)
			, NZ(1.0f)
			, EXPORT_FACETS_INFO(false)
			, CSV_FILENAME("facets.csv")
			, COORDS_IN_CSV(false){
				
			};				

	};
	

	//! Set of facets (pointers)
	typedef std::unordered_set<ccFacet*> FacetSet;
	
	static bool executeExportFacetsInfo(const FacetSet& facets,
								 const QString filename,
								 bool coordsInCSV=false,
								 bool useNativeOrientation=true,
								 bool useGlobalOrientation=false,
								 bool useCustomOrientation=false,
								 double nX=0.0f,
								 double nY=0.0f,
								 double nZ=1.0f, 
								 bool silent=false);
								 
    static bool executeExportFacets(const FacetSet& facets,
								 const QString filename,
								 bool useNativeOrientation=true,
								 bool useGlobalOrientation=false,
								 bool useCustomOrientation=false,
								 double nX=0.0f,
								 double nY=0.0f,
								 double nZ=1.0f,
								 bool silent=false);	
								 
	static QString polylineCoordsToWKT_POLYGONZ(const ccPolyline* polyline, 
											    int precision = 3);
												
	static ccGLMatrix calcOriRotMat(const FacetSet& facets,
									bool useNativeOrientation=true,
									bool useGlobalOrientation=false,
									bool useCustomOrientation=false,
									double nX=0.0f, 
									double nY=0.0f, 
									double nZ=1.0f);

								 
protected:

	//! Fuses the cells of a kd-tree to produces planar facets
	void fuseKdTreeCells();

	//! Uses Fast Marching to detect planar facets
	void extractFacetsWithFM();

	//! Exports facets (as shapefiles)
	void exportFacets();

	//! Exports statistics on a set of facets
	void exportFacetsInfo(); //GUI
	
	
	

	//! Classifies facets by orientation
	void classifyFacetsByAngle();

	//! Displays the selected entity stereogram
	void showStereogram();

protected:

	//! Uses the given algorithm to detect planar facets
	void extractFacets(CellsFusionDlg::Algorithm algo);

//protected:	
	//! Creates facets from components
	static ccHObject* createFacets(ccPointCloud* cloud,
	                        CCCoreLib::ReferenceCloudContainer& components,
	                        unsigned minPointsPerComponent,
	                        double maxEdgeLength,
	                        bool randomColors,
	                        bool& error,
							CCCoreLib::GenericProgressCallback* progress = nullptr);
//protected:							
public:
	//! Core logic for facet extraction from a point cloud
    static ccHObject* executeFacetExtraction(ccPointCloud* pc,
                                             const FacetsParams& params,
                                             bool& errorDuringFacetCreation,
                                             CCCoreLib::GenericProgressCallback* progress = nullptr);
protected:											 
	
							
	//! Set of facets (pointers)
	//typedef std::unordered_set<ccFacet*> FacetSet;

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
