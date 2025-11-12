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

#ifndef FACETS_PLUGIN_COMMANDS_HEADER
#define FACETS_PLUGIN_COMMANDS_HEADER

//CloudCompare
#include "ccCommandLineInterface.h" 


//Local
#include "qFacets.h"

//Q
#include <QFileInfo>
#include <QDir>

static const char COMMAND_FACETS[] = "FACETS";

constexpr char EXTRACT_FACETS[] = "-EXTRACT_FACETS";
constexpr char ALGO[] = "-ALGO";	
	constexpr char ALGO_FAST_MARCHING[] = "ALGO_FAST_MARCHING";
	constexpr char ALGO_KD_TREE[] = "ALGO_KD_TREE";
//ALGO_KD_TREE
constexpr char KD_TREE_FUSION_MAX_ANGLE_DEG[] = "-KD_TREE_FUSION_MAX_ANGLE_DEG";
constexpr char KD_TREE_FUSION_MAX_RELATIVE_DISTANCE[] = "-KD_TREE_FUSION_MAX_RELATIVE_DISTANCE";
//ALGO_FAST_MARCHING
constexpr char OCTREE_LEVEL[] = "-OCTREE_LEVEL";
constexpr char USE_RETRO_PROJECTION_ERROR[] = "-USE_RETRO_PROJECTION_ERROR";
//both ALGO_KD_TREE ALGO_FAST_MARCHING and 
constexpr char ERROR_MAX_PER_FACET[] = "-ERROR_MAX_PER_FACET";
//int ERROR_MEASURE_TYPE; //static int		s_errorMeasureType = 3; //max dist @ 99 %
constexpr char MAX_EDGE_LENGTH[] = "-MAX_EDGE_LENGTH";
constexpr char MIN_POINTS_PER_FACET[] = "MIN_POINTS_PER_FACET";
constexpr char ERROR_MEASURE[] = "-ERROR_MEASURE";
	constexpr char RMS[] = "RMS";
	constexpr char MAX_DIST_68_PERCENT[] = "MAX_DIST_68_PERCENT";
	constexpr char MAX_DIST_95_PERCENT[] = "MAX_DIST_95_PERCENT";
	constexpr char MAX_DIST_99_PERCENT[] = "MAX_DIST_99_PERCENT";
	constexpr char MAX_DIST[] = "MAX_DIST";


constexpr char CLASSIFY_FACETS_BY_ANGLE[] = "-CLASSIFY_FACETS_BY_ANGLE";
constexpr char CLASSIF_ANGLE_STEP[] = "-CLASSIF_ANGLE_STEP";
constexpr char CLASSIF_MAX_DIST[] = "-CLASSIF_MAX_DIST";

constexpr char EXPORT_FACETS[] = "-EXPORT_FACETS";
constexpr char SHAPE_FILENAME[] = "-SHAPE_FILENAME";
constexpr char USE_NATIVE_ORIENTATION[] = "-USE_NATIVE_ORIENTATION";
constexpr char USE_GLOBAL_ORIENTATION[] = "-USE_GLOBAL_ORIENTATION";
constexpr char USE_CUSTOM_ORIENTATION[] = "-USE_CUSTOM_ORIENTATION";


constexpr char EXPORT_FACETS_INFO[] = "-EXPORT_FACETS_INFO";
constexpr char CSV_FILENAME[] = "-CSV_FILENAME";




//##########################################################################
//
// ADD FACETS FROM A FacetSet TO A ccHObject* group
//
//##########################################################################

void addFacetsToGroup(ccHObject* group, const qFacets::FacetSet& facets)
{    
    for (ccFacet* facet : facets) {
        if (facet) {
            // ccHObject::addChild takes ownership of the pointer.
            // We safely upcast the ccFacet* to ccHObject*
            group->addChild(facet);
        }
    }
    // Now, the 'group' object contains a pointer to every single 
     //ccFacet object originally referenced in the 'facets' set.
}

//##########################################################################
//
// ADD FACETS IN A ccHObject* group TO A FacetSet
//
//##########################################################################

void getFacetsFromGroup(ccHObject* group, qFacets::FacetSet& facets)
{
	facets.clear();
	ccHObject::Container childFacets;
	group->filterChildren(childFacets, true, CC_TYPES::FACET);
	
	for ( ccHObject *childFacet : childFacets )
	{
		ccFacet* facet = static_cast<ccFacet*>(childFacet);
		if (facet->getContour())
		{
			facets.insert(facet);			
		}				
	}	
}


struct CommandFacets : public ccCommandLineInterface::Command
{
	CommandFacets() : ccCommandLineInterface::Command(QObject::tr("FACETS"), COMMAND_FACETS) {}

	virtual bool process(ccCommandLineInterface& cmd) override
	{
		cmd.print("[FACETS]");		
		if (cmd.clouds().empty())
		{
			return cmd.error(QObject::tr("No point cloud to attempt FACETS on (be sure to open one with \"-O [cloud filename]\" before \"-%2\")").arg(COMMAND_FACETS));
		}
		
		
		
		qFacets::FacetsParams params;
		QStringList paramNames = QStringList() << EXTRACT_FACETS << ALGO << 			
			KD_TREE_FUSION_MAX_ANGLE_DEG << KD_TREE_FUSION_MAX_RELATIVE_DISTANCE << 
			OCTREE_LEVEL << USE_RETRO_PROJECTION_ERROR << 
			ERROR_MAX_PER_FACET << MIN_POINTS_PER_FACET << MAX_EDGE_LENGTH << ERROR_MEASURE << 			
			CLASSIFY_FACETS_BY_ANGLE << CLASSIF_ANGLE_STEP << CLASSIF_MAX_DIST << 
			EXPORT_FACETS << SHAPE_FILENAME << USE_NATIVE_ORIENTATION << USE_GLOBAL_ORIENTATION << USE_CUSTOM_ORIENTATION << 
			EXPORT_FACETS_INFO << CSV_FILENAME;
		QStringList algoNames = QStringList() << ALGO_FAST_MARCHING << ALGO_KD_TREE;
		QStringList errorMeasureNames = QStringList() << RMS << MAX_DIST_68_PERCENT << MAX_DIST_95_PERCENT << MAX_DIST_99_PERCENT << MAX_DIST;
		
		
		if (!cmd.arguments().empty())
		{

			QString param = cmd.arguments().takeFirst().toUpper();
			
			cmd.print(QObject::tr("raw \t%1 : %2").arg(COMMAND_FACETS, param));
			while (paramNames.contains(param))
			{
				cmd.print(QObject::tr("\t%1 : %2").arg(COMMAND_FACETS, param));
				
				//EXTRACT_FACETS
				if (param == EXTRACT_FACETS)
				{
					params.EXTRACT_FACETS = true;				
				}
				// ALGO
				else if(param == ALGO)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: Algorithm type after \"-%1 %2\"").arg(COMMAND_FACETS, ALGO));
					}
					
					QString val = cmd.arguments().takeFirst().toUpper();					
					cmd.print(QObject::tr("\t-ALGO : %1").arg(val));
					if (algoNames.contains(val))					
					{
						if (val == ALGO_FAST_MARCHING)
						{
							params.ALGO = CellsFusionDlg::Algorithm::ALGO_FAST_MARCHING;
						}
						else if (val == ALGO_KD_TREE)
						{
							params.ALGO = CellsFusionDlg::Algorithm::ALGO_KD_TREE;
						}
					}
					else
						return cmd.error(QObject::tr("No valid parameter: Algorithm type after \"-%1 %2\"").arg(COMMAND_FACETS, ALGO));
					{
					}
					
				}
				//KD_TREE_FUSION_MAX_ANGLE_DEG
				else if (param == KD_TREE_FUSION_MAX_ANGLE_DEG)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 %2\"").arg(COMMAND_FACETS, KD_TREE_FUSION_MAX_ANGLE_DEG));
					}
					bool ok;
					float val = cmd.arguments().takeFirst().toFloat(&ok);
					if (!ok)
					{
						return cmd.error("Invalid number for -KD_TREE_FUSION_MAX_ANGLE_DEG!");
					}
					cmd.print(QObject::tr("\t-KD_TREE_FUSION_MAX_ANGLE_DEG : %1").arg(val));
					params.KD_TREE_FUSION_MAX_ANGLE_DEG = val;
					
				}
				// KD_TREE_FUSION_MAX_RELATIVE_DISTANCE
				else if (param == KD_TREE_FUSION_MAX_RELATIVE_DISTANCE)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 %2\"").arg(COMMAND_FACETS, KD_TREE_FUSION_MAX_RELATIVE_DISTANCE));
					}
					bool ok;
					float val = cmd.arguments().takeFirst().toFloat(&ok);
					if (!ok)
					{
						return cmd.error("Invalid number for -KD_TREE_FUSION_MAX_RELATIVE_DISTANCE!");
					}
					cmd.print(QObject::tr("\t-KD_TREE_FUSION_MAX_RELATIVE_DISTANCE : %1").arg(val));
					params.KD_TREE_FUSION_MAX_RELATIVE_DISTANCE = val;
					
				}
				// OCTREE_LEVEL
				else if (param == OCTREE_LEVEL)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 %2\"").arg(COMMAND_FACETS, OCTREE_LEVEL));
					}
					bool ok;
					float val = cmd.arguments().takeFirst().toInt(&ok);
					if (!ok)
					{
						return cmd.error("Invalid number for -OCTREE_LEVEL!");
					}
					cmd.print(QObject::tr("\t-OCTREE_LEVEL : %1").arg(val));
					params.KD_TREE_FUSION_MAX_RELATIVE_DISTANCE = val;
					
				}
				// USE_RETRO_PROJECTION_ERROR
				else if (param == USE_RETRO_PROJECTION_ERROR)
				{
					params.USE_RETRO_PROJECTION_ERROR = true;				
				}				
				// ERROR_MAX_PER_FACET
				else if (param == ERROR_MAX_PER_FACET)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 %2\"").arg(COMMAND_FACETS, ERROR_MAX_PER_FACET));
					}
					bool ok;
					float val = cmd.arguments().takeFirst().toFloat(&ok);
					if (!ok)
					{
						return cmd.error("Invalid number for -ERROR_MAX_PER_FACET!");
					}
					cmd.print(QObject::tr("\t-ERROR_MAX_PER_FACET : %1").arg(val));
					params.ERROR_MAX_PER_FACET = val;
					
				}
				// MIN_POINTS_PER_FACET
				else if (param == MIN_POINTS_PER_FACET)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 %2\"").arg(COMMAND_FACETS, MIN_POINTS_PER_FACET));
					}
					bool ok;
					float val = cmd.arguments().takeFirst().toInt(&ok);
					if (!ok)
					{
						return cmd.error("Invalid number for -MIN_POINTS_PER_FACET!");
					}
					cmd.print(QObject::tr("\t-MIN_POINTS_PER_FACET : %1").arg(val));
					params.MIN_POINTS_PER_FACET = val;
					
				}
				
				// MAX_EDGE_LENGTH
				else if (param == MAX_EDGE_LENGTH)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 %2\"").arg(COMMAND_FACETS, MAX_EDGE_LENGTH));
					}
					bool ok;
					float val = cmd.arguments().takeFirst().toFloat(&ok);
					if (!ok)
					{
						return cmd.error("Invalid number for -MAX_EDGE_LENGTH!");
					}
					cmd.print(QObject::tr("\t-MAX_EDGE_LENGTH : %1").arg(val));
					params.MAX_EDGE_LENGTH = val;
					
				}
				// ERROR_MEASURE
				else if(param == ERROR_MEASURE)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: ERROR_MEASURE type after \"-%1 %2\"").arg(COMMAND_FACETS, ERROR_MEASURE));
					}
					
					QString val = cmd.arguments().takeFirst().toUpper();					
					cmd.print(QObject::tr("\t-ERROR_MEASURE : %1").arg(val));
					if (errorMeasureNames.contains(val))					
					{						
						if (val == RMS)
						{
							params.ERROR_MEASURE = CCCoreLib::DistanceComputationTools::RMS;
						}
						else if (val == MAX_DIST_68_PERCENT)
						{
							params.ERROR_MEASURE = CCCoreLib::DistanceComputationTools::MAX_DIST_68_PERCENT;
						}
						else if (val == MAX_DIST_95_PERCENT)
						{
							params.ERROR_MEASURE = CCCoreLib::DistanceComputationTools::MAX_DIST_95_PERCENT;
						}
						else if (val == MAX_DIST_99_PERCENT)
						{
							params.ERROR_MEASURE = CCCoreLib::DistanceComputationTools::MAX_DIST_99_PERCENT;
						}
						else if (val == MAX_DIST)
						{
							params.ERROR_MEASURE = CCCoreLib::DistanceComputationTools::MAX_DIST_99_PERCENT;
						}
					}
					else
						return cmd.error(QObject::tr("No valid parameter: Error measure type after \"-%1 %2\"").arg(COMMAND_FACETS, ERROR_MEASURE));
					{
					}
					
					
				}
				// CLASSIFY_FACETS_BY_ANGLE
				else if (param == CLASSIFY_FACETS_BY_ANGLE)
				{
					params.CLASSIFY_FACETS_BY_ANGLE = true;				
				}	
				// CLASSIF_ANGLE_STEP
				else if (param == CLASSIF_ANGLE_STEP)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 %2\"").arg(COMMAND_FACETS, CLASSIF_ANGLE_STEP));
					}
					bool ok;
					float val = cmd.arguments().takeFirst().toFloat(&ok);
					if (!ok)
					{
						return cmd.error("Invalid number for -CLASSIF_ANGLE_STEP!");
					}
					cmd.print(QObject::tr("\t-CLASSIF_ANGLE_STEP : %1").arg(val));
					params.CLASSIF_ANGLE_STEP = val;
					
				}
				// CLASSIF_MAX_DIST
				else if (param == CLASSIF_MAX_DIST)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 %2\"").arg(COMMAND_FACETS, CLASSIF_MAX_DIST));
					}
					bool ok;
					float val = cmd.arguments().takeFirst().toFloat(&ok);
					if (!ok)
					{
						return cmd.error("Invalid number for -CLASSIF_MAX_DIST!");
					}
					cmd.print(QObject::tr("\t-CLASSIF_MAX_DIST : %1").arg(val));
					params.CLASSIF_MAX_DIST = val;
					
				}
				// EXPORT_FACETS
				else if (param == EXPORT_FACETS)
				{
					params.EXPORT_FACETS = true;				
				}	
				//SHAPE_FILENAME
				else if(param == SHAPE_FILENAME)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: filepath after \"-%1 %2\"").arg(COMMAND_FACETS, SHAPE_FILENAME));
					}
					
					QString val = cmd.arguments().takeFirst().toUpper();
									
					cmd.print(QObject::tr("\t-SHAPE_FILENAME : %1").arg(val));
					params.SHAPE_FILENAME = val;					
					
				}
				//USE_NATIVE_ORIENTATION
				else if (param == USE_NATIVE_ORIENTATION)
				{
					params.USE_NATIVE_ORIENTATION = true;
					params.USE_GLOBAL_ORIENTATION = false;
					params.USE_CUSTOM_ORIENTATION = false;
					
				}	
				//USE_GLOBAL_ORIENTATION
				else if (param == USE_GLOBAL_ORIENTATION)
				{
					params.USE_NATIVE_ORIENTATION = false;
					params.USE_GLOBAL_ORIENTATION = true;
					params.USE_CUSTOM_ORIENTATION = false;
					
				}
				//USE_CUSTOM_ORIENTATION
				else if (param == USE_CUSTOM_ORIENTATION)
				{
					params.USE_NATIVE_ORIENTATION = false;
					params.USE_GLOBAL_ORIENTATION = false;
					params.USE_CUSTOM_ORIENTATION = true;
					
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: Nx \"-%1 %2\"").arg(COMMAND_FACETS, USE_CUSTOM_ORIENTATION));
					}
					
					bool ok;
					float val = cmd.arguments().takeFirst().toFloat(&ok);
					if (!ok)
					{
						cmd.error("Invalid number for Nx in -USE_CUSTOM_ORIENTATION!");
						
					}
					params.NX = val;
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: Ny \"-%1 %2\"").arg(COMMAND_FACETS, USE_CUSTOM_ORIENTATION));
					}
					val = cmd.arguments().takeFirst().toFloat(&ok);
					if (!ok)
					{
						cmd.error("Invalid number for Ny in -USE_CUSTOM_ORIENTATION!");
						
					}
					params.NY = val;
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: Nz \"-%1 %2\"").arg(COMMAND_FACETS, USE_CUSTOM_ORIENTATION));
					}
					val = cmd.arguments().takeFirst().toFloat(&ok);
					if (!ok)
					{
						cmd.error("Invalid number for Nz in -USE_CUSTOM_ORIENTATION!");
						
					}
					params.NZ = val;
										
				}
				// EXPORT_FACETS_INFO
				else if (param == EXPORT_FACETS_INFO)
				{
					params.EXPORT_FACETS_INFO = true;				
				}	
				//CSV_FILENAME
				else if(param == CSV_FILENAME)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: filepath after \"-%1 %2\"").arg(COMMAND_FACETS, CSV_FILENAME));
					}
					
					QString val = cmd.arguments().takeFirst().toUpper();
									
					cmd.print(QObject::tr("\t-CSV_FILENAME : %1").arg(val));
					params.CSV_FILENAME = val;					
					
				}

				if (cmd.arguments().empty())
				{
					break;
				}
				param = cmd.arguments().takeFirst().toUpper();
			}
			if (!paramNames.contains(param)) //not sure if I need this
			{
				cmd.arguments().push_front(param);
			}
		
		}
		
		if (!params.EXTRACT_FACETS && !params.CLASSIFY_FACETS_BY_ANGLE && !params.EXPORT_FACETS && EXPORT_FACETS_INFO)
		{
			return cmd.error(QObject::tr("No valid parameter: Need one of -%2, -%3, -%4, or -%5 after Algorithm type after \"-%1\"").arg(COMMAND_FACETS,EXTRACT_FACETS,CLASSIFY_FACETS_BY_ANGLE,EXPORT_FACETS,EXPORT_FACETS_INFO));
		}
		
		// Print out relevant Parameters including default
		cmd.print(QObject::tr("[FACETS] Parameters including default"));
		if (params.EXTRACT_FACETS)
		{
			cmd.print(QObject::tr("\t-EXTRACT_FACETS"));
			
			if (params.ALGO == CellsFusionDlg::Algorithm::ALGO_KD_TREE)
			{
				cmd.print(QObject::tr("\t\t-ALGO : ALGO_KD_TREE"));
				cmd.print(QObject::tr("\t\t\t-KD_TREE_FUSION_MAX_ANGLE_DEG : %1").arg(params.KD_TREE_FUSION_MAX_ANGLE_DEG));
				cmd.print(QObject::tr("\t\t\t-KD_TREE_FUSION_MAX_RELATIVE_DISTANCE : %1").arg(params.KD_TREE_FUSION_MAX_RELATIVE_DISTANCE));
				
				
			}
			else if (params.ALGO == CellsFusionDlg::Algorithm::ALGO_FAST_MARCHING)
			{
				cmd.print(QObject::tr("\t\t-ALGO ALGO_FAST_MARCHING"));
				cmd.print(QObject::tr("\t\t\t-OCTREE_LEVEL : %1").arg(params.OCTREE_LEVEL));
				cmd.print(QObject::tr("\t\t\t-USE_RETRO_PROJECTION_ERROR : %1").arg(params.USE_RETRO_PROJECTION_ERROR));
			}			
			
			
			if (params.ERROR_MEASURE ==CCCoreLib::DistanceComputationTools::RMS) 
			{
				cmd.print(QObject::tr("\t\t-ERROR_MEASURE RMS"));
			}
			else if (params.ERROR_MEASURE ==CCCoreLib::DistanceComputationTools::MAX_DIST_68_PERCENT) 
			{
				cmd.print(QObject::tr("\t\t-ERROR_MEASURE MAX_DIST_68_PERCENT"));
			}
			else if (params.ERROR_MEASURE ==CCCoreLib::DistanceComputationTools::MAX_DIST_95_PERCENT) 
			{
				cmd.print(QObject::tr("\t\t-ERROR_MEASURE MAX_DIST_95_PERCENT"));
			}
			else if (params.ERROR_MEASURE ==CCCoreLib::DistanceComputationTools::MAX_DIST_99_PERCENT) 
			{
				cmd.print(QObject::tr("\t\t-ERROR_MEASURE MAX_DIST_99_PERCENT"));
			}
			else if (params.ERROR_MEASURE ==CCCoreLib::DistanceComputationTools::MAX_DIST) 
			{
				cmd.print(QObject::tr("\t\t-ERROR_MEASURE MAX_DIST"));
			}
			cmd.print(QObject::tr("\t\t-ERROR_MAX_PER_FACET : %1").arg(params.ERROR_MAX_PER_FACET));
			cmd.print(QObject::tr("\t\t-MIN_POINTS_PER_FACET : %1").arg(params.MIN_POINTS_PER_FACET));
			cmd.print(QObject::tr("\t\t-MAX_EDGE_LENGTH : %1").arg(params.MAX_EDGE_LENGTH));
		}
			
		if (params.CLASSIFY_FACETS_BY_ANGLE)
		{
			cmd.print(QObject::tr("\t-CLASSIFY_FACETS_BY_ANGLE"));
			cmd.print(QObject::tr("\t\t-CLASSIF_ANGLE_STEP : %1").arg(params.CLASSIF_ANGLE_STEP));
			cmd.print(QObject::tr("\t\t-CLASSIF_MAX_DIST : %1").arg(params.CLASSIF_MAX_DIST));
		}
		if (params.EXPORT_FACETS)
		{
			cmd.print(QObject::tr("\t-EXPORT_FACETS"));
			cmd.print(QObject::tr("\t\t-SHAPE_FILENAME : \"%1\"").arg(params.SHAPE_FILENAME));
			if (params.USE_NATIVE_ORIENTATION)
			{
				cmd.print(QObject::tr("\t\t-USE_NATIVE_ORIENTATION"));
			}
			else if (params.USE_GLOBAL_ORIENTATION)
			{
				cmd.print(QObject::tr("\t\t-USE_GLOBAL_ORIENTATION"));
			}
			else if (params.USE_CUSTOM_ORIENTATION)
			{
				cmd.print(QObject::tr("\t\t-USE_CUSTOM_ORIENTATION : %1 %2 %3").arg(params.NX).arg(params.NY).arg(params.NZ));
			}								
		}
		if (params.EXPORT_FACETS_INFO)
		{
			cmd.print(QObject::tr("\t-EXPORT_FACETS_INFO"));
			cmd.print(QObject::tr("\t\t-CSV_FILENAME : \"%1\"").arg(params.CSV_FILENAME));										
		}
		
		
		//Execute the facet extraction, classification by angle, and csv/shape export.
		
		for (CLCloudDesc clCloud : cmd.clouds())
		{
			
			qFacets::FacetSet facets;
			ccHObject* group=nullptr;
			
			//execute EXTRACT_FACETS
			if (params.EXTRACT_FACETS)
			{
				cmd.print(QObject::tr("[FACETS] Extracting Facets:  \"%1\"").arg(clCloud.pc->getName()));
				bool errorDuringFacetCreation = false;				
				ccHObject* group = qFacets::executeFacetExtraction(clCloud.pc, 
                                           params, 
                                           errorDuringFacetCreation,
                                           nullptr);				
				
				if (errorDuringFacetCreation)
				{
					cmd.error(QObject::tr("[FACETS] Failed to extract facets."));	
					return false;
				}
								
				getFacetsFromGroup(group, facets); //probably a better way that going between ccHObject group and facetSet
				if (facets.empty())
				{
					cmd.error(QObject::tr("[FACETS] Did not extract any facets."));
					return false;
				}				
				cmd.print(QObject::tr("[FACETS] Extracted %1 facets").arg(facets.size()));
			}

			//execute CLASSIFY_FACETS_BY_ANGLE						
			if (params.CLASSIFY_FACETS_BY_ANGLE)
			{
				cmd.print(QObject::tr("[FACETS] Classifying facets by angles."));
				if (facets.empty())
				{
					cmd.error(QObject::tr("[FACETS] Need facets.  Must use -EXTRACT_FACETS."));					
					return false;
				}
				
				
				ccHObject* group = new ccHObject(QString("FACETS group"));
				addFacetsToGroup(group, facets);  //probably a better way that going between ccHObject group and facetSet
				
				bool success = FacetsClassifier::ByOrientation(group, params.CLASSIF_ANGLE_STEP, params.CLASSIF_MAX_DIST);
				if (!success)
				{
					cmd.error(QObject::tr("[FACETS] Failed to Classify facets by angles."));	
					return false;
				}
			}
				
			//execute EXPORT_FACETS
			if (params.EXPORT_FACETS)
			{
				cmd.print(QObject::tr("[FACETS] Exporting Facets info to shape file"));
				if (facets.empty())
				{
					cmd.error(QObject::tr("[FACETS] Need facets.  Must use -EXTRACT_FACETS."));					
					return false;
				}
				
							
				QFileInfo fileInfo(params.SHAPE_FILENAME);			
				QString directoryPath = fileInfo.path();
				
				QString newFileName = clCloud.pc->getName() + QString("_") + fileInfo.fileName();   
				QString outputName = QDir(directoryPath).filePath(newFileName);	
				QDir dir;
				if (!dir.mkpath(directoryPath)) 
				{
					cmd.error(QObject::tr("[FACETS] Failed to create directories %1").arg(outputName));
					return false;
				}
				bool success = qFacets::executeExportFacets(facets, 
				                                            outputName,
															params.USE_NATIVE_ORIENTATION, 
															params.USE_GLOBAL_ORIENTATION,
															USE_CUSTOM_ORIENTATION,
															params.NX,
															params.NY,
															params.NZ,
															cmd.silentMode());
				if (!success)
				{
					cmd.error(QObject::tr("[FACETS] Failed to Export Facets to shape file."));	
					return false;
				}
				
			}
			//execute EXPORT_FACETS_INFO
			if (params.EXPORT_FACETS_INFO)
			{
				cmd.print(QObject::tr("[FACETS] Exporting Facets info to csv."));
				if (facets.empty())
				{
					cmd.error(QObject::tr("[FACETS] Need facets.  Must have -EXTRACT_FACETS."));					
					return false;
				}
				
				QFileInfo fileInfo(params.CSV_FILENAME);			
				QString directoryPath = fileInfo.path();        				
				QString newFileName = clCloud.pc->getName() + QString("_") + fileInfo.fileName();   
				QString outputName = QDir(directoryPath).filePath(newFileName);	
				QDir dir;
				if (!dir.mkpath(directoryPath)) 
				{
					cmd.error(QObject::tr("[FACETS] Failed to create directories %1").arg(outputName));
					return false;
				}
				bool success = qFacets::executeExportFacetsInfo(facets, outputName, cmd.silentMode());
				if (!success)
				{
					cmd.error(QObject::tr("[FACETS] Failed to Export Facets Info to csv"));
					return false;
				}								
			}
		
		}

		return true;
	}

};

#endif //FACETS_PLUGIN_COMMANDS_HEADER


