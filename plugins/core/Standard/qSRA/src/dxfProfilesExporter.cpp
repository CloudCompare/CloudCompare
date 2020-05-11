//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qSRA                         #
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
//#                           COPYRIGHT: EDF                               #
//#                                                                        #
//##########################################################################

#include "dxfProfilesExporter.h"

//qCC_db
#include <ccPolyline.h>

//qCC_plugins
#include <ccMainAppInterface.h>

//DXF lib
#ifdef CC_DXF_SUPPORT
#include <dl_dxf.h>
#endif

bool DxfProfilesExporter::IsEnabled()
{
#ifdef CC_DXF_SUPPORT
	return true;
#else
	return false;
#endif
}

//constants
static const char PROFILE_LAYER[] = "theoretical_profile";
static const char LEGEND_LAYER[] = "legend";
static const char VERT_PROFILE_LAYER[] = "vertical_profile_%1";
static const char HORIZ_PROFILE_LAYER[] = "horizontal_profile_%1";
static const int s_lineWidth = 5;

static const double c_textHeight_mm = 2.5;
static const double c_textMargin_mm = 2.0;
static const double c_pageHeight_mm = 297.0;
static const double c_pageWidth_mm = 210.0;
static const double c_pageMargin_mm = 10.0;
static const double c_profileMargin_mm = 50.0;

struct VertStepData
{
	double height;
	double radius_th;
	double deviation;
};

bool DxfProfilesExporter::SaveVerticalProfiles(	const QSharedPointer<DistanceMapGenerationTool::Map>& map,
												ccPolyline* profile,
												QString filename,
												unsigned angularStepCount,
												double heightStep,
												double heightShift,
												const Parameters& params,
												ccMainAppInterface* app/*=0*/)
{
#ifdef CC_DXF_SUPPORT
	assert(c_pageMargin_mm < c_profileMargin_mm);
	assert(2.0*c_profileMargin_mm < std::min(c_pageWidth_mm,c_pageHeight_mm));

	if (!map || !profile || angularStepCount == 0 || heightStep <= 0)
	{
		//invalid parameters
		return false;
	}

	//Theoretical profile bounding box
	CCVector3 profileBBMin;
	CCVector3 profileBBMax;
	profile->getAssociatedCloud()->getBoundingBox(profileBBMin,profileBBMax);
	//Mix with the map's boundaries along 'Y'
	double yMin = std::max(map->yMin, static_cast<double>(profileBBMin.y)+heightShift);
	double yMax = std::min(map->yMin + map->ySteps * map->yStep, static_cast<double>(profileBBMax.y)+heightShift);
	const double ySpan = yMax - yMin;
	//For the 'X' dimension, it's easier to stick with the th. profile
	const double xMin = profileBBMin.x;
//	const double xMax = profileBBMax.x;
	const double xSpan = profileBBMax.x - profileBBMin.x;

	if (xSpan == 0.0 && ySpan == 0.0)
	{
		if (app)
			app->dispToConsole(QString("Internal error: null profile?!"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	DL_Dxf dxf;
	DL_WriterA* dw = dxf.out(qPrintable(filename), DL_VERSION_R12);
	if (!dw)
	{
		if (app)
			app->dispToConsole(QString("Failed to open '%1' file for writing!").arg(filename),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	//write header
	dxf.writeHeader(*dw);

	//add dimensions
	dw->dxfString(9, "$INSBASE");
	dw->dxfReal(10,0.0);
	dw->dxfReal(20,0.0);
	dw->dxfReal(30,0.0);
	dw->dxfString(9, "$EXTMIN");
	dw->dxfReal(10,0.0);
	dw->dxfReal(20,0.0);
	dw->dxfReal(30,0.0);
	dw->dxfString(9, "$EXTMAX");
	dw->dxfReal(10,c_pageWidth_mm);
	dw->dxfReal(20,c_pageHeight_mm);
	dw->dxfReal(30,0.0);
	dw->dxfString(9, "$LIMMIN");
	dw->dxfReal(10,0.0);
	dw->dxfReal(20,0.0);
	dw->dxfString(9, "$LIMMAX");
	dw->dxfReal(10,c_pageWidth_mm);
	dw->dxfReal(20,c_pageHeight_mm);

	//close header
	dw->sectionEnd();

	//Opening the Tables Section
	dw->sectionTables();
	//Writing the Viewports
	dxf.writeVPort(*dw);

	//Writing the Linetypes (all by default)
	{
		dw->tableLinetypes(3);
		dxf.writeLinetype(*dw, DL_LinetypeData("BYBLOCK", "BYBLOCK", 0, 0, 0.0));
		dxf.writeLinetype(*dw, DL_LinetypeData("BYLAYER", "BYLAYER", 0, 0, 0.0));
		dxf.writeLinetype(*dw, DL_LinetypeData("CONTINUOUS", "Continuous", 0, 0, 0.0));
		dw->tableEnd();
	}

	//Writing the Layers
	dw->tableLayers(angularStepCount+3);
	QStringList profileNames;
	{
		//default layer
		dxf.writeLayer(*dw, 
			DL_LayerData("0", 0), 
			DL_Attributes(
			std::string(""),		// leave empty
			DL_Codes::black,		// default color
			100,					// default width (in 1/100 mm)
			"CONTINUOUS",			// default line style
			1.0						// linetypeScale
			));

		//theoretical profile layer
		dxf.writeLayer(*dw, 
			DL_LayerData(PROFILE_LAYER, 0), 
			DL_Attributes(
			std::string(""),
			DL_Codes::red,
			s_lineWidth,
			"CONTINUOUS",
			1.0));

		//legend layer
		dxf.writeLayer(*dw, 
			DL_LayerData(LEGEND_LAYER, 0), 
			DL_Attributes(
			std::string(""),
			DL_Codes::black,
			s_lineWidth,
			"CONTINUOUS",
			1.0));

		//vert. profile layers
		for (unsigned i=0; i<angularStepCount; ++i)
		{
			//default layer name
			QString layerName = QString(VERT_PROFILE_LAYER).arg(i+1,3,10,QChar('0'));
			//but we use the profile title if we have one!
			//DGM: nope, as it may be longer than 31 characters (R14 limit)
			//if (params.profileTitles.size() >= static_cast<int>(angularStepCount))
			//{
			//	layerName = params.profileTitles[i];
			//	layerName.replace(QChar(' '),QChar('_'));
			//}

			profileNames << layerName;
			dxf.writeLayer(*dw, 
				DL_LayerData(qPrintable(layerName), 0), //DGM: warning, toStdString doesn't preserve "local" characters
				DL_Attributes(
				std::string(""),
				i == 0 ? DL_Codes::green : -DL_Codes::green, //invisible if negative!
				s_lineWidth,
				"CONTINUOUS",
				1.0));
		}
	}
	dw->tableEnd();

	//Writing Various Other Tables
	//dxf.writeStyle(*dw); //DXFLIB V2.5
	dw->tableStyle(1);
	dxf.writeStyle(*dw, DL_StyleData("Standard", 0, 0.0, 0.75, 0.0, 0, 2.5, "txt", "")); //DXFLIB V3.3
	dw->tableEnd();

	dxf.writeView(*dw);
	dxf.writeUcs(*dw);

	dw->tableAppid(1);
	dw->tableAppidEntry(0x12);
	dw->dxfString(2, "ACAD");
	dw->dxfInt(70, 0);
	dw->tableEnd();

	//Writing Dimension Styles
	dxf.writeDimStyle(	*dw, 
						/*arrowSize*/1, 
						/*extensionLineExtension*/1,
						/*extensionLineOffset*/1,
						/*dimensionGap*/1,
						/*dimensionTextSize*/1);
	
	//Writing Block Records
	dxf.writeBlockRecord(*dw);
	//dxf.writeBlockRecord(*dw, "myblock1");
	//dxf.writeBlockRecord(*dw, "myblock2");
	dw->tableEnd();

	//Ending the Tables Section
	dw->sectionEnd();

	//Writing the Blocks Section
	{
		dw->sectionBlocks();

		dxf.writeBlock(*dw,  DL_BlockData("*Model_Space", 0, 0.0, 0.0, 0.0));
		dxf.writeEndBlock(*dw, "*Model_Space");

		dxf.writeBlock(*dw, DL_BlockData("*Paper_Space", 0, 0.0, 0.0, 0.0));
		dxf.writeEndBlock(*dw, "*Paper_Space");

		dxf.writeBlock(*dw, DL_BlockData("*Paper_Space0", 0, 0.0, 0.0, 0.0));
		dxf.writeEndBlock(*dw, "*Paper_Space0");

		dw->sectionEnd();
	}

	//Writing the Entities Section
	{
		dw->sectionEntities();

		//we make the profile fit in the middle of the page (21.0 x 29.7 cm)
		double scale = 1.0;
		if (xSpan == 0)
		{
			assert(ySpan != 0);
			scale = (c_pageHeight_mm - 2.0 * c_profileMargin_mm)/ySpan;
		}
		else if (ySpan == 0)
		{
			assert(xSpan != 0);
			scale = (c_pageWidth_mm - 2.0 * c_profileMargin_mm)/xSpan;
		}
		else
		{
			scale = std::min(	(c_pageWidth_mm - 2.0 * c_profileMargin_mm)/xSpan,
								(c_pageHeight_mm - 2.0 * c_profileMargin_mm)/ySpan );
		}

		//min corner of profile area
		const double x0 = (c_pageWidth_mm - xSpan*scale) / 2.0;
		const double y0 = (c_pageHeight_mm - ySpan*scale) / 2.0;

		//write theoretical profile (polyline)
		{
			unsigned vertexCount = profile->size();
			dxf.writePolyline(	*dw,
								DL_PolylineData(static_cast<int>(vertexCount),0,0,0),
								DL_Attributes(PROFILE_LAYER, DL_Codes::bylayer, -1, "BYLAYER", 1.0));

			for (unsigned i = 0; i < vertexCount; ++i)
			{
				const CCVector3* P = profile->getPoint(i);
				dxf.writeVertex(*dw, DL_VertexData(x0 + (P->x - xMin)*scale, y0 + (P->y + heightShift - yMin)*scale, 0.0));
			}

			dxf.writePolylineEnd(*dw);
		}

		//write legend
		{
			DL_Attributes DefaultLegendMaterial(LEGEND_LAYER, DL_Codes::bylayer, -1, "BYLAYER", 1.0);

			//write page contour
			{
				dxf.writePolyline(	*dw,
									DL_PolylineData(4, 0, 0, 1),
									DefaultLegendMaterial);

				dxf.writeVertex(*dw, DL_VertexData(	c_pageMargin_mm, c_pageMargin_mm, 0.0));
				dxf.writeVertex(*dw, DL_VertexData(	c_pageMargin_mm, c_pageHeight_mm-c_pageMargin_mm, 0.0));
				dxf.writeVertex(*dw, DL_VertexData(	c_pageWidth_mm-c_pageMargin_mm, c_pageHeight_mm-c_pageMargin_mm, 0.0));
				dxf.writeVertex(*dw, DL_VertexData(	c_pageWidth_mm-c_pageMargin_mm, c_pageMargin_mm, 0.0));

				dxf.writePolylineEnd(*dw);
			}

			double xLegend = c_pageMargin_mm + 2.0*c_textHeight_mm;
			double yLegend = c_pageMargin_mm + 2.0*c_textHeight_mm;
			const double legendWidth_mm = 20.0;
			
			//deviation magnification factor
			QString magnifyStr = QString::number(params.devMagnifyCoef);
			dxf.writeText(	*dw,
				DL_TextData(xLegend, yLegend, 0.0, xLegend, yLegend, 0.0, c_textHeight_mm, 1.0, 0, 0, 0, qPrintable(QString("Deviation magnification factor: ") + magnifyStr), "STANDARD", 0.0), //DGM: warning, toStdString doesn't preserve "local" characters
				DefaultLegendMaterial);

			//next line
			yLegend += c_textHeight_mm*2.0;

			//units
			dxf.writeText(*dw,
				DL_TextData(xLegend, yLegend, 0.0, xLegend, yLegend, 0.0, c_textHeight_mm, 1.0, 0, 0, 0, qPrintable(QString("Deviation units: ") + params.scaledDevUnits), "STANDARD", 0.0),
				DefaultLegendMaterial);

			//next line
			yLegend += c_textHeight_mm*2.0;

			//true profile line (red)
			dxf.writeLine(	*dw,
							DL_LineData(xLegend, yLegend, 0, xLegend + legendWidth_mm, yLegend, 0.0),
							DL_Attributes(LEGEND_LAYER, DL_Codes::green, -1, "BYLAYER", 1.0));

			dxf.writeText(	*dw,
							DL_TextData(xLegend + legendWidth_mm + c_textMargin_mm, yLegend, 0.0, xLegend + legendWidth_mm + c_textMargin_mm, yLegend, 0.0, c_textHeight_mm, 1.0, 0, 0, 0, qPrintable(params.legendRealProfileTitle), "STANDARD", 0.0), //DGM: warning, toStdString doesn't preserve "local" characters
							DefaultLegendMaterial);

			//next line
			yLegend += c_textHeight_mm*2.0;

			//theoretical profile line (red)
			dxf.writeLine(	*dw,
							DL_LineData(xLegend, yLegend, 0, xLegend + legendWidth_mm, yLegend, 0.0),
							DL_Attributes(LEGEND_LAYER, DL_Codes::red, -1, "BYLAYER", 1.0));

			dxf.writeText(	*dw,
							DL_TextData(xLegend + legendWidth_mm + c_textMargin_mm, yLegend, 0.0, xLegend + legendWidth_mm + c_textMargin_mm, yLegend, 0.0, c_textHeight_mm, 1.0, 0, 0, 0, qPrintable(params.legendTheoProfileTitle), "STANDARD", 0.0), //DGM: warning, toStdString doesn't preserve "local" characters
							DefaultLegendMaterial);
		}

		//write vertical profiles
		for (unsigned angleStep = 0; angleStep < angularStepCount; ++angleStep)
		{			
			std::vector<VertStepData> polySteps;
			try
			{
				polySteps.reserve(map->ySteps);
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory
				dw->dxfEOF();
				dw->close();
				delete dw;
				return false;
			}

			unsigned iMap = static_cast<unsigned>(static_cast<double>(angleStep * map->xSteps) / static_cast<double>(angularStepCount));
			for (unsigned jMap = 0; jMap < map->ySteps; ++jMap)
			{
				const DistanceMapGenerationTool::MapCell& cell = map->at(iMap + jMap * map->xSteps);

				VertStepData step;
				step.height = map->yMin + static_cast<double>(jMap) * map->yStep;

				if (step.height >= yMin && step.height <= yMax)
				{
					//find corresponding radius
					bool found = false;
					for (unsigned i = 1; i < profile->size(); ++i)
					{
						const CCVector3* A = profile->getPoint(i - 1);
						const CCVector3* B = profile->getPoint(i);

						double alpha = static_cast<double>((step.height - A->y - heightShift) / (B->y - A->y));
						if (alpha >= 0.0 && alpha <= 1.0)
						{
							//we deduce the right radius by linear interpolation
							step.radius_th = A->x + alpha * (B->x - A->x);
							found = true;
							break;
						}
					}

					if (found)
					{
						step.deviation = cell.count ? cell.value : 0.0;
						polySteps.push_back(step);
					}
				}
			}	

			const DL_Attributes DefaultMaterial(qPrintable(profileNames[angleStep]), DL_Codes::bylayer, -1, "BYLAYER", 1.0); //DGM: warning, toStdString doesn't preserve "local" characters
			const DL_Attributes GrayMaterial   (qPrintable(profileNames[angleStep]), DL_Codes::l_gray , -1, "", 1.0);

			//write layer title
			if (static_cast<int>(angleStep) < params.profileTitles.size())
			{
				const QString& title = params.profileTitles[angleStep];

				CCVector3d Ptop(c_pageWidth_mm / 2.0, y0 + ySpan * scale + c_profileMargin_mm / 2.0, 0.0);

				dxf.writeText(	*dw,
								DL_TextData(Ptop.x, Ptop.y, Ptop.z, Ptop.x, Ptop.y, Ptop.z, c_textHeight_mm, 1.0, 0, 1, 0, qPrintable(title), "STANDARD", 0.0), //DGM: warning, toStdString doesn't preserve "local" characters
								GrayMaterial);

			}
			
			//write corresponding polyline
			{
				dxf.writePolyline(	*dw,
									DL_PolylineData(static_cast<int>(polySteps.size()), 0, 0, 0),
									DefaultMaterial);

				for (size_t i = 0; i < polySteps.size(); ++i)
				{
					const VertStepData& step = polySteps[i];
					dxf.writeVertex(*dw, DL_VertexData(	x0 + (step.radius_th + step.deviation * params.devMagnifyCoef - xMin) * scale,
														y0 + (step.height - yMin) * scale, 0.0));
				}

				dxf.writePolylineEnd(*dw);
			}

			//write corresponding 'deviation bars' and labels
			CCVector3d pageShift(x0 - xMin * scale, y0 - yMin * scale, 0.0);
			{
				size_t lastStep = 0;
				for (size_t i = 0; i < polySteps.size(); ++i)
				{
					const VertStepData& step = polySteps[i];
					bool displayIt = (i == 0 || i + 1 == polySteps.size());
					if (!displayIt)
					{
						double dh = polySteps[i].height - polySteps[lastStep].height;
						double next_dh = polySteps[i + 1].height - polySteps[lastStep].height;
						if (dh >= heightStep || (next_dh > heightStep && fabs(dh - heightStep) < fabs(next_dh - heightStep)))
						{
							displayIt = true;
						}
					}

					if (displayIt)
					{
						CCVector3d Pheight(step.radius_th,step.height,0.0);
						CCVector3d Pdev(step.radius_th + step.deviation * params.devMagnifyCoef, step.height, 0.0);

						//page scaling
						Pheight = pageShift + Pheight * scale;
						Pdev = pageShift + Pdev * scale;

						//deviation bar
						dxf.writeLine(	*dw,
										DL_LineData(Pheight.x, Pheight.y, Pheight.z, Pdev.x, Pdev.y, Pdev.z),
										GrayMaterial);

						//labels

						//Horizontal justification: 0 = Left , 1 = Center, 2 = Right
						int hJustification = 0;
						//Vertical justification: 0 = Baseline, 1 = Bottom, 2 = Middle, 3 = Top
						int vJustification = 2;

						if (step.deviation < 0.0)
						{
							//deviation label on the left
							Pdev.x -= 2.0*c_textMargin_mm;
							//opposite for the height label
							Pheight.x += c_textMargin_mm;
							hJustification = 2; //Right
						}
						else
						{
							//deviation label on the right
							Pdev.x += 2.0*c_textMargin_mm;
							//opposite for the height label
							Pheight.x -= c_textMargin_mm;
							hJustification = 0; //Left
						}

						QString devText = QString::number(polySteps[i].deviation * params.devLabelMultCoef,'f',params.precision);
						dxf.writeText(	*dw,
										DL_TextData(Pdev.x, Pdev.y, Pdev.z, Pdev.x, Pdev.y, Pdev.z, c_textHeight_mm, 1.0, 0, hJustification, vJustification, qPrintable(devText), "STANDARD", 0.0), //DGM: warning, toStdString doesn't preserve "local" characters
										DefaultMaterial);

						QString heightText = QString::number(polySteps[i].height,'f',params.precision);
						dxf.writeText(	*dw,
										DL_TextData(Pheight.x, Pheight.y, Pheight.z, Pheight.x, Pheight.y, Pheight.z, c_textHeight_mm, 1.0, 0, 2 - hJustification, vJustification, qPrintable(heightText), "STANDARD", 0.0),
										GrayMaterial);
						
						lastStep = i;
					}
				}
			}
		}

		dw->sectionEnd();

	}

	//Writing the Objects Section
	dxf.writeObjects(*dw);
	dxf.writeObjectsEnd(*dw);

	//Ending and Closing the File
	dw->dxfEOF();
	dw->close();
	delete dw;
	dw = nullptr;

	return true;

#else
	return false;
#endif
}

struct HorizStepData
{
	double angle_rad;
	double deviation;
};

bool DxfProfilesExporter::SaveHorizontalProfiles(	const QSharedPointer<DistanceMapGenerationTool::Map>& map,
													ccPolyline* profile,
													QString filename,
													unsigned heightStepCount,
													double heightShift,
													double angularStep_rad,
													double radToUnitConvFactor,
													QString angleUnit,
													const Parameters& params,
													ccMainAppInterface* app/*= 0*/)
{
#ifdef CC_DXF_SUPPORT
	assert(c_pageMargin_mm < c_profileMargin_mm);
	assert(2.0*c_profileMargin_mm < std::min(c_pageWidth_mm, c_pageHeight_mm));

	if (!map || !profile || heightStepCount == 0 || angularStep_rad <= 0)
	{
		//invalid parameters
		return false;
	}

	//Theoretical profile bounding box
	CCVector3 profileBBMin;
	CCVector3 profileBBMax;
	profile->getAssociatedCloud()->getBoundingBox(profileBBMin, profileBBMax);
	//Mix with the map's boundaries along 'Y'
	double yMin = std::max(	map->yMin + 0.5 * map->xStep, //central height of first row
							static_cast<double>(profileBBMin.y) + heightShift);
	double yMax = std::min(	map->yMin + (static_cast<double>(map->ySteps) - 0.5) * map->yStep, //central height of last row
							static_cast<double>(profileBBMax.y) + heightShift);
	const double ySpan = yMax - yMin;

	//For the 'X' dimension, it's easier to stick with the th. profile
//	const double xMin = profileBBMin.x;
	const double xMax = profileBBMax.x;
	//shortcut for clarity
	const double& maxRadius = xMax;

	if (ySpan == 0.0)
	{
		if (app)
			app->dispToConsole(QString("Internal error: null profile?!"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	DL_Dxf dxf;
	DL_WriterA* dw = dxf.out(qPrintable(filename), DL_VERSION_R12);
	if (!dw)
	{
		if (app)
			app->dispToConsole(QString("Failed to open '%1' file for writing!").arg(filename),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	//write header
	dxf.writeHeader(*dw);

	//add dimensions
	dw->dxfString(9, "$INSBASE");
	dw->dxfReal(10,0.0);
	dw->dxfReal(20,0.0);
	dw->dxfReal(30,0.0);
	dw->dxfString(9, "$EXTMIN");
	dw->dxfReal(10,0.0);
	dw->dxfReal(20,0.0);
	dw->dxfReal(30,0.0);
	dw->dxfString(9, "$EXTMAX");
	dw->dxfReal(10,c_pageWidth_mm);
	dw->dxfReal(20,c_pageHeight_mm);
	dw->dxfReal(30,0.0);
	dw->dxfString(9, "$LIMMIN");
	dw->dxfReal(10,0.0);
	dw->dxfReal(20,0.0);
	dw->dxfString(9, "$LIMMAX");
	dw->dxfReal(10,c_pageWidth_mm);
	dw->dxfReal(20,c_pageHeight_mm);

	//close header
	dw->sectionEnd();

	//Opening the Tables Section
	dw->sectionTables();
	//Writing the Viewports
	dxf.writeVPort(*dw);

	//Writing the Linetypes (all by default)
	{
		dw->tableLinetypes(3);
		dxf.writeLinetype(*dw, DL_LinetypeData("BYBLOCK", "BYBLOCK", 0, 0, 0.0));
		dxf.writeLinetype(*dw, DL_LinetypeData("BYLAYER", "BYLAYER", 0, 0, 0.0));
		dxf.writeLinetype(*dw, DL_LinetypeData("CONTINUOUS", "Continuous", 0, 0, 0.0));
		dw->tableEnd();
	}

	//Writing the Layers
	dw->tableLayers(heightStepCount+2);
	QStringList profileNames;
	{
		//default layer
		dxf.writeLayer(*dw, 
			DL_LayerData("0", 0), 
			DL_Attributes(
			std::string(""),		// leave empty
			DL_Codes::black,		// default color
			100,					// default width (in 1/100 mm)
			"CONTINUOUS",			// default line style
			1.0						// linetypeScale
			));

		//legend layer
		dxf.writeLayer(*dw, 
			DL_LayerData(LEGEND_LAYER, 0), 
			DL_Attributes(
			std::string(""),
			DL_Codes::black,
			s_lineWidth,
			"CONTINUOUS",
			1.0));

		//horiz. profile layers
		for (unsigned i=0; i<heightStepCount; ++i)
		{
			//default profile name
			QString layerName = QString(HORIZ_PROFILE_LAYER).arg(i + 1, 3, 10, QChar('0'));
			//but we use the profile title if we have one!
			//DGM: nope, as it may be longer than 31 characters (R14 limit)
			//if (params.profileTitles.size() == 1)
			//{
			//	//profile height
			//	double height = yMin + static_cast<double>(i) / static_cast<double>(heightStepCount-1) * ySpan;
			//	layerName = QString(params.profileTitles[0]).arg(height,0,'f',params.precision);
			//	layerName.replace(QChar(' '),QChar('_'));
			//}

			profileNames << layerName;
			dxf.writeLayer(*dw, 
				DL_LayerData(qPrintable(layerName), 0), //DGM: warning, toStdString doesn't preserve "local" characters
				DL_Attributes(
				std::string(""),
				i == 0 ? DL_Codes::green : -DL_Codes::green, //invisible if negative!
				s_lineWidth,
				"CONTINUOUS",
				1.0));
		}
	}
	dw->tableEnd();

	//Writing Various Other Tables
	//dxf.writeStyle(*dw); //DXFLIB V2.5
	dw->tableStyle(1);
	dxf.writeStyle(*dw, DL_StyleData("Standard", 0, 0.0, 0.75, 0.0, 0, 2.5, "txt", "")); //DXFLIB V3.3
	dw->tableEnd();

	dxf.writeView(*dw);
	dxf.writeUcs(*dw);

	dw->tableAppid(1);
	dw->tableAppidEntry(0x12);
	dw->dxfString(2, "ACAD");
	dw->dxfInt(70, 0);
	dw->tableEnd();

	//Writing Dimension Styles
	dxf.writeDimStyle(	*dw, 
						/*arrowSize*/1, 
						/*extensionLineExtension*/1,
						/*extensionLineOffset*/1,
						/*dimensionGap*/1,
						/*dimensionTextSize*/1);
	
	//Writing Block Records
	dxf.writeBlockRecord(*dw);
	dw->tableEnd();

	//Ending the Tables Section
	dw->sectionEnd();

	//Writing the Blocks Section
	{
		dw->sectionBlocks();

		dxf.writeBlock(*dw,  DL_BlockData("*Model_Space", 0, 0.0, 0.0, 0.0));
		dxf.writeEndBlock(*dw, "*Model_Space");

		dxf.writeBlock(*dw, DL_BlockData("*Paper_Space", 0, 0.0, 0.0, 0.0));
		dxf.writeEndBlock(*dw, "*Paper_Space");

		dxf.writeBlock(*dw, DL_BlockData("*Paper_Space0", 0, 0.0, 0.0, 0.0));
		dxf.writeEndBlock(*dw, "*Paper_Space0");

		dw->sectionEnd();
	}

	//Writing the Entities Section
	{
		dw->sectionEntities();

		//we make the profile fit in the middle of the page (21.0 x 29.7 cm)
		double scale = std::min((c_pageWidth_mm - 2.0 * c_profileMargin_mm)  / (2.0 * maxRadius),
								(c_pageHeight_mm - 2.0 * c_profileMargin_mm) / (2.0 * maxRadius));

		//min corner of profile area
		//const double x0 = (c_pageWidth_mm  - 2.0 * maxRadius*scale) / 2.0;
		const double y0 = (c_pageHeight_mm - 2.0 * maxRadius*scale) / 2.0;
		//center of profile area
		const double xc = c_pageWidth_mm / 2.0;
		const double yc = c_pageHeight_mm / 2.0;

		//write legend
		{
			DL_Attributes DefaultLegendMaterial(LEGEND_LAYER, DL_Codes::bylayer, -1, "BYLAYER", 1.0);

			//write page contour
			{
				dxf.writePolyline(	*dw,
									DL_PolylineData(4, 0, 0, 1),
									DefaultLegendMaterial);

				dxf.writeVertex(*dw, DL_VertexData(	c_pageMargin_mm, c_pageMargin_mm, 0.0));
				dxf.writeVertex(*dw, DL_VertexData(	c_pageMargin_mm, c_pageHeight_mm-c_pageMargin_mm, 0.0));
				dxf.writeVertex(*dw, DL_VertexData(	c_pageWidth_mm-c_pageMargin_mm, c_pageHeight_mm-c_pageMargin_mm, 0.0));
				dxf.writeVertex(*dw, DL_VertexData(	c_pageWidth_mm-c_pageMargin_mm, c_pageMargin_mm, 0.0));

				dxf.writePolylineEnd(*dw);
			}

			double xLegend = c_pageMargin_mm + 2.0*c_textHeight_mm;
			double yLegend = c_pageMargin_mm + 2.0*c_textHeight_mm;
			const double legendWidth_mm = 20.0;

			//Y axis
			double axisTip = maxRadius*scale + 5.0;
			dxf.writeLine(	*dw,
							DL_LineData(xc, yc, 0.0, xc, yc-axisTip, 0.0),
							DefaultLegendMaterial);

			//Y axis tip as triangle
			{
				double axisTipSize = 3.0;

				dxf.writePolyline(	*dw,
									DL_PolylineData(3, 0, 0, 1), //closed polyline!
									DefaultLegendMaterial);
				dxf.writeVertex(*dw, DL_VertexData(xc, yc - (axisTip + axisTipSize), 0.0));
				dxf.writeVertex(*dw, DL_VertexData(xc - axisTipSize / 2.0, yc - axisTip, 0.0));
				dxf.writeVertex(*dw, DL_VertexData(xc + axisTipSize / 2.0, yc - axisTip, 0.0));
				dxf.writePolylineEnd(*dw);

				dxf.writeText(	*dw,
								DL_TextData(xc, yc - (axisTip + 2.0*axisTipSize), 0.0, xc, yc - (axisTip + 2.0*axisTipSize), 0.0, c_textHeight_mm, 1.0, 0, 1, 3, "Y", "STANDARD", 0.0),
								DefaultLegendMaterial);
			}
			
			//deviation magnification factor
			QString magnifyStr = QString::number(params.devMagnifyCoef);
			dxf.writeText(	*dw,
							DL_TextData(xLegend, yLegend, 0.0, xLegend, yLegend, 0.0, c_textHeight_mm, 1.0, 0, 0, 0, qPrintable(QString("Deviation magnification factor: ") + magnifyStr), "STANDARD", 0.0), //DGM: warning, toStdString doesn't preserve "local" characters
							DefaultLegendMaterial);

			//next line
			yLegend += c_textHeight_mm*2.0;

			//units
			dxf.writeText(	*dw,
							DL_TextData(xLegend, yLegend, 0.0, xLegend, yLegend, 0.0, c_textHeight_mm, 1.0, 0, 0, 0, qPrintable(QString("Deviation units: ") + params.scaledDevUnits), "STANDARD", 0.0), //DGM: warning, toStdString doesn't preserve "local" characters
							DefaultLegendMaterial);

			//next line
			yLegend += c_textHeight_mm*2.0;

			//true profile line (red)
			dxf.writeLine(	*dw,
							DL_LineData(xLegend, yLegend, 0, xLegend + legendWidth_mm, yLegend, 0.0),
							DL_Attributes(LEGEND_LAYER, DL_Codes::green, -1, "BYLAYER", 1.0));

			dxf.writeText(	*dw,
							DL_TextData(xLegend + legendWidth_mm + c_textMargin_mm, yLegend, 0.0, xLegend + legendWidth_mm + c_textMargin_mm, yLegend, 0.0, c_textHeight_mm, 1.0, 0, 0, 0, qPrintable(params.legendRealProfileTitle), "STANDARD", 0.0),
							DefaultLegendMaterial);

			//next line
			yLegend += c_textHeight_mm*2.0;

			//theoretical profile line (red)
			dxf.writeLine(	*dw,
							DL_LineData(xLegend, yLegend, 0, xLegend + legendWidth_mm, yLegend, 0.0),
							DL_Attributes(LEGEND_LAYER, DL_Codes::red, -1, "BYLAYER", 1.0));

			dxf.writeText(	*dw,
							DL_TextData(xLegend + legendWidth_mm + c_textMargin_mm, yLegend, 0.0, xLegend + legendWidth_mm + c_textMargin_mm, yLegend, 0.0, c_textHeight_mm, 1.0, 0, 0, 0, qPrintable(params.legendTheoProfileTitle), "STANDARD", 0.0),
							DefaultLegendMaterial);
		}

		//profile values (fixed size: one per angular step of the input grid)
		std::vector<HorizStepData> polySteps;
		try
		{
			polySteps.resize(map->xSteps);
		}
		catch (const std::bad_alloc&)
		{
			//not engouh memory
			dw->dxfEOF();
			dw->close();
			delete dw;
			return false;
		}

		//write horizontal profiles
		for (unsigned heightStep = 0; heightStep < heightStepCount; ++heightStep)
		{			
			//profile height
			double height = yMin + static_cast<double>(heightStep) / static_cast<double>(heightStepCount-1) * ySpan;

			//corresponding index in map
			if (height < map->yMin || height >= map->yMin + static_cast<double>(map->ySteps) * map->yStep)
			{
				assert(false); //we have computed yMin and yMax so that those values are totally included inside the map's boundaries...
				continue;
			}
			unsigned jMap = static_cast<unsigned>((height - map->yMin) / map->yStep);
			assert(jMap < map->ySteps);

			//find corresponding radius
			double currentRadius = 0.0;
			{
				bool found = false;
				for (unsigned i = 1; i < profile->size(); ++i)
				{
					const CCVector3* A = profile->getPoint(i - 1);
					const CCVector3* B = profile->getPoint(i);

					double alpha = static_cast<double>((height - A->y - heightShift) / (B->y - A->y));
					if (alpha >= 0.0 && alpha <= 1.0)
					{
						//we deduce the right radius by linear interpolation
						currentRadius = A->x + alpha * (B->x - A->x);
						found = true;
						break;
					}
				}

				if (!found)
				{
					assert(false); //we have computed yMin and yMax so that 'height' is totally included inside the profile's boundaries...
					continue;
				}
			}

			const QString& currentLayer = profileNames[heightStep];
			const DL_Attributes DefaultMaterial(qPrintable(currentLayer), DL_Codes::bylayer, -1, "BYLAYER", 1.0); //DGM: warning, toStdString doesn't preserve "local" characters
			const DL_Attributes GrayMaterial   (qPrintable(currentLayer), DL_Codes::l_gray , -1, "", 1.0);

			//write layer title
			if (params.profileTitles.size() == 1)
			{
				QString title = QString(params.profileTitles[0]).arg(height,0,'f',params.precision);

				CCVector3d Ptop(xc, y0 + 2.0 * maxRadius * scale + c_profileMargin_mm / 2.0, 0.0);

				dxf.writeText(	*dw,
					DL_TextData(Ptop.x, Ptop.y, Ptop.z, Ptop.x, Ptop.y, Ptop.z, c_textHeight_mm, 1.0, 0, 1, 0, qPrintable(title), "STANDARD", 0.0), //DGM: warning, toStdString doesn't preserve "local" characters
					GrayMaterial);
			}

			//write theoretical profile (polyline = circle)
			{
				dxf.writeCircle(*dw,
									DL_CircleData(xc, yc, 0.0, currentRadius*scale),
									DL_Attributes(qPrintable(currentLayer), DL_Codes::red, -1, "BYLAYER", 1.0));
			}

			assert(polySteps.size() == map->xSteps);
			{
				const DistanceMapGenerationTool::MapCell* cell = &map->at(jMap * map->xSteps);
				for (unsigned iMap=0; iMap<map->xSteps; ++iMap, ++cell)
				{
					HorizStepData step;
					step.angle_rad = 2.0*M_PI * static_cast<double>(iMap) / static_cast<double>(map->xSteps);
					step.deviation = cell->count ? cell->value : 0.0;
					polySteps[iMap] = step;
				}
			}

			//profile "direction"
			double cwSign = map->counterclockwise ? -1.0 : 1.0;

			CCVector3d pageShift(xc, yc, 0.0);

			//write profile polyline
			{
				dxf.writePolyline(	*dw,
									DL_PolylineData(static_cast<int>(polySteps.size()), 0, 0, 1), //closed shape!
									DefaultMaterial);

				for (size_t i = 0; i < polySteps.size(); ++i)
				{
					const HorizStepData& step = polySteps[i];
					double radius = currentRadius + step.deviation * params.devMagnifyCoef;
					dxf.writeVertex(*dw, DL_VertexData(	pageShift.x - (cwSign * radius * sin(step.angle_rad)) * scale,
														pageShift.y - (radius * cos(step.angle_rad)) * scale,
														0.0));
				}

				dxf.writePolylineEnd(*dw);
			}

			//write corresponding 'deviation bars' and labels
			{
				size_t lastStep = 0;
				for (size_t i = 0; i < polySteps.size(); ++i)
				{
					const HorizStepData& step = polySteps[i];
					bool displayIt = (i == 0/*|| i+1 == polySteps.size()*/); //warning: cycle
					if (i != 0)
					{
						double dAngle = polySteps[i].angle_rad - polySteps[lastStep].angle_rad;
						double next_dAngle = (i+1 == polySteps.size() ? polySteps[0].angle_rad + 2.0*M_PI : polySteps[i+1].angle_rad) - polySteps[lastStep].angle_rad;
						if (dAngle >= angularStep_rad || (next_dAngle > angularStep_rad && fabs(dAngle - angularStep_rad) < fabs(next_dAngle - angularStep_rad)))
						{
							displayIt = true;
						}
					}

					if (displayIt && i != 0) //we skip 0 as we always display a vertical axis!
					{
						CCVector3d relativePos( -cwSign * sin(step.angle_rad),
												-cos(step.angle_rad),
												0.0);
						CCVector3d Pangle = relativePos * currentRadius;
						CCVector3d Pdev = relativePos * (currentRadius + step.deviation * params.devMagnifyCoef);

						//page scaling
						Pangle = pageShift + Pangle * scale;
						Pdev = pageShift + Pdev * scale;

						//deviation bar
						dxf.writeLine(	*dw,
										DL_LineData(Pangle.x, Pangle.y, Pangle.z, Pdev.x, Pdev.y, Pdev.z),
										GrayMaterial);

						//labels

						//justification depends on the current angle
						const double c_angleMargin_rad = 5.0 / 180.0 * M_PI; //margin: 5 degrees
						const double c_margin = sin(c_angleMargin_rad);

						//Horizontal justification: 0 = Left , 1 = Center, 2 = Right
						int hJustification = 1;
						if (relativePos.x < -c_margin) //point on the left
							hJustification = 2; //Right
						else if (relativePos.x > c_margin) //point on the right
							hJustification = 0; //Left

						//Vertical justification: 0 = Baseline, 1 = Bottom, 2 = Middle, 3 = Top
						int vJustification = 2;
						if (relativePos.y < -c_margin) //point in the lower part
							vJustification = 3; //Top
						else if (relativePos.y > c_margin) //point in the upper part
							vJustification = 1; //Bottom

						int hJustificationDev = hJustification;
						int hJustificationAng = hJustification;
						int vJustificationDev = vJustification;
						int vJustificationAng = vJustification;

						//negative deviation: profile is inside the circle
						// - deviation is displayed insde
						// - angle is displayed outside
						if (step.deviation < 0.0)
						{
							Pdev -= relativePos * 2.0*c_textMargin_mm;
							Pangle += relativePos * c_textMargin_mm;
							//invert dev. text justification
							if (hJustificationDev != 1)
								hJustificationDev = 2 - hJustificationDev; // 0 <-> 2
							if (vJustificationDev != 2)
								vJustificationDev = 4 - vJustificationDev; // 1 <-> 3
						}
						else
						{
							Pdev += relativePos * 2.0*c_textMargin_mm;
							Pangle -= relativePos * c_textMargin_mm;
							//invert ang. text justification
							if (hJustificationAng != 1)
								hJustificationAng = 2 - hJustificationAng; // 0 <-> 2
							if (vJustificationAng != 2)
								vJustificationAng = 4 - vJustificationAng; // 1 <-> 3
						}

						QString devText = QString::number(polySteps[i].deviation * params.devLabelMultCoef, 'f', params.precision);
						dxf.writeText(	*dw,
										DL_TextData(Pdev.x, Pdev.y, Pdev.z, Pdev.x, Pdev.y, Pdev.z, c_textHeight_mm, 1.0, 0, hJustificationDev, vJustificationDev, qPrintable(devText), "STANDARD", 0.0),
										DefaultMaterial);

						QString angleText = QString::number(polySteps[i].angle_rad *radToUnitConvFactor, 'f', params.precision) + angleUnit;
						dxf.writeText(	*dw,
										DL_TextData(Pangle.x, Pangle.y, Pangle.z, Pangle.x, Pangle.y, Pangle.z, c_textHeight_mm, 1.0, 0, hJustificationAng, vJustificationAng, qPrintable(angleText), "STANDARD", 0.0),
										GrayMaterial);
						
						lastStep = i;
					}
				}
			}
		}

		dw->sectionEnd();

	}

	//Writing the Objects Section
	dxf.writeObjects(*dw);
	dxf.writeObjectsEnd(*dw);

	//Ending and Closing the File
	dw->dxfEOF();
	dw->close();
	delete dw;
	dw = nullptr;

	return true;

#else
	return false;
#endif
}
