//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_SECTION_EXTRACTION_SUB_DIALOG_HEADER
#define CC_SECTION_EXTRACTION_SUB_DIALOG_HEADER

//Local
#include "ccContourExtractor.h"

//Qt
#include <QDialog>

#include <ui_sectionExtractionSubDlg.h>

//! Dialog for generating sections along one or several 2D polylines (Section Extraction Tool)
class ccSectionExtractionSubDlg : public QDialog, public Ui::SectionExtractionSubDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccSectionExtractionSubDlg(QWidget* parent = 0);

	//! Sets the number of active section(s)
	void setActiveSectionCount(int count);

	//! Sets the default section thickness
	void setSectionThickness(double t);
	//! Returns the section thickness
	double getSectionThickness() const;

	//! Returns the max edge length (for contour generation)
	double getMaxEdgeLength() const;
	//! Sets the max edge length (for contour generation)
	void setMaxEdgeLength(double l);
	
	//! Returns the contour type (for contour generation)
	ccContourExtractor::ContourType getContourType() const;

	//! Whether to generate clouds or not
	bool extractClouds() const;
	//! Sets whether to generate clouds or not
	void doExtractClouds(bool state);
	//! Whether to generate contours or not
	bool extractContours() const;
	//! Sets whether to generate contours or not
	void doExtractContours(bool state, ccContourExtractor::ContourType type);

	//! Whether to split the contours or not
	bool splitContours() const;
	//! Sets whether to split the contours or not
	void doSplitContours(bool state);

	//! Whether to use multipass or not
	bool useMultiPass() const;
	//! Sets whether to use multipass or not
	void doUseMultiPass(bool state);
	

	//! Whether visual debug mode is enabled or not
	bool visualDebugMode() const;

};

#endif // CC_SECTION_EXTRACTION_SUB_DIALOG_HEADER
