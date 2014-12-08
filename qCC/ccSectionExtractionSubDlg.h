//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_SECTION_EXTRACTION_SUB_DIALOG_HEADER
#define CC_SECTION_EXTRACTION_SUB_DIALOG_HEADER

//Qt
#include <QDialog>

#include <ui_sectionExtractionSubDlg.h>

//! Dialog for generating sections along one or several 2D polylines (Section Extraction Tool)
class ccSectionExtractionSubDlg : public QDialog, public Ui::SectionExtractionSubDlg
{
public:

	//! Default constructor
	ccSectionExtractionSubDlg(QWidget* parent = 0);

	//! Sets the default section thickness
	void setSectionThickness(double t);
	//! Returns the section thickness
	double getSectionThickness() const;

	//! Returns the max edge length (for contour generation)
	double getMaxEdgeLength() const;
	//! Sets the max edge length (for contour generation)
	void setMaxEdgeLength(double l);
	
	//! Contour type
	enum ContourType { LOWER, UPPER, FULL };
	//! Returns the contour type (for contour generation)
	ContourType getContourType() const;

	//! Whether to generate clouds or not
	bool extractClouds() const;
	//! Sets whether to generate clouds or not
	void doExtractClouds(bool state);
	//! Whether to generate contours or not
	bool extractContours() const;
	//! Sets whether to generate contours or not
	void doExtractContours(bool state);

};

#endif // CC_SECTION_EXTRACTION_SUB_DIALOG_HEADER
