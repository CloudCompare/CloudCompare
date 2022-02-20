#pragma once

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

//Local
#include "ccEnvelopeExtractor.h"

//Qt
#include <QDialog>

#include <ui_sectionExtractionSubDlg.h>

//! Dialog for generating sections along one or several 2D polylines (Section Extraction Tool)
class ccSectionExtractionSubDlg : public QDialog, public Ui::SectionExtractionSubDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccSectionExtractionSubDlg(QWidget* parent = nullptr);

	//! Sets the number of active section(s)
	void setActiveSectionCount(int count);

	//! Sets the default section thickness
	void setSectionThickness(double t);
	//! Returns the section thickness
	double getSectionThickness() const;

	//! Returns the max edge length (for envelope generation)
	double getMaxEdgeLength() const;
	//! Sets the max edge length (for envelope generation)
	void setMaxEdgeLength(double l);
	
	//! Returns the envelope type (for envelope generation)
	ccEnvelopeExtractor::EnvelopeType getEnvelopeType() const;

	//! Whether to generate clouds or not
	bool extractClouds() const;
	//! Sets whether to generate clouds or not
	void doExtractClouds(bool state);
	//! Whether to generate envelopes or not
	bool extractEnvelopes() const;
	//! Sets whether to generate envelopes or not
	void doExtractEnvelopes(bool state, ccEnvelopeExtractor::EnvelopeType type);

	//! Whether to split the envelopes or not
	bool splitEnvelopes() const;
	//! Sets whether to split the envelopes or not
	void doSplitEnvelopes(bool state);

	//! Whether to use multipass or not
	bool useMultiPass() const;
	//! Sets whether to use multipass or not
	void doUseMultiPass(bool state);
	

	//! Whether visual debug mode is enabled or not
	bool visualDebugMode() const;
};
