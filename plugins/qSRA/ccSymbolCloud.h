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

#ifndef QSRA_SYMBOL_CLOUD_HEADER
#define QSRA_SYMBOL_CLOUD_HEADER

//Qt
#include <QString>

//qCC_db
#include <ccPointCloud.h>

class ccSymbolCloud : public ccPointCloud
{
public:

	//! Default constructor
	ccSymbolCloud(QString name = QString());

	//! Reserves memory for storing per-point labels
	bool reserveLabelArray(unsigned count);

	//! Adds a label
	/** Label array should have been already 'reserved' (see reserveLabelArray).
		There should be as many labels as points at the end... (however labels
		can be empty)
	**/
	void addLabel(QString label);

	//! Resizes memory for storing per-point labels
	bool resizeLabelArray(unsigned count);

	//! Sets a given label
	/** Label array should have been already resized (or filled).
	**/
	void setLabel(unsigned index, QString label);

	//! Returns a given label
	QString getLabel(unsigned index) const;

	//! Clears the label array
	void clearLabelArray();

	//! inherited from ccPointCloud
	virtual bool reserve(unsigned numberOfPoints) override;
	virtual bool resize(unsigned numberOfPoints) override;
	virtual void clear() override;

	//! Sets symbol size
	void setSymbolSize(double size) { m_symbolSize = size; }

	//! Returns symbol size
	double getSymbolSize() const { return m_symbolSize; }

	//! Sets label font size
	void setFontSize(int size) { m_fontSize = size; }

	//! Returns label font size
	int getFontSize() const { return m_fontSize; }

	//! Sets whether symbols should be displayed or not
	void showSymbols(bool state) { m_showSymbols = state; }
	//! Returns whether symbols are displayed or not
	bool symbolsShown() const { return m_showSymbols; }

	//! Sets whether labels should be displayed or not
	void showLabels(bool state) { m_showLabels = state; }
	//! Returns whether labels are displayed or not
	bool labelsShown() const { return m_showLabels; }

	//! Sets labels alignment flags
	/** See ccGenericGLDisplay::TextAlign
	**/
	void setLabelAlignmentFlags(unsigned char flags) { m_labelAlignFlags = flags; }

protected:

	//inherited from ccPointCloud
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;

	//! Labels array
	std::vector<QString> m_labels;

	//! Symbol size (in pixels)
	double m_symbolSize;

	//! Label font size (in points)
	int m_fontSize;

	//! Whether symbols are shown or not
	bool m_showSymbols;

	//! Whether labels are shown or not
	bool m_showLabels;

	//! Default label alignment flags
	unsigned char m_labelAlignFlags;

	//! Last 3D rendering parameters
	ccGLCameraParameters m_lastCameraParams;
};

#endif //QSRA_SYMBOL_CLOUD_HEADER
