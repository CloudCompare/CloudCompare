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

#ifndef QFACET_STEREOGRAM_DIALOG_HEADER
#define QFACET_STEREOGRAM_DIALOG_HEADER

#include "ui_stereogramDlg.h"
#include "ui_stereogramParamsDlg.h"

//qCC_db
#include <ccColorScale.h>
#include <ccPlane.h>

// Qt
#include <QDialog>

//system
#include <utility>

class ccHObject;
class FacetDensityGrid;
class ccColorScaleSelector;
class ccMainAppInterface;

//! Dialog for stereogram parameters (qFacets plugin)
class StereogramParamsDlg : public QDialog, public Ui::StereogramParamsDlg
{
public:

	//! Default constructor
	StereogramParamsDlg(QWidget* parent = nullptr)
		: QDialog(parent, Qt::Tool)
		, Ui::StereogramParamsDlg()
	{
		setupUi(this);
	}
};

//! Orientation-based classification widget
class StereogramWidget : public QLabel
{
	Q_OBJECT

public:

	//! Default constructor
	StereogramWidget(QWidget *parent = nullptr);

	//! Destructor
	virtual ~StereogramWidget();

	//! Sets current parameters
	bool init(	double angularStep_deg,
				ccHObject* facetGroup,
				double resolution_deg = 2.0);

	//! Returns the mean dip direction and dip
	void getMeanDir(double& meanDip_deg, double& meanDipDir_deg) { meanDip_deg = m_meanDip_deg; meanDipDir_deg = m_meanDipDir_deg; }

	//inherited from QWidget (to get a square widget!)
	int	heightForWidth (int w) const override { return w; }

	//! Sets density color scale
	void setDensityColorScale(ccColorScale::Shared colorScale) { m_densityColorScale = colorScale; }
	//! Returns density color scale
	ccColorScale::Shared getDensityColorScale() const { return m_densityColorScale; }
	//! Sets density color scale steps
	void setDensityColorScaleSteps(unsigned steps) { m_densityColorScaleSteps = steps; }
	//! Returns density color scale steps
	unsigned getDensityColorScaleSteps() const { return m_densityColorScaleSteps; }

	//! Sets the ticks frequency (0 = no ticks)
	void setTicksFreq(int freq) { m_ticksFreq = freq; }

	//! Whether to show the 'HSV' ring or not
	void showHSVRing(bool state) { m_showHSVRing = state; }

	//! Enables or not the mouse tracking mode
	void enableMouseTracking(bool state, double dipSpan_deg = 30.0, double dipDirSpan_deg = 30.0);

	//! Sets tracked center position
	void setTrackedCenter(double dip_deg, double dipDir_deg);

signals:

	//! Signal emitted when the mouse (left) button is clicked
	/** \param dip_deg dip angle (in degrees)
		\param dipDir_deg dip direction angle (in degrees)
	**/
	void pointClicked(double dip_deg, double dipDir_deg);

protected:

	//inherited from QWidget
	void paintEvent(QPaintEvent* e) override;
	void mousePressEvent(QMouseEvent* e) override;

	//! Angular step (in degrees)
	double m_angularStep_deg;

	//! Density grid
	FacetDensityGrid* m_densityGrid;

	//! Mean dip direction (in degrees)
	double m_meanDipDir_deg;
	//! Mean dip (in degrees)
	double m_meanDip_deg;

	//! Density color scale
	ccColorScale::Shared m_densityColorScale;
	//! Density color scale steps
	unsigned m_densityColorScaleSteps;

	//! Ticks frequency
	int m_ticksFreq;

	//! Whether to show the 'HSV' ring or not
	bool m_showHSVRing;

	//! Mouse tracking
	bool m_trackMouseClick;
	//! Last mouse click equivalent dip (in degrees)
	double m_clickDip_deg;
	//! Last mouse click equivalent dip direction (in degrees)
	double m_clickDipDir_deg;
	//! Click area span along dip (in degrees)
	double m_clickDipSpan_deg;
	//! Click area span along dip direction (in degrees)
	double m_clickDipDirSpan_deg;

	//! Stereogram center (pixels)
	QPoint m_center;
	//! Stereogram radius (pixels)
	int m_radius;
	
};

//! Dialog for displaying the angular repartition of facets (qFacets plugin)
class StereogramDialog : public QDialog, public Ui::StereogramDialog
{
	Q_OBJECT

public:

	//! Default constructor
	StereogramDialog(ccMainAppInterface* app);

	//! Inits dialog
	/** Warning: input 'facetGroup' should not be deleted before this dialog is closed!
	**/
	bool init(	double angularStep_deg,
				ccHObject* facetGroup,
				double resolution_deg = 2.0);

	//! Returns associated widget
	StereogramWidget* stereogram() { return m_classifWidget; }

protected slots:

	void colorScaleChanged(int);
	void spawnColorScaleEditor();
	void onTicksFreqChanged(int);
	void onHSVColorsToggled(bool);
	void onDensityColorStepsChanged(int);
	void onFilterEnabled(bool);
	void onPointClicked(double,double);
	void onFilterSizeChanged(double);
	void onFilterCenterChanged(double);
	void exportCurrentSelection();

protected:

	//inherited from QDialog
	void closeEvent(QCloseEvent* e);

	//! Changes the associated facets visibility based on the current filter parameters
	void updateFacetsFilter(bool enable);
	
	//! Associated widget
	StereogramWidget* m_classifWidget;

	//! Color scale selector/editor
	ccColorScaleSelector* m_colorScaleSelector;

	//! Main application interface
	ccMainAppInterface* m_app;

	//! Associated set of facets (unique ID)
	int m_facetGroupUniqueID;
};

#endif //QFACET_STEREOGRAM_DIALOG_HEADER
