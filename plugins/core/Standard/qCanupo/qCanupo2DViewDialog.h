//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qCANUPO                       #
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
//#      COPYRIGHT: UEB (UNIVERSITE EUROPEENNE DE BRETAGNE) / CNRS         #
//#                                                                        #
//##########################################################################

#ifndef Q_CANUPO_2DVIEW_DIALOG_HEADER
#define Q_CANUPO_2DVIEW_DIALOG_HEADER

#include <ui_qCanupo2DViewDialog.h>

//Local
#include "classifier.h"

//CCLib
#include <CCGeom.h>

class ccMainAppInterface;
class ccGLWindow;
class ccHObject;
class ccPointCloud;
class ccPolyline;

//! CANUPO plugin's 2D view dialog
class qCanupo2DViewDialog : public QDialog, public Ui::Canupo2DViewDialog
{
	Q_OBJECT

public:

	//! Default constructor
	qCanupo2DViewDialog(const CorePointDescSet* descriptors1,
						const CorePointDescSet* descriptors2,
						QString cloud1Name,
						QString cloud2Name,
						int class1 = 1,
						int class2 = 2,
						const CorePointDescSet* evaluationDescriptors = 0,
						ccMainAppInterface* app = 0);

	//! Destructor
	virtual ~qCanupo2DViewDialog();

	//! Sets picking radius (for polyline vertices)
	void setPickingRadius(int radius);

	//! Returns classifier
	const Classifier& getClassifier() { return m_classifier; }

public slots:

	//! Trains the classifier (with the current number of scales!)
	bool trainClassifier();

protected slots:

	//! Updates the boundary representation 
	void resetBoundary();

	//! Computes statistics with the current classifier
	void computeStatistics();

	void saveClassifier();
	void checkBeforeAccept();
	void setPointSize(int);
	void onScalesCountSpinBoxChanged(int);
	
	void addOrSelectPoint(int,int);
	void removePoint(int,int);
	void moveSelectedPoint(int,int,Qt::MouseButtons);
	void deselectPoint();

protected:

	//! Resets display
	void reset();

	//! Updates the list of active scales
	void updateScalesList(bool firstTime);

	//! Returns the list of active scales
	void getActiveScales(std::vector<float>& scales) const;

	//! Adds a custom object to the 2D view
	void addObject(ccHObject* obj);

	//! Updates zoom
	void updateZoom();

	//! Updates classifier path with the currently displayed polyline
	void updateClassifierPath(Classifier& classifier) const;

	//! Returns the click position in 3D
	CCVector3 getClickPos(int x, int y) const;
	//! Returns closest vertex
	int getClosestVertex(int x, int y, CCVector3& P) const;

	//! Gives access to the application (data-base, UI, etc.)
	ccMainAppInterface* m_app;

	//! Associated GL window
	ccGLWindow* m_glWindow;

	//! Associated classifier
	Classifier m_classifier;
	//! Whether the classifier has been saved (at least once)
	bool m_classifierSaved;

	//descritpors
	const CorePointDescSet* m_descriptors1;
	const CorePointDescSet* m_descriptors2;
	const CorePointDescSet* m_evaluationDescriptors;

	//classes
	int m_class1;
	QString m_cloud1Name;
	int m_class2;
	QString m_cloud2Name;

	//! Associated cloud
	ccPointCloud* m_cloud;
	//! Associated polyline
	ccPolyline* m_poly;
	//! Associated polyline vertices
	ccPointCloud* m_polyVertices;

	//! Currently selected polyline point
	int m_selectedPointIndex;
	//! Picking radius
	int m_pickingRadius;
};

#endif //Q_CANUPO_2DVIEW_DIALOG_HEADER
