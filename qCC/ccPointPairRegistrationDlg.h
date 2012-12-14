#ifndef POINT_PAIR_REGISTRATION_DIALOG_HEADER
#define POINT_PAIR_REGISTRATION_DIALOG_HEADER

//Local
#include <ccOverlayDialog.h>

//qCC_db
#include <ccPointCloud.h>


//Qt generated dialog
#include <ui_pointPairRegistrationDlg.h>

//CCLib
#include <CCGeom.h>
#include <RegistrationTools.h>

class ccGenericPointCloud;
class ccGenericGLDisplay;
class ccGLWindow;
class cc2DLabel;

//Dialog for the point-pair registration algorithm (Horn)
class ccPointPairRegistrationDlg : public ccOverlayDialog, Ui::pointPairRegistrationDlg
{
	Q_OBJECT

public:

	//! Default constructor
	ccPointPairRegistrationDlg(QWidget* parent = 0);

	//inherited from ccOverlayDialog
    virtual bool linkWith(ccGLWindow* win);
	virtual bool start();
	virtual void stop(bool state);

	//! Inits dialog
	bool init(ccGenericPointCloud* aligned, ccGenericPointCloud* reference=0);

	//! Clears dialog
	void clear();

	//! Adds a point to the 'align' set
	bool addAlignedPoint(const CCVector3& P, ccGenericPointCloud* cloud=0, unsigned pointIndex=0);
	//! Adds a point to the 'reference' set
	bool addReferencePoint(const CCVector3& P, ccGenericPointCloud* cloud=0, unsigned pointIndex=0);

protected slots:

	//! Slot called to change aligned cloud visibility
	void showAlignedCloud(bool);
	//! Slot called to change reference cloud visibility
	void showReferenceCloud(bool);

	//! Slot called to add a manual point to the 'align' set
	void addManualAlignedPoint();
	//! Slot called to add a manual point to the 'reference' set
	void addManualRefPoint();

	//! Slot called to remove the last point on the 'align' stack
	void unstackAligned();
	//! Slot called to remove the last point on the 'reference' stack
	void unstackRef();

	void processPickedPoint(int, unsigned, int, int);
	void apply();
	void align();
	void reset();
	void cancel();

protected:

	//! Enables (or not) buttons depending on the number of points in both lists
	void onPointCountChanged();

	//! Calls Horn registration (CCLib::HornRegistrationTools)
	bool callHornRegistration(CCLib::HornRegistrationTools::ScaledTransformation& trans, double& rms);

	//! Original cloud context
	struct cloudContext
	{
		//! Default constructor
		cloudContext(ccGenericPointCloud* entity);

		//! Restores cloud original state
		void restore();

		ccGenericPointCloud* cloud;
		ccGenericGLDisplay* originalDisplay;
		bool wasVisible;
		bool wasEnabled;
		bool wasSelected;
	};

	//! Aligned entity
	cloudContext m_aligned;

	//! Aligned points set
	ccPointCloud m_alignedPoints;
	
	//! Reference entity (if any)
	cloudContext m_reference;

	//! Reference points set
	ccPointCloud m_refPoints;

	//! Dedicated window
	ccGLWindow* m_win;

};

#endif //POINT_PAIR_REGISTRATION_DIALOG_HEADER