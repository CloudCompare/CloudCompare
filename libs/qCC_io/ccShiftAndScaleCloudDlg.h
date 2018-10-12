#ifndef CC_RECENTER_CLOUD_DIALOG
#define CC_RECENTER_CLOUD_DIALOG

//local
#include "qCC_io.h"
#include "ccGlobalShiftManager.h"

//Qt
#include <QDialog>

//CCLib
#include <CCGeom.h>

class Ui_GlobalShiftAndScaleDlg;
class QAbstractButton;

//! Dialog for selection of cloud center
class QCC_IO_LIB_API ccShiftAndScaleCloudDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	/** \param Pg a point expressed in the original coordinate system
		\param Dg bounding box diagonal in the original coordinate system (or <= 0 to ignore)
		\param parent parent widget
	**/
	ccShiftAndScaleCloudDlg(const CCVector3d& Pg, double Dg = 0, QWidget* parent = 0);

	//! Reverse mode constructor
	/** \param Pl a point expressed in the local coordinate system
		\param Dl bounding box diagonal in the local coordinate system
		\param Pg a point expressed in the original coordinate system
		\param Dg bounding box diagonal in the original coordinate system
		\param parent parent widget
	**/
	ccShiftAndScaleCloudDlg(const CCVector3d& Pl, double Dl, const CCVector3d& Pg, double Dg, QWidget* parent = 0);

	//! Sets the Shift fields (X, Y and Z) precision (default should be 2)
	void setShiftFieldsPrecision(int precision);

	//! Destructor
	virtual ~ccShiftAndScaleCloudDlg();

	//! Returns shift
	CCVector3d getShift() const;

	//! Returns scale
	double getScale() const;

	//! Whether shift should be applied to all files
	bool applyAll() const { return m_applyAll; }

	//! Whether the user has clicked on the cancel button or not
	bool cancelled() const { return m_cancel; }

	//! Whether to show dialog items related to scale
	void showScaleItems(bool state);

	//! Whether to show the 'Apply all' button or not
	void showApplyAllButton(bool state);
	//! Whether to show the 'Apply' button or not
	void showApplyButton(bool state);
	//! Whether to show the 'No' button or not
	void showNoButton(bool state);
	//! Whether to show the 'Cancel' button or not
	void showCancelButton(bool state);

	//! Whether to show or not the warning about non pertinent shift information
	void showWarning(bool state);

	//! Whether to show or not the title
	void showTitle(bool state);

	//! Whether to show or not the 'Keep global position' checkbox
	void showKeepGlobalPosCheckbox(bool state);
	//! Returns whether the global position should be preserved or not
	bool keepGlobalPos() const;
	//! Sets whether the global position should be preserved or not
	void setKeepGlobalPos(bool state);
	
	//! Whether to show or not the 'Preserve shift on save' checkbox
	void showPreserveShiftOnSave(bool state);
	//! Returns whether the global shift should be preserved or not
	bool preserveShiftOnSave() const;
	//! Sets whether the global shift should be preserved or not
	void setPreserveShiftOnSave(bool state);

	//! Adds shift info to the combox
	/** \param info shift info
		\return index in combo-box
	**/
	int addShiftInfo(const ccGlobalShiftManager::ShiftInfo& info);

	//! Adds shift info to the combox
	/** \param info shift info
		\return index in combo-box
	**/
	int addShiftInfo(const std::vector<ccGlobalShiftManager::ShiftInfo>& info);

	//! Returns a given input info
	bool getInfo(size_t index, ccGlobalShiftManager::ShiftInfo& info) const;

	//! Returns the number of info currently stored
	size_t infoCount() const { return m_defaultInfos.size(); }

	//! Sets the current combo-box entry (profile)
	void setCurrentProfile(int index);

	//! Adds information from default file (if any)
	bool addFileInfo();

    //! Sets the last shift and scale information
    static void SetLastInfo(const CCVector3d& shift, double scale);

protected slots:

	//! Slot called when the 'loadComboBox' index changes
	void onLoadIndexChanged(int);
	//! Slot called when the 'Keep global position' checkbox is toggled
	void onGlobalPosCheckBoxToggled(bool);
	//! Analyzes the clicked button
	void onClick(QAbstractButton* button);
	//! Updates info on the global and local coordinate systems
	void updateGlobalAndLocalSystems();
	//! Displays more info about global shift mechanism
	void displayMoreInfo();

protected:

	//! Initialization routine
	void init();

	//! Sets displayed shift
	void setShift(const CCVector3d& shift);

	//! Sets displayed scale
	void setScale(double scale);

	//! Updates info on the local coordinate system
	void updateLocalSystem();
	//! Updates info on the global coordinate system
	void updateGlobalSystem();

	//! Tries to load ShiftInfo data from a (text) file
	/** Data is stored in m_defaultInfos.
		\param filename filename
		\return success
	**/
	bool loadInfoFromFile(QString filename);

	//! Associated UI
	Ui_GlobalShiftAndScaleDlg* m_ui;

	//! Whether shift should be applied to all files
	bool m_applyAll;
	//! Whether the user has clicked on Cancel or not
	bool m_cancel;

	//! Default infos (typically loaded from the global_shift_list.txt' file)
	std::vector<ccGlobalShiftManager::ShiftInfo> m_defaultInfos;
	//! Active info entry index
	int m_activeInfoIndex;

	//! Original coordinate system point
	CCVector3d m_originalPoint;
	//! Original coordinate system diagonal
	double m_originalDiagonal;
	//! Local coordinate system point (reversed mode only)
	CCVector3d m_localPoint;
	//! Local coordinate system diagonal (reversed mode only)
	double m_localDiagonal;

	//! Whether the reverse mode is active or not
	bool m_reversedMode;
};

#endif
