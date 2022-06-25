#pragma once

//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################

//Qt
#include <QObject>
#include <QString>
#include <QIcon>

//qCC_db
#include <ccHObject.h>

//qCC_plugins
#include <ccPluginInterface.h>

class ccPointCloud;
class QAction;
class ccMainAppInterface;

//! PCL filter description
struct FilterDescription
{
public:
	QString filterName;
	QString entryName;
	QString statusTip;
	QIcon icon;

	FilterDescription()
		: filterName("PCLFilter")
		, entryName("filter")
		, statusTip("Compute something with PCL")
		, icon(QIcon(QString::fromUtf8(":/toolbar/PclUtils/icons/pcl.png")) )
	{}

	FilterDescription(QString filterName, QString entryName, QString statusTip, QString icon)
		: filterName(filterName)
		, entryName(entryName)
		, statusTip(statusTip)
		, icon(QIcon(icon))
	{}
};

//! Base abstract class for each implemented PCL filter
/** \note Remember to override compute(), at the minimum
	\author Luca Penasa
**/
class BaseFilter: public QObject
{
	Q_OBJECT

public:
	/** \brief Default constructor
	*	\param[in] desc		filter description
	*	\param[in] app		application interface (optional)
	*/
	BaseFilter(FilterDescription desc, ccMainAppInterface* app = nullptr);

	/** \brief Get the action associated with the button used in menu and toolbar creation
	**/
	inline QAction* getAction() { return m_action; }

	//! Returns the error message corresponding to a given error code
	/** Each filter have a set of possible error message to be used given bt getFilterErrorMessage()
		Baseclass implementation provides some generic messages.
		\note These messages can be replaced by re-implementing this method and handling the same
			  codes BEFORE calling the base class method
	**/
	virtual QString getErrorMessage(int errorCode) const;

	//! Returns the filter description
	const FilterDescription& getDescription() const { return m_desc; }

	//! Sets whether to show a progressbar while computing or not
	inline void setShowProgressBar(bool status) { m_showProgress = status; }

	//! Returns the first selected entity as a ccPointCloud
	/** \return nullptr if no cloud is selected or if is not a ccPointCloud
	**/
	ccPointCloud* getFirstSelectedEntityAsCCPointCloud() const;

	//! Returns the first selected entity (as a ccHObject)
	/** \return nullptr if no object is selected
	**/
	ccHObject* getFirstSelectedEntity() const;

	//! get all entities that are selected and that also are cc_point_cloud
	void getSelectedEntitiesThatAreCCPointCloud(ccHObject::Container & entities) const;

	//! get all entities that are selected and that also are of the specified type
	void getSelectedEntitiesThatAre(CC_CLASS_ENUM kind, ccHObject::Container & entities) const;

	//! Updates the internal copy of selected entities
	/** 'selectedEntities' is a vector of pointers to selected entities.
	**/
	virtual void updateSelectedEntities(const ccHObject::Container& selectedEntities);

	//! Performs the actual filter job
	/** This method MUST be re-implemented by derived filter
		\return 1 if successful (error code otherwise)
	**/
	virtual int compute() = 0;

	//! Sets associated CC application interface (to access DB)
	inline void setMainAppInterface(ccMainAppInterface* app) { m_app = app; }

	//! Returns associated CC application interface for accessing to some of mainWindow methods
	ccMainAppInterface* getMainAppInterface() { return m_app; }

public: //default error codes (reserved between -10 and 1)

	static constexpr int Success = 1;				//!< Filter successful
	static constexpr int CancelledByUser = 0;		//!< Process cancelled by user
	static constexpr int InvalidInput = -1;			//!< Internal error: Generic computation error
	static constexpr int ThreadAlreadyInUse = -2;	//!< Internal error: thread already in use
	static constexpr int InvalidParameters = -3;	//!< Invalid parameters
	static constexpr int ComputationError = -4;		//!< Generic computation error
	static constexpr int NotEnoughMemory = -5;		//!< Not enough memory

Q_SIGNALS:

	//! Signal emitted when an entity is (visually) modified
	void entityHasChanged(ccHObject*);

	//! Signal emitted when a new entity is created by the filter
	void newEntity(ccHObject*);

	//! Signal emitted when a new error message is produced
	void newErrorMessage(QString);

protected:

	//! Called when action is triggered
	/** \note performAction calls getParametersFromDialog() and then start()
	**/
	void performAction();

	//! Checks if current selection is compliant with the filter
	/** If not, an error is returned and computation stops.
		By default, baseclass method simply checks that selection
		is composed of one and only one ccPointCloud.
		This method should be overridden if necessary.
		\return true if selection is compliant (error code otherwise)
	**/
	virtual bool checkSelected() const;

	//! Optional: can be used to open a dialog and retrieve some parameters.
	/** Automatically called by performAction.
		Does nothing by default. Must be overridden if a dialog should be displayed.
		\return 1 if dialog has been successfully executed, and parameters are correct, 0 if canceled, negative error code otherwise
	**/
	virtual int getParametersFromDialog() { return 1; }

	//! Starts computation
	/** Automatically called by performAction.
		By default, baseclass method simply calls compute
		Can be overridden if needed (e.g. a pre-processing step before compute())
		\return 1 if whole process is successful (error code otherwise)
	**/
	virtual int start();

	//! Emits the error corresponding to a given error code (see newErrorMessage)
	/** Error messages are retrieved from getErrorMessage() and getFilterErrorMessage()
		\param errCode Error code (identifies a given error message)
	**/
	void throwError(int errCode);

protected: //variables

	//! Filter information
	/** Contains all pieces of information about the filter (name, etc.)
	**/
	FilterDescription m_desc;

	//! The filter action
	QAction* m_action;

	//! Currently selected entities
	ccHObject::Container m_selectedEntities;

	//! Associated application interface
	ccMainAppInterface* m_app;

	//! Do we want to show a progress bar while the filter is being applied
	bool m_showProgress;
};
