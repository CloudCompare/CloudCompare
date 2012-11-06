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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2257                                                              $
//$LastChangedDate:: 2012-10-11 23:48:15 +0200 (jeu., 11 oct. 2012)        $
//**************************************************************************
//

#ifndef CC_STD_PLUGIN_INTERFACE_HEADER
#define CC_STD_PLUGIN_INTERFACE_HEADER

//Qt
#include <QWidget>

//qCC_db
#include <ccHObject.h>
class ccProgressDialog;

//qCC
#include "ccPluginInterface.h"
#include "ccMainAppInterface.h"

//UI Modification flags
#define CC_PLUGIN_REFRESH_GL_WINDOWS            0x00000001
#define CC_PLUGIN_REFRESH_ENTITY_BROWSER        0x00000002
#define CC_PLUGIN_EXPAND_DB_TREE				0x00000004

//! Standard CC plugin interface
/** Version 1.3
**/
class ccStdPluginInterface : public ccPluginInterface
{
public:

	//inherited from ccPluginInterface
    virtual CC_PLUGIN_TYPE getType() { return CC_STD_PLUGIN; }

	//! Sets application entry point
	/** Called just after plugin creation by qCC
	**/
	void setMainAppInterface(ccMainAppInterface* app) { m_app=app; }

    //! Applies plugin action to the currently selected entities
    /** The 'selectedEntities' vector can be used to output new entities.
		When the method successfully returns (code=1), CloudCompare will
		check for any entity appended to the end of 'selectedEntities' and will
		add it to the main tree DB.
        \param selectedEntities entities selected by the user before calling the plugin
        \param uiModificationFlags flags to specify standard UI modification requests (output)
        \param progressCb a progress dialog (provided by the main GUI)
        \param parent calling widget (for proper UI integration)
        \return error code (see ccPluginInterface::getErrorMessage)
    **/
    virtual int doAction(ccHObject::Container& selectedEntities,
                            unsigned& uiModificationFlags,
                            ccProgressDialog* progressCb=NULL,
                            QWidget* parent=NULL)=0;

	//! This method is called by the main application whenever the entity selection changes
	/** Does nothing and returns true by default. Should be re-implemented by the plugin if necessary.
        \param selectedEntities currently selected entities
        \return whether the plugin menu entry and toolbar icon should be enabled or not
	**/
	virtual bool onNewSelection(const ccHObject::Container& selectedEntities) { return true; }

    //! Returns custom error messages
    /** If ccPluginInterface::doAction returns an error code different
        from 0 (action cancelled by user) or 1 (ok), the plugin might be
        asked to return an error message corresponding to this error code.
        \param errorCode custom error code (as returned by doAction)
        \return corresponding error message
    **/
    virtual QString getErrorMessage(int errorCode/*, LANGUAGE lang*/)=0;

protected:

	//! Main application interface
	ccMainAppInterface* m_app;
};

Q_DECLARE_INTERFACE(ccStdPluginInterface,"edf.rd.CloudCompare.ccStdPluginInterface/1.3")

#endif
