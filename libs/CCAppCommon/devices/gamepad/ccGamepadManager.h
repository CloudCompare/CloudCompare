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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include "CCAppCommon.h"

//Qt
#include <QObject>

class QAction;
class QMenu;
class QString;

class ccMainAppInterface;
class GamepadInput;

/**
 * \class ccGamepadManager
 * \brief Manages gamepad input and interactions within the CloudCompare application
 * 
 * This class provides a centralized management system for gamepad devices,
 * handling device connection, input processing, and user interface integration.
 * 
 * \note Inherits from QObject to support signal/slot mechanisms
 */
class CCAPPCOMMON_LIB_API ccGamepadManager : public QObject
{
    Q_OBJECT
    
public:
    /**
     * \brief Constructs a new ccGamepadManager instance
     * 
     * \param appInterface Pointer to the main application interface
     * \param parent Parent QObject for memory management
     * 
     * \note Initializes gamepad menu and input handling
     */
    ccGamepadManager(ccMainAppInterface* appInterface, QObject* parent);
    
    /**
     * \brief Destructor for ccGamepadManager
     * 
     * Ensures proper cleanup of gamepad resources
     */
    ~ccGamepadManager() override;
    
    /**
     * \brief Retrieves the menu associated with gamepad controls
     * 
     * \return QMenu* Pointer to the gamepad-related menu
     * 
     * \note Returns nullptr if no menu has been created
     */
    QMenu* menu() { return m_menu; }
    
protected:
    /**
     * \brief Enables or disables a specific gamepad device
     * 
     * \param state True to enable the device, false to disable
     * \param silent If true, suppresses user-facing messages
     * \param deviceID Optional device identifier, defaults to -1 (all devices)
     * 
     * \note Handles device connection and disconnection logic
     */
    void enableDevice(bool state, bool silent, int deviceID = -1);
    
    /**
     * \brief Releases the current gamepad device resources
     * 
     * \note Disconnects and cleans up any active gamepad connections
     */
    void releaseDevice();

    /**
     * \brief Displays a user-facing message about gamepad status
     * 
     * \param message Text of the message to display
     * \param asWarning If true, displays as a warning message
     */
    void showMessage(QString message, bool asWarning);
    
    /**
     * \brief Sets up the gamepad-related menu in the application
     * 
     * \note Creates menu items and connects necessary signals
     */
    void setupMenu();
    
    /**
     * \brief Initializes the gamepad input handling system
     * 
     * \note Configures input processing and event handling
     */
    void setupGamepadInput();
    
    /**
     * \brief Handles incoming gamepad input events
     * 
     * \note Processes and routes gamepad input to appropriate application actions
     */
    void onGamepadInput();
    
private:
    ccMainAppInterface* m_appInterface; ///< Pointer to the main application interface
    GamepadInput* m_gamepadInput;       ///< Gamepad input handling object
    QMenu* m_menu;                      ///< Menu for gamepad-related controls
    QAction* m_actionEnable;            ///< Action to enable/disable gamepad input
};
