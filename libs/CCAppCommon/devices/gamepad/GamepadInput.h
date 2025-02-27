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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "CCAppCommon.h"

//CCCoreLib
#include <CCConst.h>

//qCC_db
#include <ccGLMatrix.h>

//Qt
#include <QGamepad>
#include <QTimer>

class ccGLWindowInterface;

/**
 * \class GamepadInput
 * \brief Handles gamepad events through polling mechanism
 *
 * This class provides a wrapper around QGamepad to manage gamepad input
 * and translate gamepad events into 3D window interactions.
 *
 * \note Currently used primarily for testing purposes
 * \warning Functionality may be experimental or subject to change
 */
class CCAPPCOMMON_LIB_API GamepadInput : public QGamepad
{
    Q_OBJECT

public:
    /**
     * \brief Constructs a new GamepadInput object
     * 
     * \param parent Optional parent QObject for memory management
     * 
     * \note Initializes gamepad input with default settings
     */
    explicit GamepadInput(QObject* parent = nullptr);

    /**
     * \brief Destructor for GamepadInput
     * 
     * Ensures proper cleanup of gamepad resources
     */
    virtual ~GamepadInput();

    /**
     * \brief Starts polling for gamepad input
     * 
     * Begins periodic checks of gamepad state using internal timer
     * 
     * \note Activates updateInternalState() at regular intervals
     */
    void start();

    /**
     * \brief Stops polling for gamepad input
     * 
     * Halts periodic checks of gamepad state
     */
    void stop();

    /**
     * \brief Updates a 3D window based on current gamepad state
     * 
     * \param win Pointer to the 3D window to be updated
     * 
     * \note Translates gamepad input into 3D window transformations
     */
    void update(ccGLWindowInterface* win);

Q_SIGNALS:
    /**
     * \brief Emitted when gamepad state is updated
     * 
     * Signals that internal gamepad state has changed
     */
    void updated();

protected:
    /**
     * \brief Updates the internal gamepad state
     * 
     * Processes current gamepad input and updates internal tracking variables
     * 
     * \note Called periodically by the internal timer
     */
    void updateInternalState();

private:
    QTimer m_timer;             ///< Timer for polling gamepad state

    CCVector3 m_panning;        ///< Current panning vector
    bool m_hasPanning;          ///< Flag indicating if panning is active

    CCVector3 m_translation;    ///< Current translation vector
    bool m_hasTranslation;      ///< Flag indicating if translation is active

    ccGLMatrixd m_rotation;     ///< Current rotation matrix
    bool m_hasRotation;         ///< Flag indicating if rotation is active

    float m_zoom;               ///< Current zoom level
};