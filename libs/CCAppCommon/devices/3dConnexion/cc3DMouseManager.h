/**
 * \file cc3DMouseManager.h
 * \brief Manages 3D mouse input for CloudCompare
 *
 * This class provides an interface for handling 3D mouse devices 
 * within the CloudCompare application.
 *
 * \copyright CloudCompare project
 * \license GNU General Public License v2 or later
 */

#pragma once

#include "CCAppCommon.h"

#include <QObject>

class QAction;
class QMenu;

class ccMainAppInterface;
class Mouse3DInput;

/**
 * \class cc3DMouseManager
 * \brief Manages 3D mouse device interactions
 *
 * Handles device enabling, menu setup, and input processing 
 * for 3D mouse devices in CloudCompare.
 */
class CCAPPCOMMON_LIB_API cc3DMouseManager : public QObject
{
    Q_OBJECT

public:
    /**
     * \brief Constructor for 3D mouse manager
     * \param appInterface Pointer to the main application interface
     * \param parent Parent QObject
     */
    cc3DMouseManager( ccMainAppInterface *appInterface, QObject *parent );

    /**
     * \brief Destructor
     */
    ~cc3DMouseManager();

    /**
     * \brief Gets the menu associated with the 3D mouse
     * \return Pointer to the 3D mouse menu
     */
    QMenu *menu() { return m_menu; }

private:
    /**
     * \brief Enable or disable the 3D mouse device
     * \param state True to enable, false to disable
     * \param silent If true, suppress user notifications
     */
    void enableDevice(bool state, bool silent);

    /**
     * \brief Release the 3D mouse device resources
     */
    void releaseDevice();

    /**
     * \brief Set up the 3D mouse menu
     */
    void setupMenu();

    /**
     * \brief Handle 3D mouse key up event
     * \param key Key identifier
     */
    void on3DMouseKeyUp(int key);

    /**
     * \brief Handle 3D mouse command key up event
     * \param cmd Command identifier
     */
    void on3DMouseCMDKeyUp(int cmd);

    /**
     * \brief Handle 3D mouse key down event
     * \param key Key identifier
     */
    void on3DMouseKeyDown(int key);

    /**
     * \brief Handle 3D mouse command key down event
     * \param cmd Command identifier
     */
    void on3DMouseCMDKeyDown(int cmd);

    /**
     * \brief Handle 3D mouse movement
     * \param vec Vector of movement coordinates
     */
    void on3DMouseMove(std::vector<float> &vec);

    /**
     * \brief Handle 3D mouse device release
     */
    void on3DMouseReleased();

    /// Pointer to the main application interface
    ccMainAppInterface *m_appInterface;

    /// 3D mouse input handler
    Mouse3DInput *m3dMouseInput;

    /// Menu for 3D mouse controls
    QMenu *m_menu;

    /// Action to enable/disable 3D mouse
    QAction *m_actionEnable;
};
