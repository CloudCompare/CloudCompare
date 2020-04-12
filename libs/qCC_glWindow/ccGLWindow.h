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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_GL_WINDOW_HEADER
#define CC_GL_WINDOW_HEADER

//qCC_db
#include <ccDrawableObject.h>
#include <ccGenericGLDisplay.h>
#include <ccGLUtils.h>

//qCC
#include "ccGuiParameters.h"

//Qt
#include <QElapsedTimer>
#include <QOpenGLExtensions>
#include <QTimer>

#ifdef CC_GL_WINDOW_USE_QWINDOW
#include <QWidget>
#include <QWindow>
#else
#include <QOpenGLWidget>
#endif

//system
#include <list>
#include <unordered_set>

class QOpenGLDebugMessage;

class ccBBox;
class ccColorRampShader;
class ccFrameBufferObject;
class ccGlFilter;
class ccHObject;
class ccInteractor;
class ccPolyline;
class ccShader;

struct HotZone;

#ifdef CC_GL_WINDOW_USE_QWINDOW
class QOpenGLPaintDevice;
using ccGLWindowParent = QWindow;
#else
using ccGLWindowParent = QOpenGLWidget;
#endif


//! OpenGL 3D view
class ccGLWindow : public ccGLWindowParent, public ccGenericGLDisplay
{
	Q_OBJECT

public:

	//! Picking mode
	enum PICKING_MODE { NO_PICKING,
						ENTITY_PICKING,
						ENTITY_RECT_PICKING,
						FAST_PICKING,
						POINT_PICKING,
						TRIANGLE_PICKING,
						POINT_OR_TRIANGLE_PICKING,
						LABEL_PICKING,
						DEFAULT_PICKING,
	};

	//! Interaction flags (mostly with the mouse)
	enum INTERACTION_FLAG {

		//no interaction
		INTERACT_NONE = 0,

		//camera interactions
		INTERACT_ROTATE          =  1,
		INTERACT_PAN             =  2,
		INTERACT_CTRL_PAN        =  4,
		INTERACT_ZOOM_CAMERA     =  8,
		INTERACT_2D_ITEMS        = 16, //labels, etc.
		INTERACT_CLICKABLE_ITEMS = 32, //hot zone

		//options / modifiers
		INTERACT_TRANSFORM_ENTITIES = 64,
		
		//signals
		INTERACT_SIG_RB_CLICKED  = 128,      //right button clicked
		INTERACT_SIG_LB_CLICKED  = 256,      //left button clicked
		INTERACT_SIG_MOUSE_MOVED = 512,      //mouse moved (only if a button is clicked)
		INTERACT_SIG_BUTTON_RELEASED = 1024, //mouse button released
		INTERACT_SIG_MB_CLICKED  = 2048,     //middle button clicked
		INTERACT_SEND_ALL_SIGNALS = INTERACT_SIG_RB_CLICKED | INTERACT_SIG_LB_CLICKED | INTERACT_SIG_MB_CLICKED| INTERACT_SIG_MOUSE_MOVED | INTERACT_SIG_BUTTON_RELEASED,

		// default modes
		MODE_PAN_ONLY = INTERACT_PAN | INTERACT_ZOOM_CAMERA | INTERACT_2D_ITEMS | INTERACT_CLICKABLE_ITEMS,
		MODE_TRANSFORM_CAMERA = INTERACT_ROTATE | MODE_PAN_ONLY,
		MODE_TRANSFORM_ENTITIES = INTERACT_ROTATE | INTERACT_PAN | INTERACT_ZOOM_CAMERA | INTERACT_TRANSFORM_ENTITIES | INTERACT_CLICKABLE_ITEMS,
	};
	Q_DECLARE_FLAGS(INTERACTION_FLAGS, INTERACTION_FLAG)

	//! Default message positions on screen
	enum MessagePosition {  LOWER_LEFT_MESSAGE,
							UPPER_CENTER_MESSAGE,
							SCREEN_CENTER_MESSAGE,
	};

	//! Message type
	enum MessageType {  CUSTOM_MESSAGE,
						SCREEN_SIZE_MESSAGE,
						PERSPECTIVE_STATE_MESSAGE,
						SUN_LIGHT_STATE_MESSAGE,
						CUSTOM_LIGHT_STATE_MESSAGE,
						MANUAL_TRANSFORMATION_MESSAGE,
						MANUAL_SEGMENTATION_MESSAGE,
						ROTAION_LOCK_MESSAGE,
						FULL_SCREEN_MESSAGE,
	};

	//! Pivot symbol visibility
	enum PivotVisibility {  PIVOT_HIDE,
							PIVOT_SHOW_ON_MOVE,
							PIVOT_ALWAYS_SHOW,
	};

	//! Default constructor
	ccGLWindow(QSurfaceFormat* format = nullptr, ccGLWindowParent* parent = nullptr, bool silentInitialization = false);

	//! Destructor
	~ccGLWindow() override;

#ifdef CC_GL_WINDOW_USE_QWINDOW
	//! Returns the parent widget
	QWidget* parentWidget() const { return m_parentWidget; }

	//! Sets 'parent' widget
	void setParentWidget(QWidget* widget);

	//! Returns the font
	inline const QFont& font() const { return m_font; }

	//shortcuts
	void setWindowTitle(QString title) { setTitle(title); }
	QString windowTitle() const { return title(); }
#endif

	//! Sets 'scene graph' root
	void setSceneDB(ccHObject* root);

	//! Returns current 'scene graph' root
	ccHObject* getSceneDB();

	//replacement for the missing methods of QGLWidget
	void renderText(int x, int y, const QString & str, const QFont & font = QFont());
	void renderText(double x, double y, double z, const QString & str, const QFont & font = QFont());

	//inherited from ccGenericGLDisplay
	void toBeRefreshed() override;
	void refresh(bool only2D = false) override;
	void invalidateViewport() override;
	void deprecate3DLayer() override;
	void display3DLabel(const QString& str, const CCVector3& pos3D, const ccColor::Rgba* color = nullptr, const QFont& font = QFont()) override;
	void displayText(QString text, int x, int y, unsigned char align = ALIGN_DEFAULT, float bkgAlpha = 0.0f, const ccColor::Rgba* color = nullptr, const QFont* font = nullptr) override;
	QFont getTextDisplayFont() const override; //takes rendering zoom into account!
	QFont getLabelDisplayFont() const override; //takes rendering zoom into account!
	const ccViewportParameters& getViewportParameters() const override { return m_viewportParams; }
	QPointF toCenteredGLCoordinates(int x, int y) const override;
	QPointF toCornerGLCoordinates(int x, int y) const override;
	void setupProjectiveViewport(const ccGLMatrixd& cameraMatrix, float fov_deg = 0.0f, float ar = 1.0f, bool viewerBasedPerspective = true, bool bubbleViewMode = false) override;
#ifdef CC_GL_WINDOW_USE_QWINDOW
	inline QWidget* asWidget() override { return m_parentWidget; }
#else
	inline QWidget* asWidget() override { return this; }
#endif
	inline QSize getScreenSize() const override { return size(); }
	void getGLCameraParameters(ccGLCameraParameters& params) override;

	//! Displays a status message in the bottom-left corner
	/** WARNING: currently, 'append' is not supported for SCREEN_CENTER_MESSAGE
		\param message message (if message is empty and append is 'false', all messages will be cleared)
		\param pos message position on screen
		\param append whether to append the message or to replace existing one(s) (only messages of the same type are impacted)
		\param displayMaxDelay_sec minimum display duration
		\param type message type (if not custom, only one message of this type at a time is accepted)
	**/
	virtual void displayNewMessage(const QString& message,
									MessagePosition pos,
									bool append=false,
									int displayMaxDelay_sec = 2,
									MessageType type = CUSTOM_MESSAGE);

	//! Activates sun light
	virtual void setSunLight(bool state);
	//! Toggles sun light
	virtual void toggleSunLight();
	//! Returns whether sun light is enabled or not
	virtual bool sunLightEnabled() const {return m_sunLightEnabled;}
	//! Activates custom light
	virtual void setCustomLight(bool state);
	//! Toggles custom light
	virtual void toggleCustomLight();
	//! Returns whether custom light is enabled or not
	virtual bool customLightEnabled() const {return m_customLightEnabled;}

	//! Sets current zoom
	/** Warning: has no effect in viewer-centered perspective mode
	**/
	virtual void setZoom(float value);

	//! Updates current zoom
	/** Warning: has no effect in viewer-centered perspective mode
	**/
	virtual void updateZoom(float zoomFactor);

	//! Sets pivot visibility
	virtual void setPivotVisibility(PivotVisibility vis);

	//! Returns pivot visibility
	virtual PivotVisibility getPivotVisibility() const;

	//! Shows or hide the pivot symbol
	/** Warnings:
		- not to be mistaken with setPivotVisibility
		- only taken into account if pivot visibility is set to PIVOT_SHOW_ON_MOVE
	**/
	virtual void showPivotSymbol(bool state);

	//! Sets pixel size (i.e. zoom base)
	/** Emits the 'pixelSizeChanged' signal.
	**/
	virtual void setPixelSize(float pixelSize);

	//! Sets pivot point
	/** Emits the 'pivotPointChanged' signal.
	**/
	virtual void setPivotPoint(	const CCVector3d& P,
								bool autoUpdateCameraPos = false,
								bool verbose = false);

	//! Sets camera position
	/** Emits the 'cameraPosChanged' signal.
	**/
	virtual void setCameraPos(const CCVector3d& P);

	//! Displaces camera
	/** Values are given in objects world along the current camera
		viewing directions (we use the right hand rule):
		* X: horizontal axis (right)
		* Y: vertical axis (up)
		* Z: depth axis (pointing out of the screen)
	**/
	virtual void moveCamera(float dx, float dy, float dz);

	//! Set perspective state/mode
	/** Persepctive mode can be:
		- object-centered (moving the mouse make the object rotate)
		- viewer-centered (moving the mouse make the camera move)
		\warning Disables bubble-view mode automatically

		\param state whether perspective mode is enabled or not
		\param objectCenteredView whether view is object- or viewer-centered (forced to true in ortho. mode)
	**/
	virtual void setPerspectiveState(bool state, bool objectCenteredView);

	//! Toggles perspective mode
	/** If perspective is activated, the user must specify if it should be
		object or viewer based (see setPerspectiveState)
	**/
	virtual void togglePerspective(bool objectCentered);

	//! Returns perspective mode
	virtual bool getPerspectiveState(bool& objectCentered) const;

	//! Shortcut: returns whether object-based perspective mode is enabled
	virtual bool objectPerspectiveEnabled() const;
	//! Shortcut: returns whether viewer-based perspective mode is enabled
	virtual bool viewerPerspectiveEnabled() const;

	//! Sets bubble-view mode state
	/** Bubble-view is a kind of viewer-based perspective mode where
		the user can't displace the camera (apart from up-down or
		left-right rotations). The f.o.v. is also maximized.

		\warning Any call to a method that changes the perpsective will
		automatically disable this mode.
	**/
	void setBubbleViewMode(bool state);
	
	//! Returns whether bubble-view mode is enabled or no
	inline bool bubbleViewModeEnabled() const { return m_bubbleViewModeEnabled; }

	//! Set bubble-view f.o.v. (in degrees)
	void setBubbleViewFov(float fov_deg);

	//! Center and zoom on a given bounding box
	/** If no bounding box is defined, the current displayed 'scene graph'
		bounding box is taken.
	**/
	virtual void updateConstellationCenterAndZoom(const ccBBox* aBox = nullptr);

	//! Returns the visible objects bounding-box
	void getVisibleObjectsBB(ccBBox& box) const;

	//! Rotates the base view matrix
	/** Warning: 'base view' marix is either:
		- the rotation around the object in object-centered mode
		- the rotation around the camera center in viewer-centered mode
		(see setPerspectiveState).
	**/
	virtual void rotateBaseViewMat(const ccGLMatrixd& rotMat);

	//! Returns the base view matrix
	/** Warning: 'base view' marix is either:
		- the rotation around the object in object-centered mode
		- the rotation around the camera center in viewer-centered mode
		(see setPerspectiveState).
	**/
	virtual const ccGLMatrixd& getBaseViewMat();
	
	//! Sets the base view matrix
	/** Warning: 'base view' marix is either:
		- the rotation around the object in object-centered mode
		- the rotation around the camera center in viewer-centered mode
		(see setPerspectiveState).
	**/
	virtual void setBaseViewMat(ccGLMatrixd& mat);

	//! Sets camera to a predefined view (top, bottom, etc.)
	virtual void setView(CC_VIEW_ORIENTATION orientation, bool redraw = true);

	//! Sets camera to a custom view (forward and up directions must be specified)
	virtual void setCustomView(const CCVector3d& forward, const CCVector3d& up, bool forceRedraw = true);

	//! Sets current interaction flags
	virtual void setInteractionMode(INTERACTION_FLAGS flags);
	//! Returns the current interaction flags
	inline virtual INTERACTION_FLAGS getInteractionMode() const { return m_interactionFlags; }

	//! Sets current picking mode
	/** Picking can be applied to entities (default), points, triangles, etc.)
	**/
	virtual void setPickingMode(PICKING_MODE mode = DEFAULT_PICKING);

	//! Returns current picking mode
	inline virtual PICKING_MODE getPickingMode() const { return m_pickingMode; }

	//! Locks picking mode
	/** \warning Bes sure to unlock it at some point ;)
	**/
	inline virtual void lockPickingMode(bool state) { m_pickingModeLocked = state; }

	//! Returns whether picking mode is locked or not
	inline virtual bool isPickingModeLocked() const { return m_pickingModeLocked; }

	//! Specify whether this 3D window can be closed by the user or not
	virtual void setUnclosable(bool state);

	//! Returns context information
	virtual void getContext(CC_DRAW_CONTEXT& context);

	//! Minimum point size
	static constexpr float MIN_POINT_SIZE_F = 1.0f;
	//! Maximum point size
	static constexpr float MAX_POINT_SIZE_F = 16.0f;

	//! Sets point size
	/** \param size point size (between MIN_POINT_SIZE_F and MAX_POINT_SIZE_F)
	**/
	virtual void setPointSize(float size, bool silent = false);
	
	//! Minimum line width
	static constexpr float MIN_LINE_WIDTH_F = 1.0f;
	//! Maximum line width
	static constexpr float MAX_LINE_WIDTH_F = 16.0f;

	//! Sets line width
	/** \param width lines width (between MIN_LINE_WIDTH_F and MAX_LINE_WIDTH_F)
	**/
	virtual void setLineWidth(float width, bool silent = false);

	//! Returns current font size
	virtual int getFontPointSize() const;
	//! Returns current font size for labels
	virtual int getLabelFontPointSize() const;

	//! Returns window own DB
	virtual ccHObject* getOwnDB();
	//! Adds an entity to window own DB
	/** By default no dependency link is established between the entity and the window (DB).
	**/
	virtual void addToOwnDB(ccHObject* obj, bool noDependency = true);
	//! Removes an entity from window own DB
	virtual void removeFromOwnDB(ccHObject* obj);

	//! Sets viewport parameters (all at once)
	virtual void setViewportParameters(const ccViewportParameters& params);

	//! Sets current camera f.o.v. (field of view) in degrees
	/** FOV is only used in perspective mode.
	**/
	virtual void setFov(float fov);
	//! Returns the current f.o.v. (field of view) in degrees
	virtual float getFov() const;

	//! Sets current camera aspect ratio (width/height)
	/** AR is only used in perspective mode.
	**/
	virtual void setAspectRatio(float ar);

	//! Sets current camera 'zNear' coefficient
	/** zNear coef. is only used in perspective mode.
	**/
	virtual void setZNearCoef(double coef);

	//! Invalidate current visualization state
	/** Forces view matrix update and 3D/FBO display.
	**/
	virtual void invalidateVisualization();

	//! Renders screen to an image
	virtual QImage renderToImage(	float zoomFactor = 1.0f,
									bool dontScaleFeatures = false,
									bool renderOverlayItems = false,
									bool silent = false);

	//! Renders screen to a file
	virtual bool renderToFile(	QString filename,
								float zoomFactor = 1.0f,
								bool dontScaleFeatures = false,
								bool renderOverlayItems = false);

	static void setShaderPath( const QString &path );
	
	virtual void setShader(ccShader* shader);
	virtual void setGlFilter(ccGlFilter* filter);
	ccGlFilter* getGlFilter() { return m_activeGLFilter; }

	virtual bool areShadersEnabled() const;
	virtual bool areGLFiltersEnabled() const;

	//! Returns the actual pixel size on screen (taking zoom or perspective parameters into account)
	/** In perspective mode, this value is approximate.
	**/
	virtual double computeActualPixelSize() const;

	//! Returns the zoom value equivalent to the current camera position (perspective only)
	float computePerspectiveZoom() const;

	//! Returns whether the ColorRamp shader is supported or not
	bool hasColorRampShader() const { return m_colorRampShader != nullptr; }

	//! Returns whether rectangular picking is allowed or not
	bool isRectangularPickingAllowed() const { return m_allowRectangularEntityPicking; }

	//! Sets whether rectangular picking is allowed or not
	void setRectangularPickingAllowed(bool state) { m_allowRectangularEntityPicking = state; }

	//! Returns current viewing direction
	/** This is the direction normal to the screen
		(pointing 'inside') in world base.
	**/
	CCVector3d getCurrentViewDir() const;

	//! Returns current up direction
	/** This is the vertical direction of the screen
		(pointing 'upward') in world base.
	**/
	CCVector3d getCurrentUpDir() const;

	//! Returns current parameters for this display (const version)
	/** Warning: may return overridden parameters!
	**/
	const ccGui::ParamStruct& getDisplayParameters() const;

	//! Sets current parameters for this display
	void setDisplayParameters(const ccGui::ParamStruct& params, bool thisWindowOnly = false);

	//! Whether display parameters are overidden for this window
	bool hasOverridenDisplayParameters() const { return m_overridenDisplayParametersEnabled; }

	//! Default picking radius value
	static const int DefaultPickRadius = 5;

	//! Sets picking radius
	inline void setPickingRadius(int radius) { m_pickRadius = radius; }
	//! Returns the current picking radius
	inline int getPickingRadius() const { return m_pickRadius; }

	//! Sets whether overlay entities (scale, tetrahedron, etc.) should be displayed or not
	void displayOverlayEntities(bool state) { m_displayOverlayEntities = state; }

	//! Returns whether overlay entities (scale, tetrahedron, etc.) are displayed or not
	bool overlayEntitiesAreDisplayed() const { return m_displayOverlayEntities; }

	//! Backprojects a 2D points on a 3D triangle
	/** \warning Uses the current display parameters!
		\param P2D point on the screen
		\param A3D first vertex of the 3D triangle
		\param B3D second vertex of the 3D triangle
		\param C3D third vertex of the 3D triangle
		\return backprojected point
	**/
	CCVector3 backprojectPointOnTriangle(	const CCVector2i& P2D,
											const CCVector3& A3D,
											const CCVector3& B3D,
											const CCVector3& C3D );

	//! Returns unique ID
	inline int getUniqueID() const { return m_uniqueID; }

	//! Returns the widget width (in pixels)
	int qtWidth() const { return ccGLWindow::width(); }
	//! Returns the widget height (in pixels)
	int qtHeight() const { return ccGLWindow::height(); }
	//! Returns the widget size (in pixels)
	QSize qtSize() const { return ccGLWindowParent::size(); }

	//! Returns the OpenGL context width
	int glWidth() const { return m_glViewport.width(); }
	//! Returns the OpenGL context height
	int glHeight() const { return m_glViewport.height(); }
	//! Returns the OpenGL context size
	QSize glSize() const { return m_glViewport.size(); }

public: //LOD

	//! Returns whether LOD is enabled on this display or not
	inline bool isLODEnabled() const { return m_LODEnabled; }

	//! Enables or disables LOD on this display
	/** \return success
	**/
	bool setLODEnabled(bool state, bool autoDisable = false);

public: //fullscreen

	//! Toggles (exclusive) full-screen mode
	void toggleExclusiveFullScreen(bool state);

	//! Returns whether the window is in exclusive full screen mode or not
	inline bool exclusiveFullScreen() const { return m_exclusiveFullscreen; }

public: //debug traces on screen

	//! Shows debug info on screen
	inline void enableDebugTrace(bool state) { m_showDebugTraces = state; }

	//! Toggles debug info on screen
	inline void toggleDebugTrace() { m_showDebugTraces = !m_showDebugTraces; }

public: //stereo mode

	//! Seterovision parameters
	struct StereoParams
	{
		StereoParams();

		//! Glass/HMD type
		enum GlassType {	RED_BLUE = 1,
							BLUE_RED = 2,
							RED_CYAN = 3,
							CYAN_RED = 4,
							NVIDIA_VISION = 5,
							OCULUS = 6,
							GENERIC_STEREO_DISPLAY = 7
		};

		//! Whether stereo-mode is 'analgyph' or real stereo mode
		inline bool isAnaglyph() const { return glassType <= 4; }

		int screenWidth_mm;
		int screenDistance_mm;
		int eyeSeparation_mm;
		int stereoStrength;
		GlassType glassType;
	};

	//! Enables stereo display mode
	bool enableStereoMode(const StereoParams& params);

	//! Disables stereo display mode
	void disableStereoMode();

	//! Returns whether the stereo display mode is enabled or not
	inline bool stereoModeIsEnabled() const { return m_stereoModeEnabled; }
	
	//! Returns the current stereo mode parameters
	inline const StereoParams& getStereoParams() const { return m_stereoParams; }

	//! Sets whether to display the coordinates of the point below the cursor position
	void showCursorCoordinates(bool state) { m_showCursorCoordinates = state; }
	//! Whether the coordinates of the point below the cursor position are displayed or not
	bool cursorCoordinatesShown() const { return m_showCursorCoordinates; }

	//! Toggles the automatic setting of the pivot point at the center of the screen
	void setAutoPickPivotAtCenter(bool state);
	//! Whether the pivot point is automatically set at the center of the screen
	bool autoPickPivotAtCenter() const { return m_autoPickPivotAtCenter; }

	//! Lock the rotation axis
	void lockRotationAxis(bool state, const CCVector3d& axis);

	//! Returns whether the rotation axis is locaked or not
	bool isRotationAxisLocked() const { return m_rotationAxisLocked; }


public:

	//! Applies a 1:1 global zoom
	void zoomGlobal();

	//inherited from ccGenericGLDisplay
	void redraw(bool only2D = false, bool resetLOD = true) override;

	//called when receiving mouse wheel is rotated
	void onWheelEvent(float wheelDelta_deg);

	//! Tests frame rate
	void startFrameRateTest();

	//! Request an update of the display
	/** The request will be executed if not in auto refresh mode already
	**/
	void requestUpdate();

#ifdef CC_GL_WINDOW_USE_QWINDOW
	//! For compatibility with the QOpenGLWidget version
	inline void update() { paintGL(); }
#endif

protected:

#ifdef CC_GL_WINDOW_USE_QWINDOW
	//! Updates the display
	void paintGL();
#endif

	//! Renders the next L.O.D. level
	void renderNextLODLevel();

	//! Stops frame rate test
	void stopFrameRateTest();

	//! Reacts to the itemPickedFast signal
	void onItemPickedFast(ccHObject* pickedEntity, int pickedItemIndex, int x, int y);

	//! Checks for scheduled redraw
	void checkScheduledRedraw();

	//! OpenGL KHR debug log
	void handleLoggedMessage(const QOpenGLDebugMessage&);

	//! Performs standard picking at the last clicked mouse position (see m_lastMousePos)
	void doPicking();

signals:

	//! Signal emitted when an entity is selected in the 3D view
	void entitySelectionChanged(ccHObject* entity);
	//! Signal emitted when multiple entities are selected in the 3D view
	void entitiesSelectionChanged(std::unordered_set<int> entIDs);

	//! Signal emitted when a point (or a triangle) is picked
	/** \param entity 'picked' entity
		\param subEntityID point or triangle index in entity
		\param x mouse cursor x position
		\param y mouse cursor y position
		\param P the picked point
		\param uvw barycentric coordinates of the point (if picked on a mesh)
	**/
	void itemPicked(ccHObject* entity, unsigned subEntityID, int x, int y, const CCVector3& P, const CCVector3d& uvw);

	//! Signal emitted when an item is picked (FAST_PICKING mode only)
	/** \param entity entity
		\param subEntityID point or triangle index in entity
		\param x mouse cursor x position
		\param y mouse cursor y position
	**/
	void itemPickedFast(ccHObject* entity, int subEntityID, int x, int y);

	//! Signal emitted when fast picking is finished (FAST_PICKING mode only)
	void fastPickingFinished();

	/*** Camera link mode (interactive modifications of the view/camera are echoed to other windows) ***/

	//! Signal emitted when the window 'model view' matrix is interactively changed
	void viewMatRotated(const ccGLMatrixd& rotMat);
	//! Signal emitted when the camera is interactively displaced
	void cameraDisplaced(float ddx, float ddy);
	//! Signal emitted when the mouse wheel is rotated
	void mouseWheelRotated(float wheelDelta_deg);

	//! Signal emitted when the perspective state changes (see setPerspectiveState)
	void perspectiveStateChanged();

	//! Signal emitted when the window 'base view' matrix is changed
	void baseViewMatChanged(const ccGLMatrixd& newViewMat);

	//! Signal emitted when the pixel size is changed
	void pixelSizeChanged(float pixelSize);

	//! Signal emitted when the f.o.v. changes
	void fovChanged(float fov);

	//! Signal emitted when the zNear coef changes
	void zNearCoefChanged(float coef);

	//! Signal emitted when the pivot point is changed
	void pivotPointChanged(const CCVector3d&);

	//! Signal emitted when the camera position is changed
	void cameraPosChanged(const CCVector3d&);

	//! Signal emitted when the selected object is translated by the user
	void translation(const CCVector3d& t);

	//! Signal emitted when the selected object is rotated by the user
	/** \param rotMat rotation applied to current viewport (4x4 OpenGL matrix)
	**/
	void rotation(const ccGLMatrixd& rotMat);

	//! Signal emitted when the left mouse button is clicked on the window
	/** See INTERACT_SIG_LB_CLICKED.
		Arguments correspond to the clicked point coordinates (x,y) in
		pixels relative to the window corner!
	**/
	void leftButtonClicked(int x, int y);

	//! Signal emitted when the right mouse button is clicked on the window
	/** See INTERACT_SIG_RB_CLICKED.
		Arguments correspond to the clicked point coordinates (x,y) in
		pixels relative to the window corner!
	**/
	void rightButtonClicked(int x, int y);

	//! Signal emitted when the mouse is moved
	/** See INTERACT_SIG_MOUSE_MOVED.
		The two first arguments correspond to the current cursor coordinates (x,y)
		relative to the window corner!
	**/
	void mouseMoved(int x, int y, Qt::MouseButtons buttons);

	//! Signal emitted when a mouse button is released (cursor on the window)
	/** See INTERACT_SIG_BUTTON_RELEASED.
	**/
	void buttonReleased();

	//! Signal emitted during 3D pass of OpenGL display process
	/** Any object connected to this slot can draw additional stuff in 3D.
		Depth buffering, lights and shaders are enabled by default.
	**/
	void drawing3D();

	//! Signal emitted when files are dropped on the window
	void filesDropped(const QStringList& filenames);

	//! Signal emitted when a new label is created
	void newLabel(ccHObject* obj);

	//! Signal emitted when the exclusive fullscreen is toggled
	void exclusiveFullScreenToggled(bool exclusive);

	//! Signal emitted when the middle mouse button is clicked on the window
	/** See INTERACT_SIG_MB_CLICKED.
		Arguments correspond to the clicked point coordinates (x,y) in
		pixels relative to the window corner!
	**/
	void middleButtonClicked(int x, int y);

protected: //rendering

	//Default OpenGL functions set
	using ccQOpenGLFunctions = QOpenGLFunctions_2_1;

	//! Returns the set of OpenGL functions
	inline ccQOpenGLFunctions* functions() const { return context() ? context()->versionFunctions<ccQOpenGLFunctions>() : nullptr; }

#ifdef CC_GL_WINDOW_USE_QWINDOW
	//! Returns the context (if any)
	inline QOpenGLContext* context() const { return m_context; }
#endif

	//reimplemented from QOpenGLWidget
	//Because QOpenGLWidget::makeCurrent silently binds the widget's own FBO,
	//we need to automatically bind our own afterwards!
	//(Sadly QOpenGLWidget::makeCurrentmakeCurrent is not virtual)
	void makeCurrent();

	//! Binds an FBO or releases the current one (if input is nullptr)
	/** This method must be called instead of the FBO's own 'start' and 'stop' methods
		so as to properly handle the interactions with QOpenGLWidget's own FBO.
	**/
	bool bindFBO(ccFrameBufferObject* fbo);

	//! LOD state
	struct LODState
	{
		LODState()
			: inProgress(false)
			, level(0)
			, startIndex(0)
			, progressIndicator(0)
		{}

		//! LOD display in progress
		bool inProgress;
		//! Currently rendered LOD level
		unsigned char level;
		//! Currently rendered LOD start index
		unsigned startIndex;
		//! Currently LOD progress indicator
		unsigned progressIndicator;
	};

	//! Rendering params
	struct RenderingParams;

	//! Full rendering pass (drawBackground + draw3D + drawForeground)
	void fullRenderingPass(CC_DRAW_CONTEXT& context, RenderingParams& params);

	//! Draws the background layer
	/** Background + 2D background objects
	**/
	void drawBackground(CC_DRAW_CONTEXT& context, RenderingParams& params);

	//! Draws the main 3D layer
	void draw3D(CC_DRAW_CONTEXT& context, RenderingParams& params);

	//! Draws the foreground layer
	/** 2D foreground objects / text
	**/
	void drawForeground(CC_DRAW_CONTEXT& context, RenderingParams& params);

protected: //other methods

	//these methods are now protected to prevent issues with Retina or other high DPI displays
	//(see glWidth(), glHeight(), qtWidth(), qtHeight(), qtSize(), glSize()
	int width() const { return ccGLWindowParent::width(); }
	int height() const { return ccGLWindowParent::height(); }
	QSize size() const { return ccGLWindowParent::size(); }

	//! Returns the current (OpenGL) view matrix
	/** Warning: may be different from the 'view' matrix returned by getBaseViewMat.
		Will call automatically updateModelViewMatrix if necessary.
	**/
	virtual const ccGLMatrixd& getModelViewMatrix();
	//! Returns the current (OpenGL) projection matrix
	/** Will call automatically updateProjectionMatrix if necessary.
	**/
	virtual const ccGLMatrixd& getProjectionMatrix();

	//! Processes the clickable items
	/** \return true if an item has been clicked
	**/
	bool processClickableItems(int x, int y);

	//! Sets current font size
	/** Warning: only used internally.
		Change 'defaultFontSize' with setDisplayParameters instead!
	**/
	void setFontPointSize(int pixelSize);

	//events handling
	void mousePressEvent(QMouseEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;
	void mouseDoubleClickEvent(QMouseEvent* event) override;
	void mouseReleaseEvent(QMouseEvent* event) override;
	void wheelEvent(QWheelEvent* event) override;
	bool event(QEvent* evt) override;

	bool initialize();
	GLuint defaultQtFBO() const;

#ifdef CC_GL_WINDOW_USE_QWINDOW
	void resizeGL(int w, int h);
	virtual void dragEnterEvent(QDragEnterEvent* event);
	virtual void dropEvent(QDropEvent* event);
#else
	void initializeGL() override { initialize(); }
	void resizeGL(int w, int h) override;
	void paintGL() override;
	void dragEnterEvent(QDragEnterEvent* event) override;
	void dropEvent(QDropEvent* event) override;
#endif

	//Graphical features controls
	void drawCross();
	void drawTrihedron();
	void drawScale(const ccColor::Rgbub& color);

	//! Computes the model view matrix
	ccGLMatrixd computeModelViewMatrix(const CCVector3d& cameraCenter) const;

	//! Optional output metrics (from computeProjectionMatrix)
	struct ProjectionMetrics;

	//! Computes the projection matrix
	ccGLMatrixd computeProjectionMatrix(	const CCVector3d& cameraCenter,
											bool withGLfeatures, 
											ProjectionMetrics* metrics = nullptr, 
											double* eyeOffset = nullptr) const;
	void updateModelViewMatrix();
	void updateProjectionMatrix();
	void setStandardOrthoCenter();
	void setStandardOrthoCorner();

	//Lights controls (OpenGL scripts)
	void glEnableSunLight();
	void glDisableSunLight();
	void glEnableCustomLight();
	void glDisableCustomLight();
	void drawCustomLight();

	//! Draws pivot point symbol in 3D
	void drawPivot();

	//! Picking parameters
	struct PickingParameters;

	//! Starts picking process
	/** \param params picking parameters
	**/
	void startPicking(PickingParameters& params);

	//! Performs the picking with OpenGL
	void startOpenGLPicking(const PickingParameters& params);

	//! Starts OpenGL picking process
	void startCPUBasedPointPicking(const PickingParameters& params);

	//! Processes the picking process result and sends the corresponding signal
	void processPickingResult(	const PickingParameters& params,
								ccHObject* pickedEntity,
								int pickedItemIndex,
								const CCVector3* nearestPoint = nullptr,
								const CCVector3d* nearestPointBC = nullptr, //barycentric coordinates
								const std::unordered_set<int>* selectedIDs = nullptr);
	
	//! Updates currently active items list (m_activeItems)
	/** The items must be currently displayed in this context
		AND at least one of them must be under the mouse cursor.
	**/
	void updateActiveItemsList(int x, int y, bool extendToSelectedLabels = false);

	//! Currently active items
	/** Active items can be moved with mouse, etc.
	**/
	std::list<ccInteractor*> m_activeItems;

	//! Inits FBO (frame buffer object)
	bool initFBO(int w, int h);
	//! Inits FBO (safe)
	bool initFBOSafe(ccFrameBufferObject* &fbo, int w, int h);
	//! Releases any active FBO
	void removeFBO();
	//! Releases any active FBO (safe)
	void removeFBOSafe(ccFrameBufferObject* &fbo);

	//! Inits active GL filter (advanced shader)
	bool initGLFilter(int w, int h, bool silent = false);
	//! Releases active GL filter
	void removeGLFilter();

	//! Converts a given (mouse) position in pixels to an orientation
	/** The orientation vector origin is the current pivot point!
	**/
	CCVector3d convertMousePositionToOrientation(int x, int y);

	//! Returns the height of the 'GL filter' banner
	int getGlFilterBannerHeight() const;

	//! Returns real camera center (i.e. with z centered on the visible objects bounding-box in ortho mode)
	CCVector3d getRealCameraCenter() const;

	//! Draws the 'hot zone' (+/- icons for point size), 'leave bubble-view' button, etc.
	void drawClickableItems(int xStart, int& yStart);

	//! Disables current LOD rendering cycle
	void stopLODCycle();

	// Releases all textures, GL lists, etc.
	void uninitializeGL();

	//! Schedules a full redraw
	/** Any previously scheduled redraw will be cancelled.
		\warning The redraw will be cancelled if redraw/update is called before.
		\param maxDelay_ms the maximum delay for the call to redraw (in ms)
	**/
	void scheduleFullRedraw(unsigned maxDelay_ms);

	//! Cancels any scheduled redraw
	/** See ccGLWindow::scheduleFullRedraw.
	**/
	void cancelScheduledRedraw();

	//! Sets the OpenGL viewport (shortut)
	inline void setGLViewport(int x, int y, int w, int h) { setGLViewport(QRect(x, y, w, h)); }
	//! Sets the OpenGL viewport
	void setGLViewport(const QRect& rect);
	//! Returns the current (OpenGL) viewport
	inline const QRect& getGLViewport() const { return m_glViewport; }

	//! Logs a GL error
	/** Logs a warning or error message corresponding to the input GL error.
		If error == GL_NO_ERROR, nothing is logged.

		\param error GL error code
		\param context name of the method/object that catched the error
		\return true if an error occurred, false otherwise
	**/
	static void LogGLError(GLenum error, const char* context);

	//! Logs a GL error (shortcut)
	void logGLError(const char* context) const;

	//! Toggles auto-refresh mode
	void toggleAutoRefresh(bool state, int period_ms = 0);

	//! Returns the (relative) depth value at a given pixel position
	/** \return the (relative) depth or 1.0 if none is defined
	**/
	GLfloat getGLDepth(int x, int y, bool extendToNeighbors = false);

	//! Returns the approximate 3D position of the clicked pixel
	bool getClick3DPos(int x, int y, CCVector3d& P3D);

protected: //members

#ifdef CC_GL_WINDOW_USE_QWINDOW

	//! Associated OpenGL context
	QOpenGLContext* m_context;

	//! OpenGL device
	QOpenGLPaintDevice* m_device;

	//! Format
	QSurfaceFormat m_format;

	//! Associated widget (we use the WidgetContainer mechanism)
	QWidget* m_parentWidget;

#endif

	//! Unique ID
	int m_uniqueID;

	//! Initialization state
	bool m_initialized;

	//! Trihedron GL list
	GLuint m_trihedronGLList;

	//! Pivot center GL list
	GLuint m_pivotGLList;

	//! Viewport parameters (zoom, etc.)
	ccViewportParameters m_viewportParams;

    //! Last mouse position
	QPoint m_lastMousePos;

	//! Complete visualization matrix (GL style - double version)
	ccGLMatrixd m_viewMatd;
	//! Whether the model veiw matrix is valid (or need to be recomputed)
	bool m_validModelviewMatrix;
	//! Projection matrix (GL style - double version)
	ccGLMatrixd m_projMatd;
	//! Whether the projection matrix is valid (or need to be recomputed)
	bool m_validProjectionMatrix;
	//! Distance between the camera and the displayed objects bounding-box
	double m_cameraToBBCenterDist;
	//! Half size of the displayed objects bounding-box
	double m_bbHalfDiag;

	//! GL viewport
	QRect m_glViewport;

	//! Whether L.O.D. (level of detail) is enabled or not
	bool m_LODEnabled;
	//! Whether L.O.D. should be automatically disabled at the end of the rendering cycle
	bool m_LODAutoDisable;
	//! Whether the display should be refreshed on next call to 'refresh'
	bool m_shouldBeRefreshed;
	//! Whether the mouse (cursor) has moved after being pressed or not
	bool m_mouseMoved;
	//! Whether the mouse is currently pressed or not
	bool m_mouseButtonPressed;
	//! Whether this 3D window can be closed by the user or not
	bool m_unclosable;
	//! Current intercation flags
	INTERACTION_FLAGS m_interactionFlags;
	//! Current picking mode
	PICKING_MODE m_pickingMode;
	//! Whether picking mode is locked or not
	bool m_pickingModeLocked;

	//! Display capturing mode options
	struct CaptureModeOptions
	{
		//! Default constructor
		CaptureModeOptions()
			: enabled(false)
			, zoomFactor(1.0f)
			, renderOverlayItems(false)
		{}

		bool enabled;
		float zoomFactor;
		bool renderOverlayItems;
	};

	//! Display capturing mode options
	CaptureModeOptions m_captureMode;

	//! Temporary Message to display in the lower-left corner
	struct MessageToDisplay
	{
		MessageToDisplay()
			: messageValidity_sec(0)
			, position(LOWER_LEFT_MESSAGE)
			, type(CUSTOM_MESSAGE)
		{}
		
		//! Message
		QString message;
		//! Message end time (sec)
		qint64 messageValidity_sec;
		//! Message position on screen
		MessagePosition position;
		//! Message type
		MessageType type;
	};

	//! List of messages to display
	std::list<MessageToDisplay> m_messagesToDisplay;

	//! Last click time (msec)
	qint64 m_lastClickTime_ticks;

	//! Sun light position
	/** Relative to screen.
	**/
	float m_sunLightPos[4];
	//! Whether sun light is enabled or not
	bool m_sunLightEnabled;
	
	//! Custom light position
	/** Relative to object.
	**/
	float m_customLightPos[4];
	//! Whether custom light is enabled or not
	bool m_customLightEnabled;

	//! Clickable item
	struct ClickableItem
	{
		enum Role {	NO_ROLE,
					INCREASE_POINT_SIZE,
					DECREASE_POINT_SIZE,
					INCREASE_LINE_WIDTH,
					DECREASE_LINE_WIDTH,
					LEAVE_BUBBLE_VIEW_MODE,
					LEAVE_FULLSCREEN_MODE,
		};

		ClickableItem(): role(NO_ROLE) {}
		ClickableItem(Role _role, QRect _area) : role(_role), area(_area) {}

		Role role;
		QRect area;
	};
	
	//! Currently displayed clickable items
	std::vector<ClickableItem> m_clickableItems;

	//! Whether clickable items are visible (= mouse over) or not
	bool m_clickableItemsVisible;

	//! Currently active shader
	ccShader* m_activeShader;
	//! Whether shaders are enabled or not
	bool m_shadersEnabled;

	//! Currently active FBO (frame buffer object)
	ccFrameBufferObject* m_activeFbo;
	//! First default FBO (frame buffer object)
	ccFrameBufferObject* m_fbo;
	//! Second default FBO (frame buffer object) - used for stereo rendering
	ccFrameBufferObject* m_fbo2;
	//! Whether to always use FBO or only for GL filters
	bool m_alwaysUseFBO;
	//! Whether FBO should be updated (or simply displayed as a texture = faster!)
	bool m_updateFBO;

	// Color ramp shader
	ccColorRampShader* m_colorRampShader;
	// Custom rendering shader (OpenGL 3.3+)
	ccShader* m_customRenderingShader;

	//! Active GL filter
	ccGlFilter* m_activeGLFilter;
	//! Whether GL filters are enabled or not
	bool m_glFiltersEnabled;

	//! Window own DB
	ccHObject* m_winDBRoot;

	//! CC main DB
	ccHObject* m_globalDBRoot;

	//! Default font
	QFont m_font;

	//! Pivot symbol visibility
	PivotVisibility m_pivotVisibility;

	//! Whether pivot symbol should be shown or not
	bool m_pivotSymbolShown;

	//! Whether rectangular picking is allowed or not
	bool m_allowRectangularEntityPicking;

	//! Rectangular picking polyline
	ccPolyline* m_rectPickingPoly;

	//! Overridden display parameter 
	ccGui::ParamStruct m_overridenDisplayParameters;

	//! Whether display parameters are overidden for this window
	bool m_overridenDisplayParametersEnabled;

	//! Whether to display overlay entities or not (scale, tetrahedron, etc.)
	bool m_displayOverlayEntities;

	//! Whether initialization should be silent or not (i.e. no message to console)
	bool m_silentInitialization;

	//! Bubble-view mode state
	bool m_bubbleViewModeEnabled;
	//! Bubble-view mode f.o.v. (degrees)
	float m_bubbleViewFov_deg;
	//! Pre-bubble-view camera parameters (backup)
	ccViewportParameters m_preBubbleViewParameters;

	//! Current LOD state
	LODState m_currentLODState;

	//! LOD refresh signal sent
	bool m_LODPendingRefresh;
	//! LOD refresh signal should be ignored
	bool m_LODPendingIgnore;

	//! Internal timer
	QElapsedTimer m_timer;

	//! Touch event in progress
	bool m_touchInProgress;
	//! Touch gesture initial distance
	qreal m_touchBaseDist;

	//! Scheduler timer
	QTimer m_scheduleTimer;
	//! Scheduled full redraw (no LOD)
	qint64 m_scheduledFullRedrawTime;

	//! Seterovision mode parameters
	StereoParams m_stereoParams;

	//! Whether seterovision mode is enabled or not
	bool m_stereoModeEnabled;

	//! Former parent object (for exclusive full-screen display)
	QWidget* m_formerParent;

	//! Wether exclusive full screen is enabled or not
	bool m_exclusiveFullscreen;
	
	//! Former geometry (for exclusive full-screen display)
	QByteArray m_formerGeometry;

	//! Debug traces visibility
	bool m_showDebugTraces;

	//! Picking radius (pixels)
	int m_pickRadius;

	//! FBO support
	QOpenGLExtension_ARB_framebuffer_object	m_glExtFunc;

	//! Whether FBO support is on
	bool m_glExtFuncSupported;

	//! Auto-refresh mode
	bool m_autoRefresh;
	//! Auto-refresh timer
	QTimer m_autoRefreshTimer;

	//! Hot zone
	HotZone* m_hotZone;

	//! Whether to display the coordinates of the point below the cursor position
	bool m_showCursorCoordinates;

	//! Whether the pivot point is automatically picked at the center of the screen (when possible)
	bool m_autoPickPivotAtCenter;

	//! Deferred picking
	QTimer m_deferredPickingTimer;

	//! Ignore next mouse release event
	bool m_ignoreMouseReleaseEvent;

	//! Wheter the rotation axis is locked or not
	bool m_rotationAxisLocked;
	//! Locked rotation axis
	CCVector3d m_lockedRotationAxis;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(ccGLWindow::INTERACTION_FLAGS);

#endif //CC_GL_WINDOW_HEADER
