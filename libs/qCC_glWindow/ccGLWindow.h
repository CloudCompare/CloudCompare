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

#ifndef CC_GL_WINDOW_HEADER
#define CC_GL_WINDOW_HEADER

//CCLib
#include <CCGeom.h>

//qCC_db
#include <ccIncludeGL.h>
#include <ccGLMatrix.h>
#include <ccDrawableObject.h>
#include <ccGenericGLDisplay.h>
#include <ccGLUtils.h>

//qCC
#include "ccGuiParameters.h"

//Qt
#include <QGLWidget>
#include <QFont>

//system
#include <set>
#include <list>

//! OpenGL picking buffer size (= max hits number per 'OpenGL' selection pass)
#define CC_PICKING_BUFFER_SIZE 65536

class ccHObject;
class ccBBox;
class ccShader;
class ccColorRampShader;
class ccGlFilter;
class ccFrameBufferObject;
class ccInteractor;
class ccPolyline;

//! OpenGL 3D view
class ccGLWindow : public QGLWidget, public ccGenericGLDisplay
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
						AUTO_POINT_PICKING,
						DEFAULT_PICKING,
	};

	//! Interaction mode (with the mouse!)
	enum INTERACTION_MODE { TRANSFORM_CAMERA,
							TRANSFORM_ENTITY,
							SEGMENT_ENTITY,
							PAN_ONLY,
	};

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
	};

	//! Pivot symbol visibility
	enum PivotVisibility {  PIVOT_HIDE,
							PIVOT_SHOW_ON_MOVE,
							PIVOT_ALWAYS_SHOW,
	};

	//! Default constructor
	ccGLWindow(	QWidget *parent = 0,
				const QGLFormat& format = QGLFormat::defaultFormat(),
				QGLWidget* shareWidget = 0,
				bool silentInitialization = false);
	
	//! Destructor
	virtual ~ccGLWindow();

	//! Sets 'scene graph' root
	void setSceneDB(ccHObject* root);

	//! Returns current 'scene graph' root
	ccHObject* getSceneDB();

	//inherited from ccGenericGLDisplay
	virtual void toBeRefreshed();
	virtual void refresh();
	virtual void invalidateViewport();
	virtual unsigned getTexture(const QImage& image);
	virtual void releaseTexture(unsigned texID);
	virtual void display3DLabel(const QString& str, const CCVector3& pos3D, const unsigned char* rgbColor = 0, const QFont& font = QFont());
	virtual bool supportOpenGLVersion(unsigned openGLVersionFlag);
	virtual void displayText(QString text, int x, int y, unsigned char align = ALIGN_DEFAULT, float bkgAlpha = 0, const unsigned char* rgbColor = 0, const QFont* font = 0);
	virtual QFont getTextDisplayFont() const; //takes rendering zoom into account!
	virtual const ccViewportParameters& getViewportParameters() const { return m_viewportParams; }
	inline virtual void makeContextCurrent() { makeCurrent(); }
	virtual void setupProjectiveViewport(const ccGLMatrixd& cameraMatrix, float fov_deg = 0.0f, float ar = 1.0f, bool viewerBasedPerspective = true, bool bubbleViewMode = false);

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
	virtual void setPivotPoint(const CCVector3d& P);

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
	virtual void updateConstellationCenterAndZoom(const ccBBox* aBox = 0);

	//! Returns the visible objects bounding-box
	ccBBox getVisibleObjectsBB() const;

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
	virtual const void setBaseViewMat(ccGLMatrixd& mat);

	//! Returns the current (OpenGL) view matrix as a double array
	/** Warning: different from 'view' matrix returned by getBaseViewMat.
	**/
	virtual const double* getModelViewMatd();
	//! Returns the current (OpenGL) projection matrix as a double array
	virtual const double* getProjectionMatd();
	//! Returns the current viewport (OpenGL int[4] array)
	virtual void getViewportArray(int vp[/*4*/]);

	//! Sets camera to a predefined view (top, bottom, etc.)
	virtual void setView(CC_VIEW_ORIENTATION orientation, bool redraw = true);

	//! Sets camera to a custom view (forward and up directions must be specified)
	virtual void setCustomView(const CCVector3d& forward, const CCVector3d& up, bool forceRedraw = true);

	//! Sets current interaction mode
	virtual void setInteractionMode(INTERACTION_MODE mode);

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
	static const unsigned MIN_POINT_SIZE = 1;
	//! Maximum point size
	static const unsigned MAX_POINT_SIZE = 10;

	//! Sets point size
	/** \param size point size (between MIN_POINT_SIZE and MAX_POINT_SIZE)
	**/
	virtual void setPointSize(float size);

	//! Sets line width
	/** \param width lines width (typically between 1 and 10)
	**/
	virtual void setLineWidth(float width);

	//! Returns current font size
	virtual int getFontPointSize() const;

	//! Returns window own DB (2D objects only)
	virtual ccHObject* getOwnDB();
	//! Adds an entity to window own DB (2D objects only)
	/** By default no dependency link is established between the entity and the window (DB).
	**/
	virtual void addToOwnDB(ccHObject* obj2D, bool noDependency = true);
	//! Removes an entity from window own DB (2D objects only)
	virtual void removeFromOwnDB(ccHObject* obj2D);

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

	//! Renders screen to a file
	virtual bool renderToFile(	const char* filename,
								float zoomFactor = 1.0,
								bool dontScaleFeatures = false,
								bool renderOverlayItems = false);

	virtual void setShader(ccShader* shader);
	virtual void setGlFilter(ccGlFilter* filter);
	ccGlFilter* getGlFilter() { return m_activeGLFilter; }

	virtual bool areShadersEnabled() const;
	virtual bool areGLFiltersEnabled() const;

	//! Enables "embedded icons"
	virtual void enableEmbeddedIcons(bool state);

	//! Returns the actual pixel size on screen (taking zoom or perspective parameters into account)
	/** In perspective mode, this value is approximate.
	**/
	virtual double computeActualPixelSize() const;

	//! Returns the zoom value equivalent to the current camera position (perspective only)
	float computePerspectiveZoom() const;

	//! Returns whether the ColorRamp shader is supported or not
	bool hasColorRampShader() const { return m_colorRampShader != 0; }

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
	/** Warning: may return overriden parameters!
	**/
	const ccGui::ParamStruct& getDisplayParameters() const;

	//! Sets current parameters for this display
	void setDisplayParameters(const ccGui::ParamStruct& params, bool thisWindowOnly = false)
	{
		if (thisWindowOnly)
		{
			m_overridenDisplayParametersEnabled = true;
			m_overridenDisplayParameters = params;
		}
		else
		{
			m_overridenDisplayParametersEnabled = false;
			ccGui::Set(params);
		}
	}

	//! Whether display parameters are overidden for this window
	bool hasOverridenDisplayParameters() const { return m_overridenDisplayParametersEnabled; }

	//! Sets whether overlay entities (scale, tetrahedron, etc.) should be displayed or not
	void displayOverlayEntities(bool state) { m_displayOverlayEntities = state; }

	//! Returns whether overlay entities (scale, tetrahedron, etc.) are displayed or not
	bool overlayEntitiesAreDisplayed() const { return m_displayOverlayEntities; }

	//! Locks the manual rotation around the vertical (screen) axis
	void lockVerticalRotation(bool state) { m_verticalRotationLocked = state; }

	//! Returns whether the manual rotation around the vertical (screen) axis is locked or not
	bool isVerticalRotationLocked() const { return m_verticalRotationLocked; }

public slots:

	void zoomGlobal();
	void testFrameRate();

	//inherited from ccGenericGLDisplay
	virtual void redraw();

	//called when recieving mouse wheel is rotated
	void onWheelEvent(float wheelDelta_deg);

signals:

	//! Signal emitted when an entity is selected in the 3D view
	void entitySelectionChanged(int uniqueID);
	//! Signal emitted when multiple entities are selected in the 3D view
	void entitiesSelectionChanged(std::set<int> entIDs);

	//! Signal emitted in point picking mode to declare picking of a given point
	/** \param cloudUniqueID cloud unique ID
		\param pointIndex point index in cloud
		\param x mouse cursor x position
		\param y mouse cursor y position
	**/
	void pointPicked(int cloudUniqueID, unsigned pointIndex, int x, int y);

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
	void pixelSizeChanged(float);

	//! Signal emitted when the f.o.v. changes
	void fovChanged(float);

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

	//! Signal emitted when the left mouse button is cliked on the window
	/** Arguments correspond to the clicked point coordinates (x,y) in
		pixels and relatively to the window center.
	**/
	void leftButtonClicked(int, int);

	//! Signal emitted when the right mouse button is cliked on the window
	/** Arguments correspond to the clicked point coordinates (x,y) in
		pixels and relatively to the window center.
	**/
	void rightButtonClicked(int, int);

	//! Signal emitted when the mouse is moved
	/** SEGMENT_ENTITY mode only with either a button pressed or m_alwaysUseFBO=true! (too slow otherwise)
		Two first arguments correspond to the current cursor coordinates (x,y)
		in pixels and relatively to the window center.
	**/
	void mouseMoved(int, int, Qt::MouseButtons);

	//! Signal emitted when a mouse button is released (cursor on the window)
	/** SEGMENT_ENTITY mode only!
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

protected:

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
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);
	void closeEvent(QCloseEvent *event);

	//inherited
	void initializeGL();
	void resizeGL(int w, int h);
	void paintGL();

	//! main OpenGL loop
	void draw3D(CC_DRAW_CONTEXT& context, bool doDrawCross, ccFrameBufferObject* fbo = 0);

	//Graphical features controls
	void drawCross();
	void drawTrihedron();
	void drawGradientBackground();
	void drawScale(const colorType color[] = ccColor::white);

	//Projections controls
	void recalcModelViewMatrix();
	void recalcProjectionMatrix();
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

	//! Stops frame rate test
	void stopFrameRateTest();

	//inherited from QWidget (drag & drop support)
	virtual void dragEnterEvent(QDragEnterEvent* event);
	virtual void dropEvent(QDropEvent* event);

	//! Starts OpenGL picking process
	/** \param mode picking mode
		\param centerX picking area center X position
		\param centerY picking area center y position
		\param width picking area width
		\param height picking area height
		\param[out] subID [optional] poiter to store sub item ID (if any - <1 otherwise)
		\return item ID (if any) or <1 otherwise
	**/
	int startPicking(PICKING_MODE mode, int centerX, int centerY, int width = 5, int height = 5, int* subID = 0);
	
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
	//! Releases any active FBO
	void removeFBO();

	//! Inits active GL filter (advanced shader)
	bool initGLFilter(int w, int h);
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

	/***************************************************
					OpenGL Extensions
	***************************************************/

	//! Loads OpenGL extensions
	/** Wrapper around ccFBOUtils::InitGLEW.
		\return success
	**/
	static bool InitGLEW();

	//! GL names picking buffer
	GLuint m_pickingBuffer[CC_PICKING_BUFFER_SIZE];

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
	//! Last mouse orientation
	CCVector3d m_lastMouseOrientation;
	//! Current mouse orientation
	CCVector3d m_currentMouseOrientation;

	//! Complete visualization matrix (GL style - double version)
	double m_viewMatd[OPENGL_MATRIX_SIZE];
	//! Whether the model veiw matrix is valid (or need to be recomputed)
	bool m_validModelviewMatrix;
	//! Projection matrix (GL style - double version)
	double m_projMatd[OPENGL_MATRIX_SIZE];
	//! Whether the projection matrix is valid (or need to be recomputed)
	bool m_validProjectionMatrix;

	//! GL context width
	int m_glWidth;
	//! GL context height
	int m_glHeight;

	//! L.O.D. (level of detail) display mode
	bool m_lodActivated;
	//! Whether the display should be refreshed on next call to 'refresh'
	bool m_shouldBeRefreshed;
	//! Whether the mouse cursor has moved after being pressed or not
	bool m_cursorMoved;
	//! Whether this 3D window can be closed by the user or not
	bool m_unclosable;
	//! Current intercation mode (with mouse)
	INTERACTION_MODE m_interactionMode;
	//! Current picking mode
	PICKING_MODE m_pickingMode;
	//! Whether picking mode is locked or not
	bool m_pickingModeLocked;

	//! Display capturing mode options
	struct CaptureModeOptions
	{
		bool enabled;
		float zoomFactor;
		bool renderOverlayItems;

		//! Default constructor
		CaptureModeOptions()
			: enabled(false)
			, zoomFactor(1.0f)
			, renderOverlayItems(false)
		{}
	};

	//! Display capturing mode options
	CaptureModeOptions m_captureMode;

	//! Temporary Message to display in the lower-left corner
	struct MessageToDisplay
	{
		//! Message
		QString message;
		//! Message end time (sec)
		int messageValidity_sec;
		//! Message position on screen
		MessagePosition position;
		//! Message type
		MessageType type;
	};

	//! List of messages to display
	std::list<MessageToDisplay> m_messagesToDisplay;

	//! Last click time (msec)
	int m_lastClickTime_ticks;

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

	//! Whether embedded icons (point size, etc.) are enabled or not
	bool m_embeddedIconsEnabled;
	//! Hot zone (= where embedded icons lie) is activated (= mouse over)
	bool m_hotZoneActivated;

	//! Clickable item
	struct ClickableItem
	{
		enum Role {	NO_ROLE,
					INCREASE_POINT_SIZE,
					DECREASE_POINT_SIZE,
					LEAVE_BUBBLE_VIEW_MODE,
		};

		Role role;
		QRect area;
		
		ClickableItem() : role(NO_ROLE) {}
		ClickableItem(Role _role, QRect _area) : role(_role), area(_area) {}
	};
	//! Currently displayed clickable items
	std::vector<ClickableItem> m_clickableItems;

	//! Currently active shader
	ccShader* m_activeShader;
	//! Whether shaders are enabled or not
	bool m_shadersEnabled;

	//! Currently active FBO (frame buffer object)
	ccFrameBufferObject* m_fbo;
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

	//! Overriden display parameter 
	ccGui::ParamStruct m_overridenDisplayParameters;

	//! Whether display parameters are overidden for this window
	bool m_overridenDisplayParametersEnabled;

	//! Whether to display overlay entities or not (scale, tetrahedron, etc.)
	bool m_displayOverlayEntities;

	//! Whether initialization should be silent or not (i.e. no message to console)
	bool m_silentInitialization;

	//! To lock the manual rotation around the (screen) vertical axis
	bool m_verticalRotationLocked;

	//! Bubble-view mode state
	bool m_bubbleViewModeEnabled;
	//! Bubble-view mode f.o.v. (degrees)
	float m_bubbleViewFov_deg;
	//! Pre-bubble-view camera parameters (backup)
	ccViewportParameters m_preBubbleViewParameters;

private:

	//! Returns shaders path
	static QString getShadersPath();
};

#endif
