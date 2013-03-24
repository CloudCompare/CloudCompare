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

//qCC
#include "ccCommon.h"
#include "ccGLUtils.h"

//Qt
#include <QFont>

//! OpenGL picking buffer size (= max number of entity per 'OpenGL' selection)
#define CC_PICKING_BUFFER_SIZE 65536

class ccHObject;
class ccBBox;
class ccCalibratedImage;
class ccShader;
class ccGlFilter;
class ccFrameBufferObject;
class cc2DLabel;
class QGLPixelBuffer;

#ifdef CC_USE_DB_ROOT_AS_SCENE_GRAPH
class ccDBRoot;
#endif

//! OpenGL 3D view
class ccGLWindow : public QGLWidget, public ccGenericGLDisplay
{
    Q_OBJECT

public:

	//! Picking mode
	enum PICKING_MODE { NO_PICKING,
						ENTITY_PICKING,
						LABELS_PICKING,
						POINT_PICKING,
						TRIANGLE_PICKING,
						AUTO_POINT_PICKING,
						DEFAULT_PICKING,
	};

	//! Interaction mode (with the mouse!)
	enum INTERACTION_MODE { TRANSFORM_CAMERA,
							TRANSFORM_ENTITY,
							SEGMENT_ENTITY,
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
	};

	//! Pivot symbol visibility
	enum PivotVisibility {  PIVOT_HIDE,
							PIVOT_SHOW_ON_MOVE,
							PIVOT_ALWAYS_SHOW,
	};

	//! Default constructor
    ccGLWindow(QWidget *parent = 0, const QGLFormat& format=QGLFormat::defaultFormat(), QGLWidget* shareWidget = 0);
	//! Default destructor
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
	virtual void display3DLabel(const QString& str, const CCVector3& pos3D, const unsigned char* rgbColor=0, const QFont& font=QFont());
	virtual bool supportOpenGLVersion(unsigned openGLVersionFlag);
    virtual void displayText(QString text, int x, int y, unsigned char align=ALIGN_DEFAULT, unsigned char bkgAlpha=0, const unsigned char* rgbColor=0, const QFont* font=0);
	virtual const QFont& getTextDisplayFont() {return m_font;}
	virtual const ccViewportParameters& getViewportParameters() const { return m_params; }

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
									int displayMaxDelay_sec=2,
									MessageType type=CUSTOM_MESSAGE);

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

	//! Sets pivot point
    virtual void setPivotPoint(const CCVector3& P);

	//! Sets camera position
    virtual void setCameraPos(const CCVector3& P);

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

	//! Center and zoom on a given bounding box
	/** If no bounding box is defined, the current displayed 'scene graph'
		bounding box is taken.
	**/
    virtual void updateConstellationCenterAndZoom(const ccBBox* aBox = 0);

    //! Rotates the base view matrix
	/** Warning: 'base view' marix is either:
		- the rotation around the object in object-centered mode
		- the rotation around the camera center in viewer-centered mode
		(see setPerspectiveState).
	**/
    virtual void rotateBaseViewMat(const ccGLMatrix& rotMat);

	//! Returns the base view matrix
	/** Warning: 'base view' marix is either:
		- the rotation around the object in object-centered mode
		- the rotation around the camera center in viewer-centered mode
		(see setPerspectiveState).
	**/
    virtual const ccGLMatrix& getBaseViewMat();
	
	//! Sets the base view matrix
	/** Warning: 'base view' marix is either:
		- the rotation around the object in object-centered mode
		- the rotation around the camera center in viewer-centered mode
		(see setPerspectiveState).
	**/
    virtual const void setBaseViewMat(ccGLMatrix& mat);

	//! Returns the current (OpenGL) view matrix as a double array
	/** Warning: different from 'view' matrix returned by getBaseViewMat.
	**/
    virtual const double* getModelViewMatd();
	//! Returns the current (OpenGL) projection matrix as a double array
    virtual const double* getProjectionMatd();
	//! Returns the current viewport (OpenGL int[4] array)
    virtual void getViewportArray(int vp[/*4*/]);

	//! Sets camera to a predefined view (top, bottom, etc.)
    virtual void setView(CC_VIEW_ORIENTATION orientation, bool redraw=true);

	//! Sets current interaction mode
	virtual void setInteractionMode(INTERACTION_MODE mode);

	//! Sets current picking mode
	/** Picking can be applied to entities (default), points, triangles, etc.)
	**/
    virtual void setPickingMode(PICKING_MODE mode = DEFAULT_PICKING);

	//! Specify whether this 3D window can be closed by the user or not
    virtual void setUnclosable(bool state);

	//! Returns context information
    virtual void getContext(CC_DRAW_CONTEXT& context);

    //! Sets point size
    /** \param size point size (typically between 1 and 10)
    **/
    virtual void setPointSize(float size);

    //! Sets line width
    /** \param width lines width (typically between 1 and 10)
    **/
    virtual void setLineWidth(float width);

	//! Returns current font size
    virtual int getFontPointSize() const;
	//! Sets current font size
    virtual void setFontPointSize(int pixelSize);

	//! Returns window own DB (2D objects only)
    virtual ccHObject* getOwnDB();
	//! Adds an entity to window own DB (2D objects only)
	virtual void addToOwnDB(ccHObject* obj2D);
	//! Removes an entity from window own DB (2D objects only)
	virtual void removeFromOwnDB(ccHObject* obj2D);

	//! Sets viewport parameters (all at once)
	virtual void setViewportParameters(const ccViewportParameters& params);

	//! Applies the same camera parameters as a given calibrated image
    virtual void applyImageViewport(ccCalibratedImage* image);
	
	//! Sets current camera f.o.v. (field of view)
	/** FOV is only used in perspective mode.
	**/
    virtual void setFov(float fov);

	//! Invalidate current visualization state
	/** Forces view matrix update and 3D/FBO display.
	**/
    virtual void invalidateVisualization();

    virtual bool renderToFile(const char* filename, float zoomFactor=1.0, bool dontScaleFeatures=false);

    virtual void setShader(ccShader* shader);
    virtual void setGlFilter(ccGlFilter* filter);

    virtual bool areShadersEnabled() const;
    virtual bool areGLFiltersEnabled() const;

	//! Enables "embedded icons"
	virtual void enableEmbeddedIcons(bool state);

	//! Returns the actual pixel size on screen (taking zoom or perspective parameters into account)
	/** In perspective mode, this value is approximate.
	**/
	virtual float computeActualPixelSize() const;

public slots:
    void zoomGlobal();
    void testFrameRate();

	//inherited from ccGenericGLDisplay
    virtual void redraw();

	//called when recieving muse wheel is rotated
	void onWheelEvent(float wheelDelta_deg);

signals:

	//! Signal emitted when an entity is selected in the 3D view
    void entitySelectionChanged(int uniqueID);

    //! Signal emitted in point picking mode to declare picking of a given point
    /** \param cloudUniqueID cloud unique ID
        \param pointIndex point index in cloud
		\param x mouse cursor x position
		\param y mouse cursor y position
    **/
    void pointPicked(int cloudUniqueID, unsigned pointIndex, int x, int y);

	/*** Camera link mode (interactive modifications of the view/camera are echoed to other windows) ***/

	//! Signal emitted when the window 'model view' matrix is interactively changed
    void viewMatRotated(const ccGLMatrix& rotMat);
	//! Signal emitted when the camera is interactively displaced
    void cameraDisplaced(float ddx, float ddy);
	//! Signal emitted when the mouse wheel is rotated
    void mouseWheelRotated(float wheelDelta_deg);

	//! Signal emitted when the perspective state changes (see setPerspectiveState)
	void perspectiveStateChanged();

	//! Signal emitted when the window 'base view' matrix is changed
    void baseViewMatChanged(const ccGLMatrix& newViewMat);

	//! Signal emitted when the pivot point is changed
	void pivotPointChanged(const CCVector3&);

	//! Signal emitted when the camera position is changed
	void cameraPosChanged(const CCVector3&);

    //! Signal emitted when the selected object is translated by the user
    void translation(const CCVector3& t);

    //! Signal emitted when the selected object is rotated by the user
	/** \param rotMat rotation applied to current viewport (4x4 OpenGL matrix)
	**/
    void rotation(const ccGLMatrix& rotMat);

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

    //events handling
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent* event);
    void closeEvent(QCloseEvent *event);

    //inherited
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

	//! main OpenGL loop
	void draw3D(CC_DRAW_CONTEXT& context, bool doDrawCross, ccFrameBufferObject* fbo = 0);

    //Graphical features controls
    void drawCross();
    void drawAxis();
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

    //! Returns current viewing direction
	/** This is the direction normal to the screen
		(pointing 'inside') in world base.
	**/
    CCVector3 getCurrentViewDir() const;

    //! Starts OpenGL picking process
	/** \param cursorX cursor x position
		\param cursorY cursor y position
		\param mode picking mode
		\return item ID (if any) or <1 otherwise
	**/
    int startPicking(int cursorX, int cursorY, PICKING_MODE mode);
	
	//! Processes hits in selection mode buffer (GL_SELECT)
	/** \param hits number of hits returned by glRenderMode after 'names' rendering
		\param[out] entID entity unique ID (or -1 if none found)
		\param[out] subCompID entity sub-component ID (or -1 if none found)
	**/
    void processHits(GLint hits, int& entID, int& subCompID);

	//! Updates currently active labels list (m_activeLabels)
	/** The labels must be currently displayed in this context
		AND at least one of them must be under the mouse cursor.
	**/
	void updateActiveLabelsList(int x, int y, bool extendToSelectedLabels=false);

	//! Currently active labels
	/** Active labels can be moved with mouse, etc.
	**/
	std::vector<cc2DLabel*> m_activeLabels;

	//! Inits FBO (frame buffer object)
    bool initFBO(int w, int h);
	//! Releases any active FBO
    void removeFBO();

	//! Inits active GL filter (advanced shader)
    bool initGLFilter(int w, int h);
	//! Releases active GL filter
    void removeGLFilter();

	//! Returns the zoom value equivalent to the current camera position (perspective only)
	float computePerspectiveZoom() const;

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
	ccViewportParameters m_params;

    //! Default font size
    int m_defaultFontPixelSize;

    //! Last mouse position
    QPoint m_lastMousePos;
    //! Last mouse orientation
    CCVector3 m_lastMouseOrientation;
    //! Current mouse orientation
    CCVector3 m_currentMouseOrientation;

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

	//! Display capturing mode
    bool m_captureMode;
	//! Display capturing mode zoom factor
	float m_captureModeZoomFactor;

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
	//! Hot zone "plus" button ROI
	int m_hotZonePlusIconROI[4];
	//! Hot zone "minus" button ROI
	int m_hotZoneMinusIconROI[4];

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

    //! Active GL filter
    ccGlFilter* m_activeGLFilter;
	//! Whether GL filters are enabled or not
    bool m_glFiltersEnabled;

	//! Window own DB
	ccHObject* m_winDBRoot;

    //! CC main DB
    ccHObject* m_globalDBRoot;

	//! Associated VBO (vertex buffer object)
	vboStruct m_vbo;

	//! Default font
	QFont m_font;
   
	//! Pivot symbol visibility
	PivotVisibility m_pivotVisibility;

	//! Whether pivot symbol should be shown or not
	bool m_pivotSymbolShown;

private:

	//! Returns shaders path
	static QString getShadersPath();
};

#endif
