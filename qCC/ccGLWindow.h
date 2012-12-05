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
//$Rev:: 2208                                                              $
//$LastChangedDate:: 2012-07-17 12:26:31 +0200 (mar., 17 juil. 2012)       $
//**************************************************************************
//

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
						POINT_PICKING,
						AUTO_POINT_PICKING,
						TRIANGLE_PICKING,
						LABELS_PICKING,
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
    //virtual void redraw();
    virtual void invalidateViewport();
    virtual unsigned getTexture(const QImage& image);
    virtual void releaseTexture(unsigned texID);
	virtual void display3DLabel(const QString& str, const CCVector3& pos3D, const unsigned char* rgbColor=0, const QFont& font=QFont());
	virtual bool supportOpenGLVersion(unsigned openGLVersionFlag);
    virtual void displayText(QString text, int x, int y, bool alignRight=false, const unsigned char* rgbColor=0, const QFont& font=QFont());
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

    virtual void setZoom(float value);
    virtual void setPivotPoint(float x, float y, float z);
    //virtual void setCameraPos(float x, float y, float z);

    virtual void setSunLight(bool state);
    virtual void toggleSunLight();
	virtual bool sunLightEnabled() const {return m_sunLightEnabled;}
    virtual void setCustomLight(bool state);
    virtual void toggleCustomLight();
	virtual bool customLightEnabled() const {return m_customLightEnabled;}

    virtual void setPerspectiveState(bool state, bool objectCenteredPerspective);
    virtual void togglePerspective(bool objectCentered);
    virtual bool getPerspectiveState(bool& objectCentered) const;

    //external camera control
    virtual void rotateViewMat(const ccGLMatrix& rotMat);
    virtual void updateZoom(float zoomFactor);
	virtual void setScreenPan(float tx, float ty);
    virtual void updateScreenPan(float dx, float dy); //dx and dy are added to actual screen pan!
    virtual void updateConstellationCenterAndZoom(const ccBBox* aBox = 0);

    virtual const ccGLMatrix& getBaseModelViewMat();
    virtual const void setBaseModelViewMat(ccGLMatrix& mat);
    virtual const double* getModelViewMatd();
    virtual const double* getProjectionMatd();
    virtual void getViewportArray(int vp[/*4*/]);

    virtual void setView(CC_VIEW_ORIENTATION orientation, bool redraw=true);

	//! Set interaction mode
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
    /** \param size point size (between 1 and 10)
    **/
    virtual void setPointSize(float size);

    //! Sets line width
    /** \param width width (between 1 and 10)
    **/
    virtual void setLineWidth(float width);

    virtual int getFontPointSize() const;
    virtual void setFontPointSize(int pixelSize);

	//! Returns window own DB (2D objects only)
    virtual ccHObject* getOwnDB();
	//! Adds an entity to window own DB (2D objects only)
	virtual void addToOwnDB(ccHObject* obj2D);
	//! Removes an entity from window own DB (2D objects only)
	virtual void removeFromOwnDB(ccHObject* obj2D);

	//! Sets viewport parameters (all at once)
	virtual void setViewportParameters(const ccViewportParameters& params);

    virtual void applyImageViewport(ccCalibratedImage* theImage);
    virtual void setFov(float fov);

    virtual void invalidateVisualization();

    //! Returns camera position (taking zoom into account if centered perspective is 'on')
    virtual CCVector3 computeCameraPos() const;

    virtual bool renderToFile(const char* filename, float zoomFactor=1.0, bool dontScaleFeatures=false);

    virtual void setShader(ccShader* shader);
    virtual void setGlFilter(ccGlFilter* filter);

    virtual bool areShadersEnabled() const;
    virtual bool areGLFiltersEnabled() const;

    virtual float computeTotalZoom() const;

	//! Enables "embedded icons"
	virtual void enableEmbeddedIcons(bool state);

public slots:
    void zoomGlobal();
    void testFrameRate();

	//inherited from ccGenericGLDisplay
    virtual void redraw();

signals:

	//! Signal emitted when an entity is selected in the 3D view
    void entitySelectionChanged(int uniqueID);

    //! Signal emitted in point picking mode to declare picking of a given point
    /** \param cloudID cloud unique ID
        \param pointIndex point index in cloud
		\parm x mouse cursor x position
		\parm y mouse cursor y position
    **/
    void pointPicked(int cloudUniqueID, unsigned pointIndex, int x, int y);

	//! Signal emitted when the window base view matrix is changed
    void viewMatRotated(const ccGLMatrix& rotMat);

	//! Signal emitted when the window base view matrix is changed
    void baseViewMatChanged(const ccGLMatrix& newBaseViewMat);

	//! Signal emitted when the pivot point is changed
	void pivotPointChanged(const CCVector3&);

	//! Signal emitted when the window view is zoomed
    void zoomChanged(float zoomFactor);

    //! Signal emitted when the window view is panned
    void panChanged(float ddx, float ddy);

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

    //Lights controls
    void glEnableSunLight();
    void glDisableSunLight();
    void glEnableCustomLight();
    void glDisableCustomLight();
    void displayCustomLight();

	//! Stops frame rate test
	void stopFrameRateTest();

	//inherited from QWidget (drag & drop support)
	virtual void dragEnterEvent(QDragEnterEvent* event);
	virtual void dropEvent(QDropEvent* event);

    //! Returns viewing direction (according to base view matrix)
    CCVector3 getBaseViewMatDir() const;

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

	//! GL names picking buffer
    GLuint m_pickingBuffer[CC_PICKING_BUFFER_SIZE];

    //! Unique ID
    int m_uniqueID;

	//! Initialization state
    bool m_initialized;

	//! Viewport parameters (zoom, etc.)
	ccViewportParameters m_params;

    //! Default font size
    int m_defaultFontPixelSize;
	//! Pivot point backup
	CCVector3 m_pivotPointBackup;

    //! Last mouse position
    QPoint m_lastMousePos;
    //! Last mouse orientation
    CCVector3 m_lastMouseOrientation;
    //! Current mouse orientation
    CCVector3 m_currentMouseOrientation;

	//! Complete visualization matrix (GL style - double version)
	double m_viewMatd[16];
	//! Whether the model veiw matrix is valid (or need to be recomputed)
    bool m_validModelviewMatrix;
	//! Projection matrix (GL style - double version)
	double m_projMatd[16];
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
};

#endif
