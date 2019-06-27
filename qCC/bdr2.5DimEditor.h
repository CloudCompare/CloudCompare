#ifndef BDR_2_5D_EDITOR_HEADER
#define BDR_2_5D_EDITOR_HEADER

//qCC_db
class ccGLWindow;
class QWidget;
class QFrame;
class QComboBox;
class ccImage;
class ccCameraSensor;
class ccBBox;
#include "ccPolyline.h"
#define IMAGE_MARKER_DISPLAY_Z 1

//! 2.5D data editor (generic interface)
class bdr2Point5DimEditor
{
public:
	//! Default constructor
	bdr2Point5DimEditor();

	//! Destructor
	virtual ~bdr2Point5DimEditor();

	void clearAll();

public: //standard methods

	//! Updates the 2D display zoom	// left bottom
	virtual void update2DDisplayZoom(ccBBox& box, CCVector3d up = CCVector3d(0, 1, 0));

public:
	//! Creates the 2D view
	void create2DView(QFrame* parentFrame);

	void setAssociate3DView(ccGLWindow* win);
	ccGLWindow* getAssociate3DView() { return m_associate_3DView; }
	void destroyAss3DView() { m_associate_3DView = nullptr; }

	void updateCursorPos(const CCVector3d& P, bool b3d, bool move);
 
	//! left-bottom
	bool FromGlobalToImage(const CCVector3 & P_global, CCVector3 & P_local, bool withLensError = true);

	void init2DView();

	void setImage(QString image_path);
	void setImageAndCamera(ccCameraSensor* cam);
	ccImage* getImage() { return m_image; }

	ccGLWindow* getGLWindow() { return m_glWindow; }

	void projectToImage(ccHObject* obj);

public: // buttons
	void ZoomFit();

protected: //members

	//! 2D display
	ccGLWindow* m_glWindow;

	//! associate 3d view
	ccGLWindow* m_associate_3DView;

	ccImage* m_image;
	
	//! cursor
	//			1	
	//			|
	//		2---0---4
	//			|
	//			3
	ccPolyline* m_cursor_cross;
};

#endif //BDR_2_5D_EDITOR_HEADER
