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
class bdr2Point5DEditor 
{
public:
	bdr2Point5DEditor();
	~bdr2Point5DEditor();
	ccGLWindow* getGLWindow() { return m_glWindow; }

	virtual void init2DView();
	//! Creates the 2D view
	virtual void create2DView(QFrame* parentFrame);

	//! Updates the 2D display zoom	// left bottom
	virtual void update2DDisplayZoom(ccBBox& box, CCVector3d up = CCVector3d(0, 1, 0));

	virtual void clearAll();
protected:
	//! 2D display
	ccGLWindow* m_glWindow;
};

class bdr2Point5DimEditor : public bdr2Point5DEditor
{
public:
	//! Default constructor
	bdr2Point5DimEditor();

	//! Destructor
	~bdr2Point5DimEditor();

	

public:
	
	virtual void init2DView() override;
	//! Creates the 2D view
	virtual void create2DView(QFrame* parentFrame) override;
	virtual void clearAll() override;

	void setAssociate3DView(ccGLWindow* win);
	ccGLWindow* getAssociate3DView() { return m_associate_3DView; }
	void destroyAss3DView() { m_associate_3DView = nullptr; }

	void updateCursorPos(const CCVector3d& P, bool b3d, bool move);
 
	//! left-bottom
	bool FromGlobalToImage(const CCVector3 & P_global, CCVector3 & P_local, bool withLensError = true);

	void setImage(QString image_path);
	void setImageAndCamera(ccCameraSensor* cam);
	ccImage* getImage() { return m_image; }

	ccHObject* projectToImage(ccHObject* obj);

	bool projectBack(ccHObject * obj2D, ccHObject * obj3D);

public: // buttons
	void ZoomFit();

protected: //members

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
