#include <ccPointPairRegistrationDlg.h>

//Local
#include <mainwindow.h>
#include <ccGLWindow.h>
#include <ccAskThreeDoubleValuesDlg.h>

//qCC_db
#include <ccGenericPointCloud.h>
#include <cc2DLabel.h>
#include <ccPointCloud.h>

//Qt
#include <QMdiSubWindow>

ccPointPairRegistrationDlg::ccPointPairRegistrationDlg(QWidget* parent/*=0*/)
	: ccOverlayDialog(parent)
	, m_aligned(0)
	, m_alignedPoints("aligned points")
	, m_reference(0)
	, m_refPoints("reference points")
{
	setupUi(this);
    setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);

	connect(showAlignedCheckBox,	SIGNAL(toggled(bool)),	this,	SLOT(showAlignedCloud(bool)));
	connect(showReferenceCheckBox,	SIGNAL(toggled(bool)),	this,	SLOT(showReferenceCloud(bool)));

	connect(typeAlignToolButton,	SIGNAL(clicked()),		this,	SLOT(addManualAlignedPoint()));
	connect(typeRefToolButton,		SIGNAL(clicked()),		this,	SLOT(addManualRefPoint()));

	connect(unstackAlignToolButton,	SIGNAL(clicked()),		this,	SLOT(unstackAligned()));
	connect(unstackRefToolButton,	SIGNAL(clicked()),		this,	SLOT(unstackRef()));

	connect(alignToolButton,		SIGNAL(clicked()),      this,   SLOT(align()));
	connect(resetToolButton,		SIGNAL(clicked()),      this,   SLOT(reset()));

	connect(validToolButton,		SIGNAL(clicked()),      this,   SLOT(apply()));
    connect(cancelToolButton,		SIGNAL(clicked()),      this,   SLOT(cancel()));

	m_alignedPoints.setEnabled(true);
	m_alignedPoints.setVisible(false);

	m_refPoints.setEnabled(true);
	m_refPoints.setVisible(false);
}

ccPointPairRegistrationDlg::cloudContext::cloudContext(ccGenericPointCloud* entity)
	: cloud(entity)
	, originalDisplay(entity ? entity->getDisplay() : 0)
	, wasVisible(entity ? entity->isVisible() : false)
	, wasEnabled(entity ? entity->isEnabled() : false)
	, wasSelected(entity ? entity->isSelected() : false)
{
}

void ccPointPairRegistrationDlg::cloudContext::restore()
{
	if (!cloud)
		return;

	cloud->setDisplay(originalDisplay);
	if (originalDisplay)
		originalDisplay->redraw();
	cloud->setVisible(wasVisible);
	cloud->setEnabled(wasEnabled);
	cloud->setSelected(wasSelected);
}

void ccPointPairRegistrationDlg::clear()
{
	alignToolButton->setEnabled(false);
	validToolButton->setEnabled(false);

	while (alignedPointsTableWidget->rowCount()!=0)
		alignedPointsTableWidget->removeRow(alignedPointsTableWidget->rowCount()-1);
	while (refPointsTableWidget->rowCount()!=0)
		refPointsTableWidget->removeRow(refPointsTableWidget->rowCount()-1);

	m_alignedPoints.removeAllChildren();
	m_alignedPoints.clear();
	m_aligned.cloud = 0;
	m_refPoints.removeAllChildren();
	m_refPoints.clear();
	m_reference.cloud = 0;
}

bool ccPointPairRegistrationDlg::linkWith(ccGLWindow* win)
{
	ccGLWindow* oldWin = m_associatedWin;
	if (oldWin)
	{
		if (oldWin != win)
			disconnect(oldWin, SIGNAL(pointPicked(int, unsigned, int, int)), this, SLOT(processPickedPoint(int, unsigned, int, int)));

		oldWin->removeFromOwnDB(&m_alignedPoints);
		m_alignedPoints.setDisplay(0);
		oldWin->removeFromOwnDB(&m_refPoints);
		m_refPoints.setDisplay(0);

		oldWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
	}

	if (!ccOverlayDialog::linkWith(win))
		return false;

	m_aligned.restore();
	m_reference.restore();

	if (oldWin && MainWindow::TheInstance())
	{
		QMdiSubWindow* subWindow = MainWindow::TheInstance()->getMDISubWindow(oldWin);
		if (subWindow)
			subWindow->close();
	}

	if (m_associatedWin)
	{
		m_associatedWin->setPickingMode(ccGLWindow::POINT_PICKING);
		connect(m_associatedWin, SIGNAL(pointPicked(int, unsigned, int, int)), this, SLOT(processPickedPoint(int, unsigned, int, int)));

		m_associatedWin->addToOwnDB(&m_alignedPoints);
		m_associatedWin->addToOwnDB(&m_refPoints);

		m_associatedWin->displayNewMessage(QString(),ccGLWindow::LOWER_LEFT_MESSAGE);
		m_associatedWin->displayNewMessage("(you can add points 'manually' if necessary)",ccGLWindow::LOWER_LEFT_MESSAGE,true,3600);
		m_associatedWin->displayNewMessage("Pick equivalent points on both clouds (at least 3 pairs - mind the order)",ccGLWindow::LOWER_LEFT_MESSAGE,true,3600);
	}

	return true;
}

bool ccPointPairRegistrationDlg::start()
{
	assert(m_aligned.cloud);
	return ccOverlayDialog::start();
}

void ccPointPairRegistrationDlg::stop(bool state)
{
	//TODO

	ccOverlayDialog::stop(state);
}

void setEnabled_recursive(ccHObject* ent)
{
	assert(ent);
	ent->setEnabled(true);
	if (ent->getParent())
		setEnabled_recursive(ent->getParent());
}

//void setVisible_recursive(ccHObject* ent)
//{
//	assert(ent);
//	ent->setVisible(true);
//	if (ent->getParent())
//		setEnabled_recursive(ent->getParent());
//}

bool ccPointPairRegistrationDlg::init(ccGenericPointCloud* aligned, ccGenericPointCloud* reference/*=0*/)
{
	clear();

	assert(aligned);
	if (!aligned)
		return false;

	//create dedicated 3D view
	if (!m_associatedWin)
	{
		MainWindow* mainWin = MainWindow::TheInstance();
		ccGLWindow* win = (mainWin ? mainWin->new3DView() : 0);
		if (!win)
		{
			ccLog::Error("[ccPointPairRegistrationDlg] Failed to create dedicated 3D view!");
			return false;
		}
		linkWith(win);
		assert(m_associatedWin);
	}

	m_aligned = cloudContext(aligned);
	m_reference = cloudContext(reference);

	if (aligned->getDisplay())
	{
		const ccViewportParameters& vParams = aligned->getDisplay()->getViewportParameters();
		m_associatedWin->setViewportParameters(vParams);
	}

	//add aligned cloud to display
	if (aligned)
	{
		m_associatedWin->addToOwnDB(aligned);
		aligned->setVisible(true);
		aligned->setSelected(false);
		setEnabled_recursive(aligned);
		//setVisible_recursive(aligned);
	}

	//add reference cloud (if any) to display
	if (reference)
	{
		m_associatedWin->addToOwnDB(reference);
		reference->setVisible(true);
		reference->setSelected(false);
		setEnabled_recursive(reference);
		//setVisible_recursive(reference);
	}

	showReferenceCheckBox->setChecked(reference != 0);
	showReferenceCheckBox->setEnabled(reference != 0);
	showAlignedCheckBox->setChecked(true);

	m_associatedWin->showMaximized();
	m_associatedWin->zoomGlobal();
	m_associatedWin->redraw();
	m_associatedWin->displayNewMessage("[Point-pair registration]",ccGLWindow::UPPER_CENTER_MESSAGE,true,3600);

	onPointCountChanged();
	
	return true;
}

static double s_last_ax = 0;
static double s_last_ay = 0;
static double s_last_az = 0;
void ccPointPairRegistrationDlg::addManualAlignedPoint()
{
	ccAskThreeDoubleValuesDlg ptsDlg("x","y","z",-1.0e8,1.0e8,s_last_ax,s_last_ay,s_last_az,6,"Add aligned point",this);
	if (!ptsDlg.exec())
		return;

	//save values for current session
	s_last_ax = ptsDlg.doubleSpinBox1->value();
	s_last_ay = ptsDlg.doubleSpinBox2->value();
	s_last_az = ptsDlg.doubleSpinBox3->value();

	CCVector3 P(s_last_ax,
				s_last_ay,
				s_last_az);

	addAlignedPoint(P);
}

static double s_last_rx = 0;
static double s_last_ry = 0;
static double s_last_rz = 0;
void ccPointPairRegistrationDlg::addManualRefPoint()
{
	ccAskThreeDoubleValuesDlg ptsDlg("x","y","z",-1.0e8,1.0e8,s_last_rx,s_last_ry,s_last_rz,6,"Add reference point",this);
	if (!ptsDlg.exec())
		return;

	//save values for current session
	s_last_rx = ptsDlg.doubleSpinBox1->value();
	s_last_ry = ptsDlg.doubleSpinBox2->value();
	s_last_rz = ptsDlg.doubleSpinBox3->value();

	CCVector3 P(s_last_rx,
				s_last_ry,
				s_last_rz);

	addReferencePoint(P);
}

void ccPointPairRegistrationDlg::processPickedPoint(int cloudUniqueID, unsigned pointIndex, int, int)
{
	if (!m_associatedWin)
		return;

	ccHObject* db = m_associatedWin->getOwnDB();
	if (!db)
		return;

	ccHObject* obj = db->find(cloudUniqueID);
	if (obj && obj->isKindOf(CC_POINT_CLOUD))
	{
		ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(obj);
		const CCVector3* P = cloud->getPoint(pointIndex);

		if (P)
		{
			if (cloud == m_aligned.cloud)
				addAlignedPoint(*P,m_aligned.cloud,pointIndex);
			else if (cloud == m_reference.cloud)
				addReferencePoint(*P,m_reference.cloud,pointIndex);
			else
			{
				assert(false);
				return;
			}

			m_associatedWin->redraw();
		}
	}
}

void ccPointPairRegistrationDlg::onPointCountChanged()
{
	bool canAlign = (m_alignedPoints.size()==m_refPoints.size() && m_refPoints.size()>2); //we need at least 3 points
	alignToolButton->setEnabled(canAlign);
	validToolButton->setEnabled(canAlign);

	unstackAlignToolButton->setEnabled(m_alignedPoints.size()!=0);
	unstackRefToolButton->setEnabled(m_refPoints.size()!=0);
}

bool ccPointPairRegistrationDlg::addAlignedPoint(const CCVector3& P, ccGenericPointCloud* cloud/*=0*/, unsigned pointIndex/*=0*/)
{
	for (unsigned i=0;i<m_alignedPoints.size();++i)
	{
		if ((*m_alignedPoints.getPoint(i)-P).norm() < ZERO_TOLERANCE)
		{
			ccLog::Error("Point already picked or too close to an already inserted one!");
			return false;
		}
	}

	//add point to 'align' set
	unsigned newPointIndex = m_alignedPoints.size();
	if (newPointIndex==m_alignedPoints.capacity())
	{
		if (!m_alignedPoints.reserve(newPointIndex+1))
		{
			ccLog::Error("Not enough memory?!");
			return false;
		}
	}
	m_alignedPoints.addPoint(P);
	
	//add corresponding row in table
	int rowIndex = alignedPointsTableWidget->rowCount();
	QString pointName = QString("A%1").arg(rowIndex);
	alignedPointsTableWidget->setRowCount(rowIndex+1);
	for (int d=0;d<3;++d)
	{
		QTableWidgetItem* item = new QTableWidgetItem();
		item->setData(Qt::EditRole, P.u[d]); 
		alignedPointsTableWidget->setItem(rowIndex, d, item);
		alignedPointsTableWidget->setVerticalHeaderItem(rowIndex,new QTableWidgetItem(pointName));
	}

	//add a label!
	cc2DLabel* label = new cc2DLabel();
	label->addPoint(&m_alignedPoints,newPointIndex);
	label->setName(pointName);
	label->setVisible(true);
	label->setDisplayedIn2D(false);
	label->setDisplayedIn3D(true);
	label->setDisplay(m_associatedWin);
	m_alignedPoints.addChild(label);

	if (m_associatedWin)
		m_associatedWin->redraw();

	onPointCountChanged();

	return true;
}

bool ccPointPairRegistrationDlg::addReferencePoint(const CCVector3& P, ccGenericPointCloud* cloud/*=0*/, unsigned pointIndex/*=0*/)
{
	for (unsigned i=0;i<m_refPoints.size();++i)
	{
		if ((*m_refPoints.getPoint(i)-P).norm() < ZERO_TOLERANCE)
		{
			ccLog::Error("Point already picked or too close to an already inserted one!");
			return false;
		}
	}

	//add point to 'reference' set
	unsigned newPointIndex = m_refPoints.size();
	if (newPointIndex==m_refPoints.capacity())
	{
		if (!m_refPoints.reserve(newPointIndex+1))
		{
			ccLog::Error("Not enough memory?!");
			return false;
		}
	}
	m_refPoints.addPoint(P);
	
	//add corresponding row in table
	int rowIndex = refPointsTableWidget->rowCount();
	QString pointName = QString("R%1").arg(rowIndex);
	refPointsTableWidget->setRowCount(rowIndex+1);
	for (int d=0;d<3;++d)
	{
		QTableWidgetItem* item = new QTableWidgetItem();
		item->setData(Qt::EditRole, P.u[d]); 
		refPointsTableWidget->setItem(rowIndex, d, item);
		refPointsTableWidget->setVerticalHeaderItem(rowIndex,new QTableWidgetItem(pointName));
	}

	//add a label!
	cc2DLabel* label = new cc2DLabel();
	label->addPoint(&m_refPoints,newPointIndex);
	label->setName(pointName);
	label->setVisible(true);
	label->setDisplayedIn2D(false);
	label->setDisplayedIn3D(true);
	label->setDisplay(m_associatedWin);
	m_refPoints.addChild(label);

	if (m_associatedWin)
		m_associatedWin->redraw();

	onPointCountChanged();

	return true;
}

void ccPointPairRegistrationDlg::unstackAligned()
{
	unsigned pointCount = m_alignedPoints.size();
	if (pointCount==0) //nothing to do
		return;

	assert(alignedPointsTableWidget->rowCount()>0);
	alignedPointsTableWidget->removeRow(alignedPointsTableWidget->rowCount()-1);

	//remove label
	assert(m_alignedPoints.getChildrenNumber() == pointCount);
	m_alignedPoints.removeChild(pointCount-1);
	//remove point
	m_alignedPoints.resize(pointCount-1);

	if (m_associatedWin)
		m_associatedWin->redraw();

	onPointCountChanged();
}

void ccPointPairRegistrationDlg::unstackRef()
{
	unsigned pointCount = m_refPoints.size();
	if (pointCount==0) //nothing to do
		return;

	assert(refPointsTableWidget->rowCount()>0);
	refPointsTableWidget->removeRow(refPointsTableWidget->rowCount()-1);

	//remove label
	assert(m_refPoints.getChildrenNumber() == pointCount);
	m_refPoints.removeChild(pointCount-1);
	//remove point
	m_refPoints.resize(pointCount-1);

	if (m_associatedWin)
		m_associatedWin->redraw();

	onPointCountChanged();
}

void ccPointPairRegistrationDlg::showAlignedCloud(bool state)
{
	if (!m_aligned.cloud)
		return;

	m_aligned.cloud->setVisible(state);
	m_alignedPoints.setEnabled(state);

	if (m_associatedWin)
	{
		if (autoZoomCheckBox->isChecked())
			m_associatedWin->zoomGlobal();
		m_associatedWin->redraw();
	}
}

void ccPointPairRegistrationDlg::showReferenceCloud(bool state)
{
	if (!m_reference.cloud)
		return;

	m_reference.cloud->setVisible(state);
	m_refPoints.setEnabled(state);

	if (m_associatedWin)
	{
		if (autoZoomCheckBox->isChecked())
			m_associatedWin->zoomGlobal();
		m_associatedWin->redraw();
	}
}

bool ccPointPairRegistrationDlg::callHornRegistration(CCLib::HornRegistrationTools::ScaledTransformation& trans, double& rms)
{
	assert(m_aligned.cloud);
	if (!m_aligned.cloud)
		return false;

	if (m_alignedPoints.size() != m_refPoints.size() || m_refPoints.size() < 2)
	{
		assert(false);
		ccLog::Error("Need at least 3 points for each cloud (and the same number of points in both clouds)!");
		return false;
	}
	unsigned count = m_refPoints.size();

	//fixed scale?
	bool fixedScale = fixedScalecheckBox->isChecked();

	//call Horn registration method
	if (!CCLib::HornRegistrationTools::FindAbsoluteOrientation(&m_alignedPoints, &m_refPoints, trans, fixedScale))
	{
		ccLog::Error("Registration failed! (points are aligned?)");
		return false;
	}

	rms = CCLib::HornRegistrationTools::ComputeRMS(&m_alignedPoints, &m_refPoints, trans);

	return true;
}

void ccPointPairRegistrationDlg::align()
{
	CCLib::HornRegistrationTools::ScaledTransformation trans;
	double rms;
	if (callHornRegistration(trans,rms))
	{
		if (rms >= 0)
		{
			ccLog::Print(QString("[ccPointPairRegistrationDlg] Current RMS: %1").arg(rms));
			m_associatedWin->displayNewMessage(QString("RMS: %1").arg(rms),ccGLWindow::LOWER_LEFT_MESSAGE);
		}

		//fixed scale?
		bool fixedScale = fixedScalecheckBox->isChecked();

		//apply (scaled) transformation...
		if (!fixedScale)
		{
			ccLog::Print(QString("[ccPointPairRegistrationDlg] Scale: %1").arg(trans.s));
			if (trans.R.isValid())
				trans.R.scale(trans.s);
		}
		ccGLMatrix transMat(trans.R,trans.T);
		//...virtually
		m_aligned.cloud->setGLTransformation(transMat);
		m_alignedPoints.setGLTransformation(transMat);

		if (m_associatedWin)
		{
			if (autoZoomCheckBox->isChecked())
				m_associatedWin->zoomGlobal();
			m_associatedWin->redraw();
		}

		resetToolButton->setEnabled(true);
	}
}

void ccPointPairRegistrationDlg::reset()
{
	if (!m_aligned.cloud)
		return;

	m_aligned.cloud->enableGLTransformation(false);
	m_alignedPoints.enableGLTransformation(false);

	if (m_associatedWin)
	{
		if (autoZoomCheckBox->isChecked())
			m_associatedWin->zoomGlobal();
		m_associatedWin->redraw();
	}

	resetToolButton->setEnabled(false);
}

void ccPointPairRegistrationDlg::apply()
{
	CCLib::HornRegistrationTools::ScaledTransformation trans;
	double rms;
	if (callHornRegistration(trans,rms))
	{
		if (rms >= 0)
			ccLog::Print(QString("[ccPointPairRegistrationDlg] Current RMS: %1").arg(rms));

		//fixed scale?
		bool fixedScale = fixedScalecheckBox->isChecked();

		//apply (scaled) transformation...
		if (!fixedScale && trans.R.isValid())
			trans.R.scale(trans.s);
		ccGLMatrix transMat(trans.R,trans.T);
		//...acutally
		m_aligned.cloud->applyGLTransformation_recursive();
		m_alignedPoints.setGLTransformation(transMat);

		ccLog::Print("[PointPairRegistration] Applied matrix:");
		const float* mat = transMat.data();
		ccLog::Print("%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f",mat[0],mat[4],mat[8],mat[12],mat[1],mat[5],mat[9],mat[13],mat[2],mat[6],mat[10],mat[14],mat[3],mat[7],mat[11],mat[15]);
		if (!fixedScale && trans.s != 1.0)
			ccLog::Print(QString("[PointPairRegistration] Warning: scale = %1 (already integrated in above matrix!)").arg(trans.s));
	}

	stop(true);
}

void ccPointPairRegistrationDlg::cancel()
{
	if (m_aligned.cloud)
		m_aligned.cloud->enableGLTransformation(false);

	stop(false);
}
