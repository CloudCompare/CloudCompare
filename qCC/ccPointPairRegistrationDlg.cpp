#include <ccPointPairRegistrationDlg.h>

//Local
#include <MainWindow.h>
#include <ccGLWindow.h>
#include <ccAskThreeDoubleValuesDlg.h>

//qCC_db
#include <ccGenericPointCloud.h>
#include <cc2DLabel.h>
#include <ccPointCloud.h>

//Qt
#include <QMdiSubWindow.h>

ccPointPairRegistrationDlg::ccPointPairRegistrationDlg(QWidget* parent/*=0*/)
	: ccOverlayDialog(parent)
	, m_aligned(0)
	, m_reference(0)
	, m_addedPoints(0)
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

	connect(validToolButton,		SIGNAL(clicked()),      this,   SLOT(apply()));
    connect(cancelToolButton,		SIGNAL(clicked()),      this,   SLOT(cancel()));
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

	m_alignPoints.clear();
	m_refPoints.clear();
}

bool ccPointPairRegistrationDlg::linkWith(ccGLWindow* win)
{
	ccGLWindow* oldWin = m_associatedWin;
	if (oldWin)
	{
		if (oldWin != win)
			disconnect(oldWin, SIGNAL(pointPicked(int, unsigned, int, int)), this, SLOT(processPickedPoint(int, unsigned, int, int)));

		if (m_addedPoints)
			oldWin->removeFromOwnDB(m_addedPoints);

		oldWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
	}

	if (!ccOverlayDialog::linkWith(win))
		return false;

	if (m_addedPoints)
		delete m_addedPoints;
	m_addedPoints=0;

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

		m_addedPoints = new ccPointCloud("added points");
		m_associatedWin->addToOwnDB(m_addedPoints);

		m_associatedWin->displayNewMessage("(you can add points 'manually' if necessary)",ccGLWindow::LOWER_LEFT_MESSAGE,false,3600);
		m_associatedWin->displayNewMessage("Pick equivalent points on both clouds (same order!)",ccGLWindow::LOWER_LEFT_MESSAGE,true,3600);
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
	bool canAlign = (m_alignPoints.size()==m_refPoints.size() && m_refPoints.size()>2); //we need at least 3 points
	alignToolButton->setEnabled(canAlign);
	validToolButton->setEnabled(canAlign);

	unstackAlignToolButton->setEnabled(!m_alignPoints.empty());
	unstackRefToolButton->setEnabled(!m_refPoints.empty());
}

bool ccPointPairRegistrationDlg::addAlignedPoint(const CCVector3& P, ccGenericPointCloud* cloud/*=0*/, unsigned pointIndex/*=0*/)
{
	for (unsigned i=0;i<m_alignPoints.size();++i)
	{
		if ((m_alignPoints[i]-P).norm() < ZERO_TOLERANCE)
		{
			ccLog::Error("Point already picked or too close to an already inserted one!");
			return false;
		}
	}

	//add point to 'align' set
	m_alignPoints.push_back(P);
	
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

	//add a fake point to 'm_addedPoints' if point is manually added
	if (!cloud)
	{
		cloud = m_addedPoints;
		pointIndex = m_addedPoints->size();
		m_addedPoints->reserve(pointIndex+1);
		m_addedPoints->addPoint(P);
	}

	//add a label!
	cc2DLabel* label = new cc2DLabel();
	label->addPoint(cloud,pointIndex);
	label->setName(pointName);
	label->setVisible(true);
	label->setDisplayedIn2D(false);
	label->setDisplayedIn3D(true);
	m_aligned.labels.push_back(label);

	if (m_associatedWin)
		m_associatedWin->addToOwnDB(label);

	onPointCountChanged();

	return true;
}

bool ccPointPairRegistrationDlg::addReferencePoint(const CCVector3& P, ccGenericPointCloud* cloud/*=0*/, unsigned pointIndex/*=0*/)
{
	for (unsigned i=0;i<m_refPoints.size();++i)
	{
		if ((m_refPoints[i]-P).norm() < ZERO_TOLERANCE)
		{
			ccLog::Error("Point already picked or too close to an already inserted one!");
			return false;
		}
	}

	//add point to 'reference' set
	m_refPoints.push_back(P);
	
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

	//add a fake point to 'm_addedPoints' if point is manually added
	if (!cloud)
	{
		cloud = m_addedPoints;
		pointIndex = m_addedPoints->size();
		m_addedPoints->reserve(pointIndex+1);
		m_addedPoints->addPoint(P);
	}

	//add a label!
	cc2DLabel* label = new cc2DLabel();
	label->addPoint(cloud,pointIndex);
	label->setName(pointName);
	label->setVisible(true);
	label->setDisplayedIn2D(false);
	label->setDisplayedIn3D(true);
	m_reference.labels.push_back(label);

	if (m_associatedWin)
		m_associatedWin->addToOwnDB(label);

	onPointCountChanged();

	return true;
}

void ccPointPairRegistrationDlg::unstackAligned()
{
}

void ccPointPairRegistrationDlg::unstackRef()
{
	if (m_refPoints.empty()) //nothing to do
		return;

	assert(refPointsTableWidget->rowCount()>0);
	refPointsTableWidget->removeRow(refPointsTableWidget->rowCount()-1);

	//remove label
	assert(!m_reference.labels.empty());
	cc2DLabel* label = m_reference.labels.back();
	if (label)
	{
		if (m_associatedWin)
		{
			m_associatedWin->removeFromOwnDB(label);
			m_associatedWin->redraw();
		}
		//remove added point as well?
		//if (label->getPoint(0).cloud == m_addedPoints)
		//{
		//	//not so easy (we can leave it ;)
		//}

		delete label;
		label=0;
	}
	m_reference.labels.pop_back();

	onPointCountChanged();
}

void ccPointPairRegistrationDlg::showAlignedCloud(bool state)
{
	if (!m_aligned.cloud)
		return;

	m_aligned.cloud->setVisible(state);
	for (unsigned i=0;i<m_aligned.labels.size();++i)
		m_aligned.labels[i]->setVisible(state);

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
	for (unsigned i=0;i<m_reference.labels.size();++i)
		m_reference.labels[i]->setVisible(state);

	if (m_associatedWin)
	{
		if (autoZoomCheckBox->isChecked())
			m_associatedWin->zoomGlobal();
		m_associatedWin->redraw();
	}
}

void ccPointPairRegistrationDlg::apply()
{
	align();
	
	if (m_aligned.cloud)
		m_aligned.cloud->applyGLTransformation_recursive();

	stop(true);
}

void ccPointPairRegistrationDlg::align()
{
	//TODO
}

void ccPointPairRegistrationDlg::cancel()
{
	stop(false);
}
