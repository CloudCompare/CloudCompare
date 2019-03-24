
#include "bdrFacetFilterDlg.h"

//Local
#include "ccPickingHub.h"

//qCC_db
#include <ccGLUtils.h>
#include <ccGenericMesh.h>
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>

//qCC_gl
#include <ccGLWidget.h>

//CCLib
#include <CCConst.h>
#include <GenericTriangle.h>

//Qt
#include <QMdiSubWindow>
#include <QtMath>

//local
#include "mainwindow.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

#include "stocker_parser.h"
#include "ccFacet.h"

bdrFacetFilterDlg::bdrFacetFilterDlg(/*ccGLWindow* win, ccHObject* _facet,*/ QWidget* parent/*=0*/)
	: QDialog(parent, Qt::Tool)
	, Ui::BDRFacetFilterDlg()
	, m_win(0)
	
{
	setupUi(this);
	connect(BDRFacetFilterDistSlider, &QAbstractSlider::valueChanged, this, &bdrFacetFilterDlg::iDistThresholdChanged);
	connect(BDRFacetFilterConfSlider, &QAbstractSlider::valueChanged, this, &bdrFacetFilterDlg::iConfThresholdChanged);
	connect(BDRFacetFilterDistSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &bdrFacetFilterDlg::dDistThresholdChanged);
	connect(BDRFacetFilterConfSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &bdrFacetFilterDlg::dConfThresholdChanged);
	
	connect(BDRFacetFilterChecktoolButton, &QAbstractButton::clicked, this, &bdrFacetFilterDlg::CheckModel);
	connect(BDRFacetFilterRestoretoolButton, &QAbstractButton::clicked, this, &bdrFacetFilterDlg::Restore);
}

bdrFacetFilterDlg::~bdrFacetFilterDlg()
{
}

void bdrFacetFilterDlg::initWith(ccGLWindow* win, ccHObject::Container _facet)
{
 	setEnabled(win != nullptr);
 	if (!win)
 		return;
 
 	if (m_win) {
 		delete m_win;
 		m_win = nullptr;
 	}
 	m_win = win;

 	m_facetObjs.clear();
	std::vector<double> m_all_distance, m_all_confidence;
 	for (auto & f : _facet) {
		ccFacet* facet = ccHObjectCaster::ToFacet(f);
		if (!facet) continue;
		m_all_confidence.push_back(facet->getConfidence());
		m_all_distance.push_back(2);
 		m_facetObjs.push_back(f);
 		m_oldstate[f] = f->isEnabled();
		m_initialstate[f] = f->isEnabled();
 	}
	if (!m_all_confidence.empty()) { sort(m_all_confidence.begin(), m_all_confidence.end()); }
	if (!m_all_distance.empty()) { sort(m_all_distance.begin(), m_all_distance.end()); }
}

void bdrFacetFilterDlg::iDistThresholdChanged(int val) {
	m_win->redraw();
}
void bdrFacetFilterDlg::iConfThresholdChanged(int val) {
	m_win->redraw();
}
void bdrFacetFilterDlg::dDistThresholdChanged(double val) {
	BDRFacetFilterDistSlider->blockSignals(true);
	BDRFacetFilterDistSlider->setValue(qFloor(val));
	BDRFacetFilterDistSlider->blockSignals(false);


	m_win->redraw();
}
void bdrFacetFilterDlg::dConfThresholdChanged(double val) {
	BDRFacetFilterConfSlider->blockSignals(true);
	BDRFacetFilterConfSlider->setValue(qFloor(val));
	BDRFacetFilterConfSlider->blockSignals(false);

	for (auto & facetObj : m_facetObjs) {
		ccFacet* facet = ccHObjectCaster::ToFacet(facetObj);
		if (facet->getConfidence() < val) {
			facet->setEnabled(false);
			facet->prepareDisplayForRefresh_recursive();
		}
 	}

	m_win->redraw();
}

void bdrFacetFilterDlg::CheckModel()
{
// 	for (ccHObject* facetObj : m_facetObjs) {
// 		facetObj->setEnabled(false);
// 		facetObj->prepareDisplayForRefresh();
// 	}

	m_win->redraw();
}

void bdrFacetFilterDlg::Restore()
{
	for (ccHObject* facetObj : m_facetObjs) {
		facetObj->setEnabled(m_initialstate[facetObj]);
		facetObj->prepareDisplayForRefresh();
	}
	m_win->redraw();
}
