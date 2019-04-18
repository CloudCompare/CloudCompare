
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
#include "ccHistogramWindow.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

#include "stocker_parser.h"
#include "ccFacet.h"
#include <algorithm>

size_t g_bin_count = 20;

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
	connect(BDRFacetFilterDistHisto, &QAbstractButton::clicked, this, &bdrFacetFilterDlg::doBDRFacetFilterDistHisto);
	connect(BDRFacetFilterConfHisto, &QAbstractButton::clicked, this, &bdrFacetFilterDlg::doBDRFacetFilterConfHisto);
	
	connect(BDRFacetFilterChecktoolButton, &QAbstractButton::clicked, this, &bdrFacetFilterDlg::CheckModel);
	connect(BDRFacetFilterRestoretoolButton, &QAbstractButton::clicked, this, &bdrFacetFilterDlg::Restore);

	connect(buttonBox, SIGNAL(accepted()), this, SLOT(ConfirmAndExit()));
	connect(buttonBox, SIGNAL(rejected()), this, SLOT(RestoreAndExit()));
}

bdrFacetFilterDlg::~bdrFacetFilterDlg()
{
	Clear();
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

	m_facetObjs.clear(); m_facetObjs.shrink_to_fit();
	m_distance_histo.clear(); 
	m_confidence_histo.clear();
	m_distance_histo.resize(g_bin_count, 0);
	m_confidence_histo.resize(g_bin_count, 0);

	std::vector<double> all_conf, all_dist;

 	for (auto & f : _facet) {
		ccFacet* facet = ccHObjectCaster::ToFacet(f);
		assert(facet);
		all_dist.push_back(facet->getDistance());
		all_conf.push_back(facet->getConfidence());
 		m_facetObjs.push_back(f);
 
		m_initialstate[f] = f->isEnabled();
 	}
	if (!all_dist.empty()) { sort(all_dist.begin(), all_dist.end()); }
	if (!all_conf.empty()) { sort(all_conf.begin(), all_conf.end()); }
	min_dist = all_dist.front(); max_dist = all_dist.back();
	min_conf = all_conf.front(); max_conf = all_conf.back();
	double range_dist = max_dist - min_dist;
	double range_conf = max_conf - min_conf;
	for (auto & f : _facet) {
		ccFacet* facet = ccHObjectCaster::ToFacet(f);
		assert(facet);

		double step_dist = range_dist / static_cast<double>(g_bin_count);
		double step_conf = range_conf / static_cast<double>(g_bin_count);

		if (step_dist > 0.0) {
			size_t bin = static_cast<size_t>(floor((facet->getDistance() - min_dist) / step_dist));
			++m_distance_histo[std::min(bin, g_bin_count - 1)];
		}
		if (step_conf > 0.0) {
			size_t bin = static_cast<size_t>(floor((facet->getConfidence() - min_conf) / step_conf));
			++m_confidence_histo[std::min(bin, g_bin_count - 1)];
		}
	}
	m_val_dist = max_dist;
	m_val_conf = min_conf;

	BDRFacetFilterDistSpinBox->setMinimum(all_dist.front());
	BDRFacetFilterDistSpinBox->setMaximum(all_dist.back());
	BDRFacetFilterDistSpinBox->setValue(all_dist.back());

	BDRFacetFilterDistSlider->setMinimum(all_dist.front());
	BDRFacetFilterDistSlider->setMaximum(all_dist.back());
	BDRFacetFilterDistSlider->setValue(all_dist.back());

	BDRFacetFilterConfSpinBox->setMinimum(all_conf.front());
	BDRFacetFilterConfSpinBox->setMaximum(all_conf.back());
	BDRFacetFilterConfSpinBox->setValue(all_conf.front());

	BDRFacetFilterConfSlider->setMinimum(all_conf.front());
	BDRFacetFilterConfSlider->setMaximum(all_conf.back());
	BDRFacetFilterConfSlider->setValue(all_conf.front());
}

void bdrFacetFilterDlg::Clear()
{
	m_distance_histo.clear();
	m_confidence_histo.clear();
// 	if (m_win) {
// 		delete m_win;
// 		m_win = nullptr;
// 	}
	m_facetObjs.clear();
//	m_initialstate.clear();
}

void bdrFacetFilterDlg::ReflectThresholdChange(/*FILTER_TYPE type, double val*/)
{
	for (auto & facetObj : m_facetObjs) {
		ccFacet* facet = ccHObjectCaster::ToFacet(facetObj);
		if (facet->getDistance() > m_val_dist && facet->getConfidence() < m_val_conf) {
			facet->setEnabled(false);
		}
		else {
			facet->setEnabled(true);
		}
		facet->prepareDisplayForRefresh_recursive();
	}
/*
	switch (type)
	{
	case bdrFacetFilterDlg::FILTER_DISTANCE: {
		for (auto & facetObj : m_facetObjs) {
			ccFacet* facet = ccHObjectCaster::ToFacet(facetObj);
			facet->setEnabled(facet->getDistance() < val);
			facet->prepareDisplayForRefresh_recursive();
		}
		break;
	}
	case bdrFacetFilterDlg::FILTER_COVERAGE: {
		for (auto & facetObj : m_facetObjs) {
			ccFacet* facet = ccHObjectCaster::ToFacet(facetObj);
			facet->setEnabled(facet->getCoverage() > val);
			facet->prepareDisplayForRefresh_recursive();
		}
		break;
	}
	case bdrFacetFilterDlg::FILTER_FITTING: {
		for (auto & facetObj : m_facetObjs) {
			ccFacet* facet = ccHObjectCaster::ToFacet(facetObj);
			facet->setEnabled(facet->getFitting() > val);
			facet->prepareDisplayForRefresh_recursive();
		}
		break;
	}
	case bdrFacetFilterDlg::FILTER_CONFIDENCE: {
		for (auto & facetObj : m_facetObjs) {
			ccFacet* facet = ccHObjectCaster::ToFacet(facetObj);
			facet->setEnabled(facet->getConfidence() > val);
			facet->prepareDisplayForRefresh_recursive();
		}
		break; 
	}
	default:
		break;
	}
*/
	m_win->redraw();
}

void bdrFacetFilterDlg::iDistThresholdChanged(int val) {
	m_val_dist = val;
	BDRFacetFilterDistSpinBox->blockSignals(true);
	BDRFacetFilterDistSpinBox->setValue(val);
	BDRFacetFilterDistSpinBox->blockSignals(false);
	ReflectThresholdChange();
}
void bdrFacetFilterDlg::iConfThresholdChanged(int val) {
	m_val_conf = val;
	BDRFacetFilterConfSpinBox->blockSignals(true);
	BDRFacetFilterConfSpinBox->setValue(val);
	BDRFacetFilterConfSpinBox->blockSignals(false);
	ReflectThresholdChange();
}
void bdrFacetFilterDlg::dDistThresholdChanged(double val) {
	m_val_dist = val;
	BDRFacetFilterDistSlider->blockSignals(true);
	BDRFacetFilterDistSlider->setValue(qFloor(val));
	BDRFacetFilterDistSlider->blockSignals(false);
	ReflectThresholdChange();
}
void bdrFacetFilterDlg::dConfThresholdChanged(double val) {
	m_val_conf = val;
	BDRFacetFilterConfSlider->blockSignals(true);
	BDRFacetFilterConfSlider->setValue(qFloor(val));
	BDRFacetFilterConfSlider->blockSignals(false);
	ReflectThresholdChange();
}

void bdrFacetFilterDlg::CheckModel()
{
// 	for (ccHObject* facetObj : m_facetObjs) {
// 		facetObj->setEnabled(false);
// 		facetObj->prepareDisplayForRefresh();
// 	}
	typedef stocker::PolyMesh PMesh;
	typedef stocker::PMeshAL PMeshAL;
	PMesh pmesh;
	std::vector<size_t> used_index;
	for (size_t i = 0; i < m_facetObjs.size(); i++) {
		ccHObject* facetEnt = m_facetObjs[i];
		if (!facetEnt->isEnabled()) continue;
		ccFacet* facetObj = ccHObjectCaster::ToFacet(facetEnt);
		assert(facetObj);
		
		ccPolyline* contour = facetObj->getContour();
		if (!contour) { continue; }
		std::vector<CCVector3> contour_points = contour->getPoints();
		if (contour_points.size() < 3) continue;

		PMesh::VertexIterator vi = PMeshAL::AddVertices(pmesh, contour_points.size());
		PMesh::FaceIterator fi = PMeshAL::AddFaces(pmesh, 1);
		for (size_t _p = 0; _p < contour_points.size(); _p++) {
			vi->P() = stocker::parse_xyz(contour_points[_p]); /*vi->Q() = confidences[i];*/ ++vi;
			fi->V(_p) = &(*vi);
		}
		fi->Q() = static_cast<double>(i);/*confidences[i];*/
		used_index.push_back(i);
	}
	
	vcg::tri::Clean<PMesh>::MergeCloseVertex(pmesh, 0.000000001);

	do 
	{
		vcg::tri::UpdateFlags<PMesh>::FaceBorderFromNone(pmesh);
		vcg::tri::UpdateFlags<PMesh>::VertexBorderFromFaceBorder(pmesh);
		vcg::tri::UpdateSelection<PMesh>::FaceFromBorderFlag(pmesh);
		vcg::tri::UpdateSelection<PMesh>::VertexFromBorderFlag(pmesh);

		size_t select_face = vcg::tri::UpdateSelection<PMesh>::FaceCount(pmesh);
		size_t select_vertex = vcg::tri::UpdateSelection<PMesh>::VertexCount(pmesh);

		if (select_face == 0) { break; }
		vcg::tri::UpdateSelection<PMesh>::VertexClear(pmesh);
		vcg::tri::UpdateSelection<PMesh>::VertexFromFaceStrict(pmesh);
		int vvn = pmesh.vn;
		int ffn = pmesh.fn;
		PMesh::FaceIterator fi;
		PMesh::VertexIterator vi;
		for (fi = pmesh.face.begin(); fi != pmesh.face.end(); ++fi)
			if (!(*fi).IsD() && (*fi).IsS())
				vcg::tri::Allocator<PMesh>::DeleteFace(pmesh, *fi);
		for (vi = pmesh.vert.begin(); vi != pmesh.vert.end(); ++vi)
			if (!(*vi).IsD() && (*vi).IsS())
				vcg::tri::Allocator<PMesh>::DeleteVertex(pmesh, *vi);

		vcg::tri::UpdateTopology<PMesh>::FaceFace(pmesh);
		vcg::tri::UpdateFlags<PMesh>::Clear(pmesh);
		//	Log("Deleted %i faces, %i vertices.", ffn - m.cm.fn, vvn - m.cm.vn);
	} while (1);
		
	for (size_t i = 0; i < used_index.size(); i++) {
		ccHObject* facetEnt = m_facetObjs[i];
		ccFacet* facetObj = ccHObjectCaster::ToFacet(facetEnt);
		assert(facetObj);

		if (!pmesh.face[i].IsD()) {
			facetObj->setEnabled(false);
			facetObj->prepareDisplayForRefresh_recursive();
		}
	}

	//then remove unreferenced vertices
// 	vcg::tri::Clean<PMesh>::RemoveUnreferencedVertex(pmesh);
// 	vcg::tri::Allocator<PMesh>::CompactEveryVector(pmesh);

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

void bdrFacetFilterDlg::ConfirmAndExit()
{
	//! it is already assigned to the main window
	Clear();
}

void bdrFacetFilterDlg::RestoreAndExit()
{
	Restore();
	Clear();
}

void bdrFacetFilterDlg::doBDRFacetFilterDistHisto()
{
	ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
	hDlg->setAttribute(Qt::WA_DeleteOnClose, true);
	hDlg->setWindowTitle("Facet Histogram");

	ccHistogramWindow* histogram = hDlg->window();
	{
		histogram->setTitle("Distance histogram");
		histogram->fromBinArray(m_distance_histo, min_dist, max_dist);
		histogram->setAxisLabels("distance", "Count");
		histogram->refresh();
	}

	hDlg->show();
}

void bdrFacetFilterDlg::doBDRFacetFilterConfHisto()
{
	ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
	hDlg->setAttribute(Qt::WA_DeleteOnClose, true);
	hDlg->setWindowTitle("Facet Histogram");

	ccHistogramWindow* histogram = hDlg->window();
	{
		histogram->setTitle("Confidence histogram");
		histogram->fromBinArray(m_confidence_histo, min_conf, max_conf);
		histogram->setAxisLabels("confidence", "Count");
		histogram->refresh();
	}

	hDlg->show();
}
