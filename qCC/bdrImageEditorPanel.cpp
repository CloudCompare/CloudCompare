#include "bdrImageEditorPanel.h"

//local
#include "mainwindow.h"
#include "bdr2.5DimEditor.h"
#include "ccDBRoot.h"
#include "ccHObject.h"
#include "ccCameraSensor.h"
#include "ccImage.h"
#include "ccGLWindow.h"
#include "ccPointCloud.h"
#include "ccHObjectCaster.h"
#include "ccBBox.h"
#include "bdrSketcherDlg.h"
#include "vcg/space/point3.h"

#include <iostream>
#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

#include "stocker_parser.h"

bdrImageEditorPanel::bdrImageEditorPanel(bdr2Point5DimEditor* img, ccDBRoot* root, QWidget* parent)
	: m_pbdrImshow(img)
	, m_UI(new Ui::bdrImageEditorPanelDlg)
	, m_root(root)
	, QDialog(parent, Qt::Tool)
{
	m_UI->setupUi(this);
	m_UI->verticalLayout->setContentsMargins(0, 0, 0, 0);
	m_UI->verticalLayout->setSpacing(0);

	m_pSketcher = new bdrSketcher(parent);
	m_pSketcher->setFixedHeight(23);
	m_pSketcher->hide();
	connect(m_pSketcher, &ccOverlayDialog::processFinished, this, &bdrImageEditorPanel::stopEditor);
	m_UI->verticalLayoutTraceFP->addWidget(m_pSketcher);

	setMinimumHeight(23);
	setMaximumHeight(155);
	m_image_display_height = 80;

	connect(m_UI->ZoomFitToolButton,		&QAbstractButton::clicked, this, &bdrImageEditorPanel::ZoomFit);
	connect(m_UI->toggleListToolButton, &QAbstractButton::clicked, this, &bdrImageEditorPanel::toogleImageList);
	connect(m_UI->displayAllToolButton, &QAbstractButton::clicked, this, &bdrImageEditorPanel::display);
	connect(m_UI->PreviousToolButton, &QAbstractButton::clicked, this, &bdrImageEditorPanel::previous);
	connect(m_UI->NextToolButton, &QAbstractButton::clicked, this, &bdrImageEditorPanel::next);
	connect(m_UI->polyEditToolButton, &QAbstractButton::clicked, this, &bdrImageEditorPanel::startEditor);


	//imageListWidget
	m_UI->imageListWidget->setSelectionMode(QAbstractItemView::SingleSelection);
	connect(m_UI->imageListWidget->selectionModel(), &QItemSelectionModel::selectionChanged, this, &bdrImageEditorPanel::changeSelection);
	// double click list
	connect(m_UI->imageListWidget, &QAbstractItemView::doubleClicked, this, &bdrImageEditorPanel::displayImage);
	
	connect(m_root, &ccDBRoot::selectionChanged, this, &bdrImageEditorPanel::selectImage);
}

void bdrImageEditorPanel::ZoomFit()
{
	if (m_pbdrImshow) {
		m_pbdrImshow->ZoomFit();
	}
}

void bdrImageEditorPanel::toogleImageList()
{
	if (m_UI->toggleListToolButton->isChecked()) {
		if (m_pSketcher->isHidden()) {
			setFixedHeight(155);
		}
		else {
			setFixedHeight(178);
		}
	}
	else {
		if (m_pSketcher->isHidden()) {
			setFixedHeight(23);
		}
		else {
			setFixedHeight(46);
		}
	}
}

void bdrImageEditorPanel::changeSelection()
{
	QList<QListWidgetItem*> list = m_UI->imageListWidget->selectedItems();
	if (!list.empty()) {
		QListWidgetItem* sel_item = list.front();
		sel_item->text();
		ccHObject::Container children;
		m_root->getRootEntity()->filterChildrenByName(children, true, sel_item->text(), true);
		m_root->unselectAllEntities();
		if (!children.empty()) {
			m_root->selectEntity(children.front());
		}
		m_UI->imageListWidget->scrollToItem(sel_item);
	}
	else {
		//m_root->unselectAllEntities();
	}
	//m_root->selectEntity()
}

void bdrImageEditorPanel::displayImage()
{	
	//m_root->getSelectedEntities()
	emit imageDisplayed();
}

void bdrImageEditorPanel::selectImage()
{
	ccHObject::Container sels;
	m_root->getSelectedEntities(sels, CC_TYPES::CAMERA_SENSOR);
	if (sels.empty()) {
		return;
	}
	for (size_t i = 0; i < m_UI->imageListWidget->count(); i++) {
		QListWidgetItem* item = m_UI->imageListWidget->item(i);
		if (sels.back()->getName() == item->text())	{
			item->setSelected(true);
			break;
		}
	}
}

void bdrImageEditorPanel::previous()
{
	if (m_UI->imageListWidget->count() < 2) {
		return;
	}
	ccImage* cur_img = m_pbdrImshow->getImage();
	if (!cur_img) { return; }

	ccHObject* cam = cur_img->getAssociatedSensor();
	if (!cam) return;

	m_root->selectEntity(cam);
	
	QList<QListWidgetItem*> list = m_UI->imageListWidget->selectedItems();
	if (list.empty()) { return; }

	int cur_index = m_UI->imageListWidget->row(list.front());
	m_UI->imageListWidget->item((cur_index - 1) % m_UI->imageListWidget->count())->setSelected(true);
	displayImage();
}

void bdrImageEditorPanel::next()
{
	if (m_UI->imageListWidget->count() < 2) {
		return;
	}
	ccImage* cur_img = m_pbdrImshow->getImage();
	if (!cur_img) { return; }

	ccHObject* cam = cur_img->getAssociatedSensor();
	if (!cam) return;

	m_root->selectEntity(cam);

	QList<QListWidgetItem*> list = m_UI->imageListWidget->selectedItems();
	if (list.empty()) { return; }

	int cur_index = m_UI->imageListWidget->row(list.front());
	m_UI->imageListWidget->setItemSelected(m_UI->imageListWidget->item((cur_index + 1) % m_UI->imageListWidget->count()), true);
	displayImage();
}

void bdrImageEditorPanel::toogleDisplayAll()
{
	display(!m_UI->displayAllToolButton->isChecked());
}

ccHObject* bdrImageEditorPanel::getTraceBlock(QString image_name)
{
	ccHObject* cam = nullptr;
	ccHObject* dest = nullptr;
	if (image_name.isEmpty()) {
		//! get current camera
		if (m_pbdrImshow->getImage()) {
			cam = m_pbdrImshow->getImage()->getAssociatedSensor();		
		}
	}
	else {
		assert(true); // not finished
		//! get camera by name
		//MainWindow::TheInstance()->db_image()->getRootEntity()->filterChildrenByName();
	}

	if (cam) {
		for (size_t i = 0; i < cam->getChildrenNumber(); i++) {
			ccHObject* child = cam->getChild(i);
			if (child->isA(CC_TYPES::ST_BLOCKGROUP) && child->getName() == (cam->getName() + BDDB_BLOCKGROUP_SUFFIX)) {
				dest = child;
				break;
			}
		}
		if (!dest) {
			StBlockGroup* group = new StBlockGroup(cam->getName() + BDDB_BLOCKGROUP_SUFFIX);
			if (group) {
				MainWindow::TheInstance()->addToDB_Image(group, false, false, false, true);
				cam->addChild(group);
				dest = group;
			}
		}
		return dest;
	}

	return nullptr;
}

void bdrImageEditorPanel::startEditor()
{
	if (!m_pSketcher || !m_pbdrImshow->getImage()) {
		m_UI->polyEditToolButton->setChecked(false);
		return;
	}
	//m_UI->polyEditToolButton->setChecked(true);
	m_pSketcher->setTraceViewMode(true);
	m_pSketcher->linkWith(m_pbdrImshow->getGLWindow());

	//! destination
	ccHObject* dest_block = getTraceBlock(QString());
	if (!dest_block) {
		dest_block = MainWindow::TheInstance()->db_image()->getRootEntity();
	}
	if (!dest_block) { return; }
	m_pSketcher->SetDestAndGround(dest_block, 0);

	//////////////////////////////////////////////////////////////////////////

 	//! import
 	ccHObject::Container footprints;
// 	dest_block->filterChildren(footprints, true, CC_TYPES::ST_FOOTPRINT, true, nullptr);
 	m_pSketcher->importEntities3D(m_projected_2D_3D);

	//////////////////////////////////////////////////////////////////////////

	if (!m_pSketcher->start()) {
		stopEditor(false);
	}
}

void bdrImageEditorPanel::stopEditor(bool state)
{
	m_UI->polyEditToolButton->setChecked(false);
	if (m_pSketcher) {
		m_pSketcher->removeAllEntities();
	}
	//! reset gl
	m_pbdrImshow->init2DView();

	if (state) {
		//! reproject the entities back to 3d
		for (ProjectedPair prj_2d_3d : m_projected_2D_3D) {
			m_pbdrImshow->projectBack(prj_2d_3d.first, prj_2d_3d.second);
		}
	}
}

void bdrImageEditorPanel::updateCursorPos(const CCVector3d & P, bool b3d)
{
	if (isLinkToMainView()) {
		m_pbdrImshow->updateCursorPos(P, b3d, !m_UI->polyEditToolButton->isChecked());
	}
}

bool bdrImageEditorPanel::isLinkToMainView()
{
	return m_UI->linkViewToolButton->isChecked();
}

void bdrImageEditorPanel::setProjection(std::vector<ccHObject*> project_entities)
{
	m_projected_2D_3D.clear();
	for (ccHObject* ent : project_entities) {
		ccHObject* projected = m_pbdrImshow->projectToImage(ent);
		if (projected) {
			m_projected_2D_3D.push_back(std::make_pair(projected, ent));
		}
	}
	if (!m_projected_2D_3D.empty()) {
		m_pbdrImshow->getGLWindow()->redraw();
	}
}

void bdrImageEditorPanel::clearAll()
{
	m_UI->imageListWidget->clear();
	m_pbdrImshow->clearAll();
}

void bdrImageEditorPanel::setItems(std::vector<ccHObject*> items, int defaultSelectedIndex)
{
	m_UI->imageListWidget->clear();
	if (items.empty()) { return; }
	int max_width = -1;
	for (size_t i = 0; i < items.size(); i++) {
		ccCameraSensor* camObj = ccHObjectCaster::ToCameraSensor(items[i]);
		
		QListWidgetItem *item = new QListWidgetItem;
		QImage image = camObj->getImage();
		if (!image.isNull()) {
			int width = static_cast<int>((double)image.width() / (double)image.height() * (double)m_image_display_height);
			item->setSizeHint(QSize(width, m_image_display_height + 20));
			item->setIcon(QIcon(QPixmap::fromImage(image)));
			if (max_width < width) {
				max_width = width;
			}
		}
		item->setText(camObj->getName());
		m_UI->imageListWidget->addItem(item);
		
	}
	if (max_width <= 0) {
		return;
	}
	m_UI->imageListWidget->setIconSize(QSize(max_width, m_image_display_height));
	if (defaultSelectedIndex >= 0 && defaultSelectedIndex < items.size()) {
		m_UI->imageListWidget->setItemSelected(m_UI->imageListWidget->item(defaultSelectedIndex), true);
	}
}

void bdrImageEditorPanel::setItems(std::vector<ccCameraSensor*> items, int defaultSelectedIndex)
{
	m_UI->imageListWidget->clear();
	if (items.empty()) { return; }
	int max_width = -1;

	int corruptNum = -1;
	for (size_t i = 0; i < items.size(); i++) {
		ccCameraSensor* camObj = items[i];

		QListWidgetItem *item = new QListWidgetItem;
		QImage image = camObj->getImage(false);
		if (!image.isNull()) {
			int width = static_cast<int>((double)image.width() / (double)image.height() * (double)m_image_display_height);
			item->setSizeHint(QSize(width, m_image_display_height + 20));
			item->setIcon(QIcon(QPixmap::fromImage(image)));
			if (max_width < width) {
				max_width = width;
				m_UI->imageListWidget->setIconSize(QSize(max_width, m_image_display_height));
			}
			item->setText(camObj->getName());
			m_UI->imageListWidget->addItem(item);
		}
		else {
			corruptNum = i;
			break;
		}
	}

	if (corruptNum >= 0) {
		ProgStartNorm_("get image thumbs", items.size() - corruptNum)
			for (size_t i = corruptNum; i < items.size(); i++) {
				ccCameraSensor* camObj = items[i];

				QListWidgetItem *item = new QListWidgetItem;
				QImage image = camObj->getImage(true, true);
				if (!image.isNull()) {
					int width = static_cast<int>((double)image.width() / (double)image.height() * (double)m_image_display_height);
					item->setSizeHint(QSize(width, m_image_display_height + 20));
					item->setIcon(QIcon(QPixmap::fromImage(image)));
					if (max_width < width) {
						max_width = width;
						m_UI->imageListWidget->setIconSize(QSize(max_width, m_image_display_height));
					}
					item->setText(camObj->getName());
					m_UI->imageListWidget->addItem(item);
				}
				ProgStep()
			}
		ProgEnd
	}
	
	if (max_width <= 0) {
		return;
	}
	//imageListWidget->setIconSize(QSize(max_width, m_image_display_height));
	if (defaultSelectedIndex >= 0 && defaultSelectedIndex < items.size()) {
		m_UI->imageListWidget->setItemSelected(m_UI->imageListWidget->item(defaultSelectedIndex), true);
	}
}

void bdrImageEditorPanel::display(bool display_all)
{
	m_UI->displayAllToolButton->setChecked(display_all);
	//! sort by area
	ccHObject::Container children =	GetEnabledObjFromGroup(m_root->getRootEntity(), CC_TYPES::CAMERA_SENSOR, true, true);
	std::vector<ccCameraSensor*> items;
	for (ccHObject* obj : children)	{
		ccCameraSensor* camObj = ccHObjectCaster::ToCameraSensor(obj);
		assert(camObj); if (!camObj) { return; }
		if (!camObj->isBranchEnabled())	{
			continue;
		}
		if (!display_all && camObj->getDisplayOrder() < 0) {
			continue;
		}
		else {
			items.push_back(camObj);
		}
	}
	if (!display_all) {
		if (!items.empty()) {
			std::sort(items.begin(), items.end(), [](ccCameraSensor* _l, ccCameraSensor* _r) {
				return _l->getDisplayOrder() < _r->getDisplayOrder();
			});
		}
	}
	setItems(items, 0);
}

double bdrImageEditorPanel::getBoxScale()
{
	return m_UI->BoxScaleDoubleSpinBox->value();
}

bool bdrImageEditorPanel::isObjChecked()
{
	return m_objViewBox.isValid() && m_UI->CheckObjToolButton->isChecked();
}


