#include "bdrImageEditorPanel.h"

//local
#include "mainwindow.h"
#include "bdr2.5DimEditor.h"
#include "ccDBRoot.h"
#include "ccHObject.h"
#include "ccCameraSensor.h"
#include "ccHObjectCaster.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

bdrImageEditorPanel::bdrImageEditorPanel(bdr2Point5DimEditor* img, ccDBRoot* root, QWidget* parent)
	: m_pbdrImshow(img)
	, m_root(root)
	, QDialog(parent, Qt::Tool)
	, Ui::bdrImageEditorPanelDlg()
{
	setupUi(this);
	verticalLayout->setContentsMargins(0, 0, 0, 0);
	setMinimumHeight(25);
	setMaximumHeight(150);
	m_image_display_height = 80;

	connect(ZoomFitToolButton,		&QAbstractButton::clicked, this, &bdrImageEditorPanel::ZoomFit);
	connect(toggleListToolButton, &QAbstractButton::clicked, this, &bdrImageEditorPanel::toogleImageList);
	//imageListWidget
	connect(imageListWidget->selectionModel(), &QItemSelectionModel::selectionChanged, this, &bdrImageEditorPanel::changeSelection);
	// double click list
	connect(imageListWidget, &QAbstractItemView::doubleClicked, this, &bdrImageEditorPanel::displayImage);
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
	setFixedHeight(toggleListToolButton->isChecked() ? 150 : 23);
}
void bdrImageEditorPanel::changeSelection()
{
	QList<QListWidgetItem*> list = imageListWidget->selectedItems();
	if (!list.empty()) {
		QListWidgetItem* sel_item = list.front();
		sel_item->text();
		ccHObject::Container children;
		m_root->getRootEntity()->filterChildrenByName(children, true, sel_item->text(), true);
		if (!children.empty()) {
			m_root->selectEntity(children.front());
		}
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
	//imageListWidget->scr
	//imageListWidget->setCurrentItem()
}

void bdrImageEditorPanel::setItems(std::vector<ccHObject*> items, int defaultSelectedIndex)
{
	imageListWidget->clear();
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
		imageListWidget->addItem(item);
		
	}
	if (max_width <= 0) {
		return;
	}
	imageListWidget->setIconSize(QSize(max_width, m_image_display_height));
	if (defaultSelectedIndex >= 0 && defaultSelectedIndex < items.size()) {
		imageListWidget->setItemSelected(imageListWidget->item(defaultSelectedIndex), true);
	}
}