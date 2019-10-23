/*
File	:		bdrProjectDlg.h
Brief	:		The header file of bdrProjectDlg

Author	:		Xinyi Liu
Date	:		2019/10/20
E-mail	:		liuxy0319@outlook.com
*/

#include "bdrProjectDlg.h"
#include "stocker_parser.h"
#include <cfloat>
#include <QFileDialog>
#include <QFileInfo>
#include <QSettings>
#include <QDirIterator>
#include <QCheckBox>
#include <QScrollBar>
#include <QCloseEvent>
#include <QMessageBox>
#include "ccPersistentSettings.h"
#include "ccFileUtils.h"
#include "ccHObject.h"
#include "ccHObjectCaster.h"
#include "FileIOFilter.h"
#include "FileIO.h"
#include "bdr2.5DimEditor.h"
#include "bdrViewerControlDlg.h"
#include "bdrPosImportDlg.h"
#include "bdrPGConnDlg.h"
#include "bdrLasTilesDlg.h"
#include "bdrCameraParaDlg.h"
#include "ccGLWindow.h"
#include "ccBBox.h"

listData * listData::New(importDataType type)
{
	switch (type) {
	case IMPORT_POINTS:
		return new pointsListData();
	case IMPORT_IMAGES:
		return new imagesListData();
	case IMPORT_MISCS:
		return new miscsListData();
	case IMPORT_MODELS:
		return new modelsListData();
	case IMPORT_POSTGIS:
		return new postGISLiistData();
	default:
		break;
	}
	return nullptr;
}

void listData::createObject(BlockDB::blkDataInfo* info)
{
}

void listData::toBlkDataInfo(BlockDB::blkDataInfo * info)
{
	if (info) {
		strcpy(info->sPath, m_path.toLocal8Bit());
		strcpy(info->sName, m_name.toLocal8Bit());
		sprintf(info->sID, "%d", m_index);
		info->nGroupID = m_groupID;
	}
}

void listData::fromBlkDataInfo(BlockDB::blkDataInfo * info)
{
	if (info) {
		m_path = QString::fromLocal8Bit(info->sPath);
		m_name = QString::fromLocal8Bit(info->sName);
		m_index = QString::fromLocal8Bit(info->sID).toInt();
		m_groupID = info->nGroupID;
	}
}

void pointsListData::createObject(BlockDB::blkDataInfo* info)
{
	listData::createObject(info);
	this->toBlkDataInfo(info);
	m_object = createObjectFromBlkDataInfo(info, true);
}

void pointsListData::toBlkDataInfo(BlockDB::blkDataInfo * info)
{
	listData::toBlkDataInfo(info);
	if (info) {
		BlockDB::blkPtCldInfo* pInfo = static_cast<BlockDB::blkPtCldInfo*>(info);
		// level
		pInfo->setStrLevel(m_level.toStdString());
	}
}

void pointsListData::fromBlkDataInfo(BlockDB::blkDataInfo * info)
{
	listData::fromBlkDataInfo(info);
	if (info && info->dataType() == BlockDB::Blk_PtCld) {
		BlockDB::blkPtCldInfo* pInfo = static_cast<BlockDB::blkPtCldInfo*>(info);
		m_level = QString::fromStdString(pInfo->getStrLevel());
	}
}

void imagesListData::createObject(BlockDB::blkDataInfo* info)
{
	listData::createObject(info);

	if (info) {
		this->toBlkDataInfo(info);
		m_object = createObjectFromBlkDataInfo(info, false);
	}
}

void imagesListData::toBlkDataInfo(BlockDB::blkDataInfo * info)
{
	listData::toBlkDataInfo(info);
	if (info) {
		auto pInfo = static_cast<BlockDB::blkImageInfo*>(info);
		if (_finite(posXs)) pInfo->posXs = posXs;
		if (_finite(posYs)) pInfo->posYs = posYs;
		if (_finite(posZs)) pInfo->posZs = posZs;
		if (_finite(posPhi)) pInfo->posPhi = posPhi;
		if (_finite(posOmega)) pInfo->posOmega = posOmega;
		if (_finite(posKappa)) pInfo->posKappa = posKappa;		
		if (_finite(gpsLat)) pInfo->gpsLat = gpsLat;
		if (_finite(gpsLon)) pInfo->gpsLon = gpsLon;
		if (_finite(gpsHeight)) pInfo->gpsHeight = gpsHeight;

		pInfo->cameraName = m_cam.toStdString();
		pInfo->setStrLevel(m_level.toStdString());
	}
}

void imagesListData::fromBlkDataInfo(BlockDB::blkDataInfo * info)
{
	listData::fromBlkDataInfo(info);
	if (info && info->dataType() == BlockDB::Blk_Image) {
		BlockDB::blkImageInfo* pInfo = static_cast<BlockDB::blkImageInfo*>(info);
		m_level = QString::fromStdString(pInfo->getStrLevel());
		this->posXs = pInfo->posXs;
		this->posYs = pInfo->posYs;
		this->posZs = pInfo->posZs;
		this->posPhi = pInfo->posPhi;
		this->posOmega = pInfo->posOmega;
		this->posKappa = pInfo->posKappa;
		this->gps_time = pInfo->gps_time;
		this->gpsLat = pInfo->gpsLat;
		this->gpsLon = pInfo->gpsLon;
		this->gpsHeight = pInfo->gpsHeight;

		this->m_cam = QString::fromStdString(pInfo->getStrCamera());
	}
}

void miscsListData::createObject(BlockDB::blkDataInfo* info)
{
	if (!info) {
		//if (this->m_meta)
		{

		}
	}
	listData::createObject(info);
	{

	}
}

void modelsListData::createObject(BlockDB::blkDataInfo * info)
{
}

void postGISLiistData::createObject(BlockDB::blkDataInfo* info)
{
	listData::createObject(info);
	

}

class bdrTableWidgetItem : public QTableWidgetItem
{
public:
	explicit bdrTableWidgetItem(int type = Type)
		: QTableWidgetItem(type)
		, m_list_data(nullptr)
	{}
	explicit bdrTableWidgetItem(const QString &text, int type = Type)
		: QTableWidgetItem(text, type)
		, m_list_data(nullptr)
	{}
	bdrTableWidgetItem(listData* data)
		: QTableWidgetItem()
		, m_list_data(data)
	{}
	listData* m_list_data;
};

bool isRowChecked(QTableWidget* tableWidget, int row) {
	return tableWidget->item(row, ListCol_ID)->checkState() == Qt::Checked;
}
listData* getRowListData(QTableWidget* tableWidget, int row) {
	bdrTableWidgetItem* bdItem = static_cast<bdrTableWidgetItem*>(tableWidget->item(row, ListCol_ID));
	return bdItem ? bdItem->m_list_data : nullptr;
}

bdrProjectDlg::bdrProjectDlg(QWidget* parent)
	: m_UI(new Ui::bdrProjectDlg)
	, m_associateProject(nullptr)
	, m_ownProject(nullptr)
	, m_postGIS(nullptr)
	, m_preview(nullptr)
{
	m_UI->setupUi(this);

	m_UI->previewToolButton->setChecked(false);
	m_UI->previewDockWidget->setVisible(false);

	setWindowFlags(windowFlags() | Qt::WindowMaximizeButtonHint);

	if (!m_preview) {
		m_preview = new bdr2Point5DEditor();
		m_preview->create2DView(m_UI->previewFrame);
		m_preview->getGLWindow()->displayOverlayEntities(false);
		connect(m_preview->getGLWindow(), &ccGLWindow::mouseMoved, this, &bdrProjectDlg::echoMouseMoved);
	}
	m_UI->previewFrame->setMouseTracking(true);
	m_UI->previewDockWidget->setMouseTracking(true);
	setMouseTracking(true);

	//! by default, display the level for points
	QStringList level_names;
	for (int i = 0; i < BlockDB::PCLEVEL_END; i++) {
		level_names << BlockDB::g_strPtCldLevelName[i];
	}
	level_names << "All";
	m_UI->productLevelComboBox->addItems(level_names);
	m_UI->productLevelComboBox->setCurrentIndex(BlockDB::PCLEVEL_TILE);
	
	//! setup the widgets
	for (int i = 0; i < IMPORT_TYPE_END; i++) {
		QTableWidget* tableWidget = getTableWidget(static_cast<importDataType>(i)); assert(tableWidget); if (!tableWidget) return;
		//! fill the header names
		tableWidget->setColumnCount(data_list_column[i]);
		QStringList header_names;
		for (int col_i = 0; col_i < ListCol_END; col_i++) {
			header_names << listHeaderName[col_i];
		}
		for (int col_i = 0; col_i < data_list_column[i]; col_i++) {
			header_names << data_list_names[i][col_i];
		}
		tableWidget->setHorizontalHeaderLabels(header_names);

		tableWidget->horizontalHeader()->sectionsMovable();
		tableWidget->horizontalHeader()->setSectionResizeMode(ListCol_ID, QHeaderView::ResizeToContents);
		
		tableWidget->setStyleSheet("selection-background-color:lightblue;");
		tableWidget->horizontalScrollBar()->setStyleSheet("QScrollBar{background:transparent; height:10px;}"
			"QScrollBar::handle{background:lightgray; border:2px solid transparent; border-radius:5px;}"
			"QScrollBar::handle:hover{background:gray;}"
			"QScrollBar::sub-line{background:transparent;}"
			"QScrollBar::add-line{background:transparent;}");
		tableWidget->verticalScrollBar()->setStyleSheet("QScrollBar{background:transparent; width: 10px;}"
			"QScrollBar::handle{background:lightgray; border:2px solid transparent; border-radius:5px;}"
			"QScrollBar::handle:hover{background:gray;}"
			"QScrollBar::sub-line{background:transparent;}"
			"QScrollBar::add-line{background:transparent;}");
		
		connect(tableWidget, &QTableWidget::itemChanged, this, &bdrProjectDlg::onItemChanged);
		connect(tableWidget, &QTableWidget::itemSelectionChanged, this, [=]() { onSelectionChanged(tableWidget); });
	}
	m_UI->pointsTableWidget->setColumnWidth(PointsCol_Path, 200);
	m_UI->imagesTableWidget->setColumnWidth(ImagesCol_Path, 200);
	
	connect(m_UI->dataFilesTabWidget,			SIGNAL(currentChanged(int)),		this, SLOT(onDataFilesChanged(int)));
	connect(m_UI->buttonBox,					&QDialogButtonBox::accepted,		this, &bdrProjectDlg::acceptAndExit);
	connect(m_UI->buttonBox,					&QDialogButtonBox::rejected,		this, &bdrProjectDlg::clear);
	connect(m_UI->applyToolButton,				&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionApply);
	
	connect(m_UI->importLocalFileToolButton,	&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionImportFile);
	connect(m_UI->importLocalFolderToolButton,	&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionImportFolder);
	connect(m_UI->exportDBToolButton,			&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionExportDB);
	
	connect(m_UI->importDeleteToolButton,		&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionDelete);
	connect(m_UI->searchToolButton,				&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionSearch);
	connect(m_UI->searchCancleToolButton,		&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionSearchCancle);
	connect(m_UI->selectAllToolButton,			&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionSelectAll);
	connect(m_UI->toggleAllToolButton,			&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionToggleAll);
	connect(m_UI->selectToolButton,				&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionSelect);
	connect(m_UI->toggleToolButton,				&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionToggle);

	connect(m_UI->previewToolButton,			&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionPreview);

	// combox
	{
		m_UI->productLevelComboBox->addItem("All");
		connect(m_UI->productLevelComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onLevelChanged(int)));
	}
	connect(m_UI->levelFilterToolButton,		&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionLevelFilter);
	connect(m_UI->levelSetupToolButton,			&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionLevelSetup);

	connect(m_UI->groupIDSetupToolButton,		&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionGroupIDSetup);
	connect(m_UI->reorganizeIndexToolButton,	&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionReorganizeIndex);

	//////////////////////////////////////////////////////////////////////////
	//! dialogs
	m_viewerCtrlDlg = new bdrViewerControlDlg(this, m_preview->getGLWindow());
	m_posImportDlg = new bdrPosImportDlg(this);
	m_pgConnDlg = new bdrPGConnDlg(this);
	m_lasTilesDlg = new bdrLasTilesDlg(this);
	m_UI->previewVerticalLayout->addWidget(m_viewerCtrlDlg);

	//////////////////////////////////////////////////////////////////////////
	// points tab
	connect(m_UI->pointsTileToolButton,			&QAbstractButton::clicked, this, &bdrProjectDlg::doActionLasTiles);
	connect(m_UI->pointsBoundaryToolButton,		&QAbstractButton::clicked, this, &bdrProjectDlg::doActionPointsBoundary);
	connect(m_UI->searchAssLevelToolButton,		&QAbstractButton::clicked, this, &bdrProjectDlg::doActionSearchAssLevel);
	// iamges tab
	connect(m_UI->posFileToolButton,			&QAbstractButton::clicked, this, &bdrProjectDlg::doActionPosFile);
	connect(m_UI->gpsInfoToolButton,			&QAbstractButton::clicked, this, &bdrProjectDlg::doActionGpsInfo);
	connect(m_UI->imageCoorCvtToolButton,		&QAbstractButton::clicked, this, &bdrProjectDlg::doActionImgCoorCvt);
	connect(m_UI->undistortToolButton,			&QAbstractButton::clicked, this, &bdrProjectDlg::doActionImageUndistort);
	m_UI->cameraComboBox->addItem(QString::fromLocal8Bit("添加相机"));
	connect(m_UI->cameraComboBox,				SIGNAL(activated(int)),	   this, SLOT(clicked_CameraComboBox(int)));
	connect(m_UI->cameraOptionsToolButton,		&QAbstractButton::clicked, this, &bdrProjectDlg::doActionCameraOptions);
	connect(m_UI->cameraSetupToolButton,		&QAbstractButton::clicked, this, &bdrProjectDlg::doActionCameraSetup);
	// miscs tab
	connect(m_UI->addGCPToolButton,				&QAbstractButton::clicked, this, &bdrProjectDlg::doActionAddGcp);
	connect(m_UI->productCoorToolButton,		&QAbstractButton::clicked, this, &bdrProjectDlg::doActionProductCoor);
	connect(m_UI->addNewMiscToolButton,			&QAbstractButton::clicked, this, &bdrProjectDlg::doActionAddNewMisc);
	// models tab
	connect(m_UI->displayModelBdryToolButton,	&QAbstractButton::clicked, this, &bdrProjectDlg::doActionDisplayModelBdry);
	connect(m_UI->viewModelToolButton,			&QAbstractButton::clicked, this, &bdrProjectDlg::doActionViewModel);
	connect(m_UI->exportModelToolButton,		&QAbstractButton::clicked, this, &bdrProjectDlg::doActionExportModel);
	// postgis tab
	connect(m_UI->importDBToolButton,			&QAbstractButton::clicked, this, &bdrProjectDlg::doActionImportDatabase);
	connect(m_UI->settingDBToolButton,			&QAbstractButton::clicked, this, &bdrProjectDlg::doActionSettingDatabase);
	connect(m_UI->importFromDBAllToolButton,	&QAbstractButton::clicked, this, &bdrProjectDlg::doActionImportDBAll);
	connect(m_UI->importFromDBRegionToolButton, &QAbstractButton::clicked, this, &bdrProjectDlg::doActionImportDBRegion);
	connect(m_UI->queryDBToolButton,			&QAbstractButton::clicked, this, &bdrProjectDlg::doActionQueryDB);
}

bdrProjectDlg::~bdrProjectDlg()
{
	delete m_UI;
}

void bdrProjectDlg::clear()
{
	ccGLWindow* glWin = m_preview->getGLWindow();
	if (glWin && m_ownProject) glWin->removeFromOwnDB(m_ownProject);
	resetObjects();
	linkWithProject(nullptr);
	
	for (size_t i = 0; i < IMPORT_TYPE_END; i++) {
		listData::Container& list_data = getListDatas((importDataType)i);
		for (listData::Container::iterator it = list_data.begin(); it != list_data.end(); it++)	{
			delete *it;
		}
		list_data.clear();
		QTableWidget* tableWidget = getTableWidget((importDataType)i);
		while (tableWidget->rowCount() > 0) {
			tableWidget->removeRow(0);
		}
	}

	while (m_UI->cameraComboBox->count() > 1) {
		m_UI->cameraComboBox->removeItem(0);
	}
}

void bdrProjectDlg::linkWithProject(DataBaseHObject * proj)
{
	if (m_associateProject != proj)	{
		m_associateProject = proj;
		if (m_associateProject)	{
			//! CLEAR
			resetObjects();
			m_ownProject = DataBaseHObject::Create(m_associateProject->getPath());
			//m_ownProject->setPath(m_associateProject->getPath());
			if (m_preview) {
				m_preview->getGLWindow()->addToOwnDB(m_ownProject);
			}
			HObjectToList(m_associateProject);
		}
	}
}

QString bdrProjectDlg::getProjectPath()
{
	return m_UI->projDirLineEdit->text();
}
void bdrProjectDlg::setProjectPath(QString path, bool enable)
{
	m_UI->projDirLineEdit->setText(path);
	m_UI->projDirLineEdit->setEnabled(enable);
	m_UI->projDirToolButton->setEnabled(enable);
}
int bdrProjectDlg::getProjectID()
{
	return m_UI->projIDSpinBox->value();
}
int bdrProjectDlg::getProjetcGroupID()
{
	return m_UI->groupIDSpinBox->value();
}

importDataType bdrProjectDlg::getCurrentTab() {
	return importDataType(m_UI->dataFilesTabWidget->currentIndex());
}
importDataType bdrProjectDlg::getCurrentTab(QTableWidget * widget)
{
	if (widget == m_UI->pointsTableWidget) {
		return IMPORT_POINTS;
	}
	else if (widget == m_UI->imagesTableWidget) {
		return IMPORT_IMAGES;
	}
	else if (widget == m_UI->miscsTableWidget) {
		return IMPORT_MISCS;
	}
	else if (widget == m_UI->modelsTableWidget) {
		return IMPORT_MODELS;
	}
	else if (widget == m_UI->postgisTableWidget) {
		return IMPORT_POSTGIS;
	}
	return importDataType();
}
QTableWidget * bdrProjectDlg::getTableWidget(importDataType type)
{
	switch (type)
	{
	case IMPORT_POINTS:
		return m_UI->pointsTableWidget;
	case IMPORT_IMAGES:
		return m_UI->imagesTableWidget;
	case IMPORT_MISCS:
		return m_UI->miscsTableWidget;
	case IMPORT_MODELS:
		return m_UI->modelsTableWidget;
	case IMPORT_POSTGIS:
		return m_UI->postgisTableWidget;
	default:
		throw std::runtime_error("internal error");
		assert(false);
		break;
	}
	return nullptr;
}
std::vector<listData*>& bdrProjectDlg::getListDatas(importDataType type)
{
	switch (type)
	{
	case IMPORT_POINTS:
		return m_points_data;
	case IMPORT_IMAGES:
		return m_images_data;
	case IMPORT_MISCS:
		return m_miscs_data;
	case IMPORT_MODELS:
		return m_models_data;
	case IMPORT_POSTGIS:
		return m_postgis_data;
	default:
		throw std::runtime_error("internal error");
		assert(false);
		break;
	}
	return std::vector<listData*>();
}

// void bdrProjectDlg::mouseMoveEvent(QMouseEvent * event)
// {
// 	if (m_preview&&m_preview->getGLWindow()) {
// // 		bool m = m_preview->getGLWindow()->underMouse();
// // 		if (m) {
// // 			m_preview->getGLWindow()->mouseMoveEvent(event);
// // 		}
// 	}
// }

bool bdrProjectDlg::event(QEvent * evt)
{
	switch (evt->type())
	{
	case QEvent::Close:
		clear();
		evt->accept();
		break;
	default:
		break;
	}


 	if (isPreviewEnable()) {
 		ccGLWindow* gl = m_preview->getGLWindow();
 		switch (evt->type())
 		{
 		case QEvent::MouseMove:
 		{
 			QMouseEvent* event = static_cast<QMouseEvent*>(evt);
 			gl->mouseMoveEvent(event);
 			evt->accept();
 		}
 			break;
 		case QEvent::MouseButtonPress:
 		{
 			QMouseEvent* event = static_cast<QMouseEvent*>(evt);
 			gl->mousePressEvent(event); 
 			evt->accept();
 		}
 			break;
 		case QEvent::MouseButtonRelease:
 		{
 			QMouseEvent* event = static_cast<QMouseEvent*>(evt);
 			gl->mouseReleaseEvent(event);
 			evt->accept();
 		}
 			break;
 		case QEvent::Wheel:
 		{
 			QWheelEvent* event = static_cast<QWheelEvent*>(evt);
 			gl->wheelEvent(event);
 			evt->accept();
 		}
 			break;
 		default:
 			break;
 		}
 	}
	
	return QDialog::event(evt);
}

void bdrProjectDlg::echoMouseMoved(int x, int y, Qt::MouseButtons buttons)
{
	// set a ground map to echo mouse move
// 	CCVector3d p;
// 	m_preview->getGLWindow()->getClick3DPos(x, y, .0f, p);
	//m_UI->statusPreviewLabel->setText(QStringLiteral("%1,%2").arg(x).arg(y));

}

void bdrProjectDlg::onLevelChanged(int)
{
	// change the combox name
}

void bdrProjectDlg::onDataFilesChanged(int index)
{
	//! clear
	while (m_UI->productLevelComboBox->count() > 0) {
		m_UI->productLevelComboBox->removeItem(0);
	}

	QStringList level_names;
	switch (index)
	{
	case 0:
		for (int i = 0; i < BlockDB::PCLEVEL_END; i++) {
			level_names << BlockDB::g_strPtCldLevelName[i];
		}
		level_names << "All";
		m_UI->productLevelComboBox->addItems(level_names);
		m_UI->productLevelComboBox->setCurrentIndex(BlockDB::PCLEVEL_TILE);
		break;
	case 1:
		for (int i = 0; i < BlockDB::IMGLEVEL_END; i++) {
			level_names << BlockDB::g_strImageLevelName[i];
		}
		level_names << "All";
		m_UI->productLevelComboBox->addItems(level_names);
		m_UI->productLevelComboBox->setCurrentIndex(BlockDB::IMGLEVEL_STRIP);
		break;
	case 2:
		for (int i = 0; i < BlockDB::MISCAPP_END; i++) {
			level_names << BlockDB::g_strMiscAppName[i];
		}
		level_names << "All";
		m_UI->productLevelComboBox->addItems(level_names);
		m_UI->productLevelComboBox->setCurrentIndex(BlockDB::MISCAPP_CAM);
		break;
	case 3:
	case 4:
		level_names << "All";
		m_UI->productLevelComboBox->addItems(level_names);
		m_UI->productLevelComboBox->setCurrentIndex(0);
		break;
	default:
		break;
	}
}

void bdrProjectDlg::onItemChanged(QTableWidgetItem * item)
{
	QTableWidget* tableWidget = item->tableWidget();
	importDataType data_type = getCurrentTab(tableWidget);
	
	bdrTableWidgetItem* bdItem = static_cast<bdrTableWidgetItem*>(tableWidget->item(item->row(), ListCol_ID));
	listData* data = bdItem->m_list_data;

	if (!data || !data->isDisplayed()) return;

	int iCol = item->column();
	QString iText = item->text();
	
	if (iCol == ListCol_ID) {
		data->m_index = iText.toInt();
		return;
	}
	else if (iCol == ListCol_Name) {
		data->m_name = iText;
		return;
	}

	switch (data_type)
	{
	case IMPORT_POINTS: {
		pointsListData* pd = static_cast<pointsListData*>(data);
		switch (iCol)
		{
		case PointsCol_PtsCnt:
			pd->m_pointCnt = item->text().toUInt();
			break;
		case PointsCol_Level:
			pd->m_level = iText;
			break;
		case PointsCol_AssLv:
			pd->m_assLevels = iText.simplified().split(QChar(';'), QString::SkipEmptyParts);
			break;
		case PointsCol_GroupID:
			pd->m_groupID = iText.toInt();
			break;
		case PointsCol_Path:
			pd->m_path = iText;
			break;
		default:
			break;
		}
		break;
	}
	case IMPORT_IMAGES: {
		imagesListData* pd = static_cast<imagesListData*>(data);
		switch (iCol)
		{
		case ImagesCol_CamName:
			pd->m_cam = iText;
			break;
		case ImagesCol_PosXs:
			pd->posXs = iText.toDouble();
			break;
		case ImagesCol_PosYs:
			pd->posYs = iText.toDouble();
			break;
		case ImagesCol_PosZs:
			pd->posZs = iText.toDouble();
			break;
		case ImagesCol_PosPhi:
			pd->posPhi = iText.toDouble();
			break;
		case ImagesCol_PosOmega:
			pd->posOmega = iText.toDouble();
			break;
		case ImagesCol_PosKappa:
			pd->posKappa = iText.toDouble();
			break;
		case ImagesCol_GpsLat:
			pd->gpsLat = iText.toDouble();
			break;
		case ImagesCol_GpsLot:
			pd->gpsLon = iText.toDouble();
			break;
		case ImagesCol_GpsHgt:
			pd->gpsHeight = iText.toDouble();
			break;
		case ImagesCol_GpsTime:
			pd->gps_time = iText.toDouble();
			break;
		case ImagesCol_GroupID:
			pd->m_groupID = iText.toInt();
			break;
		case ImagesCol_Level:
			pd->m_level = iText;
			break;
		case ImagesCol_Path:
			pd->m_path = iText;
			break;
		default:
			break;
		}
	}
		break;
	case IMPORT_MISCS:
		break;
	case IMPORT_MODELS:
		break;
	case IMPORT_POSTGIS:
		break;
	case IMPORT_TYPE_END:
	default:
		break;
	}
}

void bdrProjectDlg::onSelectionChanged(QTableWidget * table)
{
	if (isPreviewEnable()) {
		//! set the HObject selected
		
	}
}

void bdrProjectDlg::diaplayMessage(QString message, PRJ_ERROR_CODE error_code)
{
	switch (error_code)
	{
	case bdrProjectDlg::PRJMSG_STATUS:
		m_UI->statusPrjLabel->setText("Info: " + message);
		break;
	case bdrProjectDlg::PRJMSG_WARNING:
		m_UI->statusPrjLabel->setText("Warning: " + message);
		break;
	case bdrProjectDlg::PRJMSG_ERROR:
		m_UI->statusPrjLabel->setText("Error: " + message);
		break;
	case bdrProjectDlg::PRJMSG_CRITICAL:
		QMessageBox::critical(this, "Error!", message);
		break;
	default:
		break;
	}
}
void bdrProjectDlg::diaplayMessage(const char* message, PRJ_ERROR_CODE error_code)
{
	diaplayMessage(QString::fromLocal8Bit(message), error_code);
}

bool bdrProjectDlg::insertItemToTable(listData * data)
{
	importDataType data_type = data->getDataType();
	assert(data);
	QTableWidget* tableWidget = getTableWidget(data_type);
	if (!tableWidget) {
		return false;
	}
	
	const int table_index = tableWidget->rowCount();

	//! insert a new row
	tableWidget->insertRow(table_index);
	tableWidget->setVerticalHeaderItem(table_index, new QTableWidgetItem(QString::number(table_index + 1)));

	tableWidget->setItem(table_index, ListCol_ID, new bdrTableWidgetItem(data));
	for (size_t i = ListCol_ID + 1; i < data_list_column[data_type]; i++) {
		tableWidget->setItem(table_index, i, new QTableWidgetItem);
	}
	//! fill index, and name
	tableWidget->item(table_index, ListCol_ID)->setCheckState(Qt::Checked);
	tableWidget->item(table_index, ListCol_ID)->setText(QString::number(data->m_index));	
	tableWidget->item(table_index, ListCol_ID)->setFlags(tableWidget->item(table_index, ListCol_ID)->flags() & ~Qt::ItemIsEditable);
	tableWidget->item(table_index, ListCol_Name)->setText(data->m_name);

	switch (data_type)
	{
	case IMPORT_POINTS:
	{
		tableWidget->item(table_index, PointsCol_Path)->setText(data->m_path);
		tableWidget->item(table_index, PointsCol_GroupID)->setText(QString::number(data->m_groupID));
		tableWidget->item(table_index, PointsCol_Level)->setText(data->m_level);

		pointsListData* pData = static_cast<pointsListData*>(data);
		tableWidget->item(table_index, PointsCol_PtsCnt)->setText(QString::number(pData->m_pointCnt));

		QString al;	for (auto l : pData->m_assLevels.toSet()) { al += (l + ";"); }
		tableWidget->item(table_index, PointsCol_AssLv)->setText(al);

		break;
	}
	case IMPORT_IMAGES:
	{
		tableWidget->item(table_index, ImagesCol_Path)->setText(data->m_path);
		tableWidget->item(table_index, ImagesCol_GroupID)->setText(QString::number(data->m_groupID));
		tableWidget->item(table_index, ImagesCol_Level)->setText(data->m_level);

		imagesListData* pData = static_cast<imagesListData*>(data);
		tableWidget->item(table_index, ImagesCol_CamName)->setText(pData->m_cam);
		if (_finite(pData->posXs)) tableWidget->item(table_index, ImagesCol_PosXs)->setText(QString::number(pData->posXs, 'f', 6));
		if (_finite(pData->posYs)) tableWidget->item(table_index, ImagesCol_PosYs)->setText(QString::number(pData->posYs, 'f', 6));
		if (_finite(pData->posZs)) tableWidget->item(table_index, ImagesCol_PosZs)->setText(QString::number(pData->posZs, 'f', 6));
		if (_finite(pData->posPhi)) tableWidget->item(table_index, ImagesCol_PosPhi)->setText(QString::number(pData->posPhi, 'f', 6));
		if (_finite(pData->posOmega)) tableWidget->item(table_index, ImagesCol_PosOmega)->setText(QString::number(pData->posOmega, 'f', 6));
		if (_finite(pData->posKappa)) tableWidget->item(table_index, ImagesCol_PosKappa)->setText(QString::number(pData->posKappa, 'f', 6));
		if (_finite(pData->gpsLat)) tableWidget->item(table_index, ImagesCol_GpsLat)->setText(QString::number(pData->gpsLat, 'f', 6));
		if (_finite(pData->gpsLon)) tableWidget->item(table_index, ImagesCol_GpsLot)->setText(QString::number(pData->gpsLon, 'f', 6));
		if (_finite(pData->gpsHeight)) tableWidget->item(table_index, ImagesCol_GpsHgt)->setText(QString::number(pData->gpsHeight, 'f', 6));
		if (_finite(pData->gps_time)) tableWidget->item(table_index, ImagesCol_GpsHgt)->setText(QString::number(pData->gps_time, 'f', 6));

		break;
	}
	case IMPORT_MISCS:
	{
		tableWidget->item(table_index, MiscsCol_Path)->setText(data->m_path);
		tableWidget->item(table_index, MiscsCol_GroupID)->setText(QString::number(data->m_groupID));
		tableWidget->item(table_index, MiscsCol_Level)->setText(data->m_level);

		miscsListData* pData = static_cast<miscsListData*>(data);
		tableWidget->item(table_index, MiscsCol_MetaKey)->setText(pData->m_meta_key);
		tableWidget->item(table_index, MiscsCol_MetaValue)->setText(pData->m_meta_value);
		break;
	}
	case IMPORT_MODELS:
	{
		tableWidget->item(table_index, ModelsCol_Path)->setText(data->m_path);
		tableWidget->item(table_index, ModelsCol_GroupID)->setText(QString::number(data->m_groupID));
		tableWidget->item(table_index, ModelsCol_Level)->setText(data->m_level);

		modelsListData* pData = static_cast<modelsListData*>(data);
		tableWidget->item(table_index, ModelsCol_Origin)->setText(pData->m_origin);
		tableWidget->item(table_index, ModelsCol_MetaKey)->setText(pData->m_meta_key);
		tableWidget->item(table_index, ModelsCol_MetaValue)->setText(pData->m_meta_value);
		break;
	}
	case IMPORT_POSTGIS:
	{
		tableWidget->item(table_index, PostgisCol_Path)->setText(data->m_path);
		tableWidget->item(table_index, PostgisCol_Level)->setText(data->m_level);

		postGISLiistData* pData = static_cast<postGISLiistData*>(data);
		tableWidget->item(table_index, PostgisCol_MetaKey)->setText(pData->m_meta_key);
		tableWidget->item(table_index, PostgisCol_MetaValue)->setText(pData->m_meta_value);
		break;
	}
	default:
		assert(false);
		break;
	}
	data->setDisplayed(true);
	tableWidget->scrollToBottom();

	return true;
}

listData* bdrProjectDlg::addFilePathToTable(QString path, importDataType data_type)
{
	//! load data

	listData::Container& list_datas = getListDatas(data_type);
	listData* data = listData::New(data_type);
	if (!data) { return false; }

	//! fill the data
	data->m_name = QFileInfo(path).completeBaseName();
	data->m_path = path;
	data->m_index = list_datas.empty() ? 0 : (list_datas.back()->m_index + 1);
	data->m_groupID = getProjetcGroupID();

	QString level_cb = m_UI->productLevelComboBox->currentText();
	if (level_cb != "uno" && level_cb != "All") {
		data->m_level = level_cb;
	}

	if (!data) { return false; }
	try {
		if (!insertItemToTable(data)) {
			if (data) {
				delete data;
				data = nullptr;
			}
			return false;
		}
	}
	catch (const std::exception&) {
		if (data) {
			delete data;
			data = nullptr;
		}
		return false;
	}
	list_datas.push_back(data);
	return data;
}

bool bdrProjectDlg::ListToHObject(bool preview_control)
{
	DataBaseHObject* tempProj = nullptr;
	auto failedExitprj = [&]() {
		if (tempProj) {
			delete tempProj;
			tempProj = nullptr;
		}
	};
	auto failedExit = [](void* itm) {
		if (itm) {
			delete itm;
			itm = nullptr;
		}
	};
	//TODO: set a new project, if success, transfer children
	m_ownProject->clear();
	
	//! points
	bool update = !preview_control || (preview_control && 1);
	if (update) {
		for (listData* data : m_points_data) if (data) {
			pointsListData* pData = static_cast<pointsListData*>(data); if (!pData) continue;
			
			BlockDB::blkPtCldInfo info;
			pData->setObject(nullptr);
			pData->createObject(&info); // load file
			if (!pData->getObject()) { failedExitprj(); continue; }
						
			m_ownProject->addData(pData->getObject(), &info, false);
		}
	}

	//! images
	update = !preview_control || (preview_control && 1);
	if (update) {
		//! firstly, save the camera data
		std::vector<BlockDB::blkCameraInfo> camData = getCameraData();
		for (size_t i = 0; i < camData.size(); i++) {
			StHObject* camObj = createObjectFromBlkDataInfo(&camData[i], false);
			m_ownProject->addData(camObj, &camData[i], false);
		}
		for (listData* data : m_images_data) if (data) {
			imagesListData* pData = static_cast<imagesListData*>(data); if (!pData) continue;

			BlockDB::blkImageInfo info;
			pData->setObject(nullptr); pData->createObject(&info);
			if (!pData->getObject()) { failedExitprj(); continue; }
						
			m_ownProject->addData(pData->getObject(), &info, false);
		}
	}

	//! miscs
	update = !preview_control || (preview_control && 1);
	if (preview_control && 1) {
		for (listData* data : m_miscs_data) if (data) {
			miscsListData* pData = static_cast<miscsListData*>(data); if (!pData) continue;

			BlockDB::blkMiscsInfo info;
			pData->setObject(nullptr); pData->createObject(&info);
			if (!pData->getObject()) { failedExitprj(); continue; }

			m_ownProject->addData(pData->getObject(), &info, false);
		}
	}

	//! postgis
	update = !preview_control || (preview_control && 1);
	if (0) {
		m_postGIS;
	}

	return true;
}

bool bdrProjectDlg::HObjectToList(StHObject* projObj)
{
	resetLists();
	DataBaseHObject* baseObj = ToDatabaseProject(projObj); if (!baseObj) { return false; }
	
	for (auto info : baseObj->m_blkData->getPtClds()) {
		listData::Container& list_datas = getListDatas(IMPORT_POINTS);
		listData* data = listData::New(IMPORT_POINTS);
		data->fromBlkDataInfo(&info);

		if (!insertItemToTable(data)) {
			if (data) {
				delete data;
				data = nullptr;
			}
			continue;
		}
		list_datas.push_back(data);
	}
	for (auto info : baseObj->m_blkData->getImages()) {
		listData::Container& list_datas = getListDatas(IMPORT_IMAGES);
		listData* data = listData::New(IMPORT_IMAGES);
		data->fromBlkDataInfo(&info);

		if (!insertItemToTable(data)) {
			if (data) {
				delete data;
				data = nullptr;
			}
			continue;
		}
		list_datas.push_back(data);
	}
	for (auto info : baseObj->m_blkData->getCameras()) {
		addCameraData(info);
	}
	
	return true;
}

void bdrProjectDlg::resetLists()
{
	m_points_data.clear();
	m_images_data.clear();
	m_miscs_data.clear();
}

void bdrProjectDlg::resetObjects()
{
	if (m_ownProject) {
		delete m_ownProject;
		m_ownProject = nullptr;
	}
}

bool bdrProjectDlg::isPreviewEnable()
{
	return m_preview && m_preview->getGLWindow() && m_UI->previewDockWidget->isVisible();
}


void bdrProjectDlg::doActionOpenProject()
{
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString path = QFileDialog::getExistingDirectory(this,
		tr("project directory"),
		QFileInfo(currentPath).absoluteFilePath(),
		QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

	if (path.isEmpty()) {
		return;
	}
	resetObjects();
	m_ownProject = new DataBaseHObject(QFileInfo(path).completeBaseName());
	m_ownProject->setPath(path);
	if (!m_ownProject->load()) {
		delete m_ownProject;
		m_ownProject = nullptr;
		return;
	}
	if (HObjectToList(m_ownProject)) {
		m_UI->projDirLineEdit->setText(path);
		if (isPreviewEnable()) {
			m_preview->getGLWindow()->redraw();
		}
	}
}

void bdrProjectDlg::doActionImportFile()
{
	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
	QString currentOpenDlgFilter = settings.value(ccPS::SelectedInputFilter(), "*.*").toString();

	// Add all available file I/O filters (with import capabilities)
	const QStringList filterStrings = FileIOFilter::ImportFilterList();
	const QString &allFilter = filterStrings.at(0);

	if (!filterStrings.contains(currentOpenDlgFilter)) {
		currentOpenDlgFilter = allFilter;
	}
	//file choosing dialog
	QStringList selectedFiles = QFileDialog::getOpenFileNames(this,
		tr("Open file(s)"),
		currentPath,
		filterStrings.join(";;"),
		&currentOpenDlgFilter);
	if (selectedFiles.isEmpty())
		return;

	//save last loading parameters
	currentPath = QFileInfo(selectedFiles[0]).absoluteFilePath();
	settings.setValue(ccPS::CurrentPath(), currentPath);
	settings.setValue(ccPS::SelectedInputFilter(), currentOpenDlgFilter);
	settings.endGroup();

	for (auto file_path : selectedFiles) {
		if (!addFilePathToTable(file_path, getCurrentTab())) {

		}
	}
	getTableWidget(getCurrentTab())->resizeColumnToContents(ListCol_Name);
}

void bdrProjectDlg::doActionImportFolder()
{
	//! get directory
			//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString dirname = QFileDialog::getExistingDirectory(this,
		tr("Open Directory"),
		QFileInfo(currentPath).absolutePath(),
		QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

	if (!QFileInfo(dirname).exists()) {
		return;
	}

	//save last loading parameters
	currentPath = QFileInfo(dirname).absoluteFilePath();
	settings.setValue(ccPS::CurrentPath(), currentPath);
	settings.endGroup();

	QStringList files;
	QStringList nameFilters;
	// let user to set the filter
	switch (getCurrentTab())
	{
	case IMPORT_POINTS:
		nameFilters << "*.las" << "*.laz" << "*.ply" << "*.obj" << "*.pcd";
		break;
	case IMPORT_IMAGES:
		nameFilters << "*.jpg" << "*.tiff" << "*.tif" << "*.png" << "*.bmp";
		break;
	default:
		nameFilters << "*.*";
		break;
	}

	QDirIterator dir_iter(dirname, nameFilters, QDir::Files | QDir::NoSymLinks | QDir::Readable, QDirIterator::Subdirectories);
	while (dir_iter.hasNext()) {
		files.append(dir_iter.next());
	}

	for (auto file_path : files) {
		if (!addFilePathToTable(file_path, getCurrentTab())) {

		}
	}
	getTableWidget(getCurrentTab())->resizeColumnToContents(ListCol_Name);
}

void bdrProjectDlg::doActionExportDB()
{
}

void bdrProjectDlg::doActionDelete()
{
	QTableWidget* tableWidget = getTableWidget(getCurrentTab());
	int i = 0;
	int count = tableWidget->rowCount();
	do
	{
		bdrTableWidgetItem* bdItem = static_cast<bdrTableWidgetItem*>(tableWidget->item(i, ListCol_ID));
		if (bdItem->checkState() == Qt::Checked) {
			listData::Container& list_datas = getListDatas(getCurrentTab());
			for (listData::Container::iterator it = list_datas.begin(); it != list_datas.end(); it++) {
				if (*it == bdItem->m_list_data) {
					delete *it;
					list_datas.erase(it--);
					bdItem->m_list_data = nullptr;

					break;
				}
			}
			tableWidget->removeRow(i--);
			count--;
		}

		i++;
	} while (i < count);
}

void bdrProjectDlg::doActionSearch()
{
	QTableWidget* tableWidget = getTableWidget(getCurrentTab());
	listData::Container& list_datas = getListDatas(getCurrentTab());

	QString search_field = m_UI->searchFieldLineEdit->text();
	for (int i = tableWidget->rowCount() - 1; i >= 0; i--) {
		bdrTableWidgetItem* item = static_cast<bdrTableWidgetItem*>(tableWidget->item(i, ListCol_ID));
		
		if (item && !tableWidget->item(i, ListCol_Name)->text().contains(search_field)) {
			if (item->m_list_data) {
				item->m_list_data->setDisplayed(false);
			}
			tableWidget->removeRow(i);
		}
	}
}

void bdrProjectDlg::doActionSearchCancle()
{
	QTableWidget* tableWidget = getTableWidget(getCurrentTab());
	listData::Container& list_datas = getListDatas(getCurrentTab());
	for (auto & data : list_datas) {
		data->setDisplayed(false);
	}
	while (tableWidget->rowCount() > 0) {
		tableWidget->removeRow(0);
	}
	for (auto & data : list_datas) {
		insertItemToTable(data);
	}
}

void bdrProjectDlg::doActionLevelFilter()
{
}

void bdrProjectDlg::doActionLevelSetup()
{
}

void bdrProjectDlg::doActionGroupIDSetup()
{
}

void bdrProjectDlg::doActionReorganizeIndex()
{
	// TODO: temporarily for display
	importDataType data_type = getCurrentTab();
	listData::Container& list_datas = getListDatas(data_type);
	listData* data = listData::New(data_type);
	if (!data) { return; }

	//! fill the data
	data->m_index = list_datas.empty() ? 0 : (list_datas.back()->m_index + 1);
	data->m_groupID = getProjetcGroupID();

	QString level_cb = m_UI->productLevelComboBox->currentText();
	if (level_cb != "uno" && level_cb != "All") {
		data->m_level = level_cb;
	}
	if (insertItemToTable(data)) {
		list_datas.push_back(data);
	}
	else {
		delete data;
		data = nullptr;
	}
}

void bdrProjectDlg::doActionLasTiles()
{
	if (m_lasTilesDlg->exec()) {
		//! add tiled points to the list
	}
}

void bdrProjectDlg::doActionPointsBoundary()
{
}

void bdrProjectDlg::doActionSearchAssLevel()
{
}

void bdrProjectDlg::doActionPosFile()
{
	if (m_posImportDlg->exec()) {
		//! search pos to list
		std::vector<BlockDB::blkImageInfo> ImagePosInfo = m_posImportDlg->getImagePosInfo();
		if (!ImagePosInfo.empty()) {
			std::vector<bool> pos_checked(ImagePosInfo.size(), false);
			QTableWidget* tableWidget = getTableWidget(IMPORT_IMAGES);
			for (size_t i = 0; i < tableWidget->rowCount(); i++) {
				if (isRowChecked(tableWidget, i)) {
					for (size_t j = 0; j < ImagePosInfo.size(); ++j) {
						if (pos_checked[j]) continue; BlockDB::blkImageInfo info = ImagePosInfo[j];
						if (tableWidget->item(i, ListCol_Name)->text().contains(QFileInfo(info.sName).baseName())) {
							if (_finite(info.posXs)) tableWidget->item(i, ImagesCol_PosXs)->setText(QString::number(info.posXs, 'f', 6));
							if (_finite(info.posYs)) tableWidget->item(i, ImagesCol_PosYs)->setText(QString::number(info.posYs, 'f', 6));
							if (_finite(info.posZs)) tableWidget->item(i, ImagesCol_PosZs)->setText(QString::number(info.posZs, 'f', 6));
							if (_finite(info.posPhi)) tableWidget->item(i, ImagesCol_PosPhi)->setText(QString::number(info.posPhi, 'f', 6));
							if (_finite(info.posOmega))	tableWidget->item(i, ImagesCol_PosOmega)->setText(QString::number(info.posOmega, 'f', 6));
							if (_finite(info.posKappa))	tableWidget->item(i, ImagesCol_PosKappa)->setText(QString::number(info.posKappa, 'f', 6));
							if (_finite(info.gps_time))	tableWidget->item(i, ImagesCol_GpsTime)->setText(QString::number(info.gps_time, 'f', 6));

							break;
						}
					}
				}
			}
		}
	}
}

void bdrProjectDlg::doActionGpsInfo()
{
	ccProgressDialog pdlg;
	//for (size_t i = 0; i < ; i++)
	{
		//getImageGPSInfo()
	}
}

void bdrProjectDlg::doActionImgCoorCvt()
{
}

void bdrProjectDlg::doActionImageUndistort()
{
}

void bdrProjectDlg::addCameraData(const BlockDB::blkCameraInfo & info)
{
	int index = m_UI->cameraComboBox->count() - 1;
	if (index < 0) { index = 0; } // in case the end of camera combobox is not "add new camera"
	m_UI->cameraComboBox->insertItem(index, QString::fromLocal8Bit(info.sName), QVariant::fromValue(info));
	m_UI->cameraComboBox->setCurrentIndex(index);
}

std::vector<BlockDB::blkCameraInfo> bdrProjectDlg::getCameraData()
{
	std::vector<BlockDB::blkCameraInfo> camData;
	for (size_t i = 0; i < m_UI->cameraComboBox->count() - 1; i++) {
		QVariant v = m_UI->cameraComboBox->itemData(i);
		if (v.canConvert<BlockDB::blkCameraInfo>()) {
			camData.push_back(v.value<BlockDB::blkCameraInfo>());
		}
	}
	return camData;
}

void bdrProjectDlg::clicked_CameraComboBox(int index)
{
	if (index == m_UI->cameraComboBox->count() - 1) {

		bdrCameraParaDlg* camParaDlg = new bdrCameraParaDlg(this);
		camParaDlg->setCameraData(getCameraData());
		if (camParaDlg->exec()) {
			BlockDB::blkCameraInfo info = camParaDlg->getCameraInfo();
			addCameraData(info);
		}
	}
}

void bdrProjectDlg::doActionCameraOptions()
{
	QVariant v = m_UI->cameraComboBox->currentData();
	if (v.canConvert<BlockDB::blkCameraInfo>()) {
		BlockDB::blkCameraInfo info = v.value<BlockDB::blkCameraInfo>();
		bdrCameraParaDlg* camParaDlg = new bdrCameraParaDlg(this);
		camParaDlg->setCameraInfo(info);
		if (camParaDlg->exec()) {
			info = camParaDlg->getCameraInfo();
			m_UI->cameraComboBox->setItemData(m_UI->cameraComboBox->currentIndex(), QVariant::fromValue(info));
		}
	}
}

void bdrProjectDlg::doActionCameraSetup()
{
	if (m_UI->cameraComboBox->currentIndex() == m_UI->cameraComboBox->count() - 1) {
		QMessageBox::critical(this, "Error!", QString::fromLocal8Bit("请先添加相机"));
		return;
	}
	QTableWidget* tableWidget = getTableWidget(IMPORT_IMAGES);
	QString camName = m_UI->cameraComboBox->currentText();
	for (size_t i = 0; i < tableWidget->rowCount(); i++) {
		if (isRowChecked(tableWidget, i)) {
			tableWidget->item(i, ImagesCol_CamName)->setText(camName);
		}
	}
}

void bdrProjectDlg::doActionAddGcp()
{
}

void bdrProjectDlg::doActionProductCoor()
{
}

void bdrProjectDlg::doActionAddNewMisc()
{
	importDataType data_type = IMPORT_MISCS;
	listData::Container& list_datas = getListDatas(data_type);
	listData* data = listData::New(data_type);
	if (!data) { return; }

	//! fill the data
	data->m_index = list_datas.empty() ? 0 : (list_datas.back()->m_index + 1);
	data->m_groupID = getProjetcGroupID();

	QString level_cb = m_UI->productLevelComboBox->currentText();
	if (level_cb != "uno" && level_cb != "All") {
		data->m_level = level_cb;
	}
	if (insertItemToTable(data)) {
		list_datas.push_back(data);
	}
	else {
		delete data;
		data = nullptr;
	}
}

void bdrProjectDlg::doActionDisplayModelBdry()
{
}

void bdrProjectDlg::doActionViewModel()
{
}

void bdrProjectDlg::doActionExportModel()
{
}

void bdrProjectDlg::doActionImportDatabase()
{
	if (m_pgConnDlg->exec()) {

	}
}

void bdrProjectDlg::doActionSettingDatabase()
{
	if (m_pgConnDlg->exec()) {

	}
}

void bdrProjectDlg::doActionImportDBAll()
{
}

void bdrProjectDlg::doActionImportDBRegion()
{
}

void bdrProjectDlg::doActionQueryDB()
{
}

void bdrProjectDlg::doActionSelectAll()
{
	QTableWidget* tableWidget = getTableWidget(getCurrentTab());
	if (!tableWidget) { return; }
	const int count = tableWidget->rowCount();
	for (int i = 0; i < count; i++) {
		tableWidget->item(i, ListCol_ID)->setCheckState(Qt::Checked);
	}
}

void bdrProjectDlg::doActionToggleAll()
{
	QTableWidget* tableWidget = getTableWidget(getCurrentTab());
	if (!tableWidget) { return; }
	const int count = tableWidget->rowCount();
	for (int i = 0; i < count; i++) {
		tableWidget->item(i, ListCol_ID)->setCheckState(Qt::Unchecked);
	}
}

void bdrProjectDlg::doActionSelect()
{
	QTableWidget* tableWidget = getTableWidget(getCurrentTab());
	if (!tableWidget) { return; }
	const int count = tableWidget->rowCount();
	for (QTableWidgetItem* item : tableWidget->selectedItems())	{
		item->setCheckState(Qt::Checked);
	}
}

void bdrProjectDlg::doActionToggle()
{
	QTableWidget* tableWidget = getTableWidget(getCurrentTab());
	if (!tableWidget) { return; }
	const int count = tableWidget->rowCount();
	for (QTableWidgetItem* item : tableWidget->selectedItems()) {
		item->setCheckState((item->checkState() == Qt::Unchecked) ? Qt::Checked : Qt::Unchecked);
	}
}

void bdrProjectDlg::doActionPreview()
{
	updatePreview();
}

void bdrProjectDlg::acceptAndExit()
{
	apply();

	if (!generateProject()) {
		diaplayMessage("无法生成工程文件", PRJMSG_CRITICAL);
		return;
	}
	else {
		clear();
		
		accept();
	}
}

void bdrProjectDlg::doActionApply()
{
	apply();
	
	updatePreview();
	m_preview->getGLWindow()->zoomGlobal();	
}

void bdrProjectDlg::updatePreview()
{
	if (isPreviewEnable()) {
		QApplication::processEvents();
		ccGLWindow* glWin = m_preview->getGLWindow();
		if (glWin && m_ownProject) {
			m_ownProject->setDisplay_recursive(glWin);
			glWin->redraw();
			QApplication::processEvents();
		}
	}
}

void bdrProjectDlg::apply()
{
	ListToHObject();
}

bool bdrProjectDlg::generateProject()
{
	QString project_path = getProjectPath();
	if (project_path.isEmpty() || !QFileInfo(project_path).isDir()) {
		diaplayMessage("请设置正确的工程文件夹路径", PRJMSG_ERROR);
		return false;
	}
	if (!m_associateProject) { return false; }
	if (project_path != m_associateProject->getPath()) {
		m_associateProject->setPath(project_path);
	}
	if (m_ownProject) {
		return m_ownProject->save();
	}
	else {
		return false;
	}
}
