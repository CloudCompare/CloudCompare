#include "bdrProjectDlg.h"
#include "stocker_parser.h"
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

listData * listData::New(importDataType type)
{
	switch (type) {
	case IMPORT_POINTS:
		return new pointsListData();
	case IMPORT_IMAGES:
		return new imagesListData();
	default:
		break;
	}
	return nullptr;
}

ccHObject * listData::createObject()
{
	if (m_object) { delete m_object; m_object = nullptr; }
	//! fast load
	FileIOFilter::LoadParameters parameters;
	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	{
		parameters.alwaysDisplayLoadDialog = false;
		parameters.loadMode = 0;
	}

	return FileIOFilter::LoadFromFile(m_path, parameters, result, QString());
}

ccHObject * pointsListData::createObject()
{
	return listData::createObject();
}

ccHObject * imagesListData::createObject()
{
	return listData::createObject();
}

bdrProjectDlg::bdrProjectDlg(QWidget* parent)
	: m_UI(new Ui::bdrProjectDlg)
	, m_associateProject(nullptr)
	, m_ownProject(nullptr)
	, m_postGIS(nullptr)
	, m_preview(nullptr)
{
	m_UI->setupUi(this);

	m_UI->previewFrame->setVisible(false);
	m_UI->previewInfoGroupBox->setVisible(false);
	
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

	connect(m_UI->productLevelComboBox,			SIGNAL(currentIndexChanged(int)),	this, SLOT(onLevelChanged(int)));
	connect(m_UI->dataFilesTabWidget,			SIGNAL(currentChanged(int)),		this, SLOT(onDataFilesChanged(int)));
	connect(m_UI->buttonBox,					&QDialogButtonBox::accepted,		this, &bdrProjectDlg::acceptAndExit);
	connect(m_UI->importLocalFileToolButton,	&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionImportFile);
	connect(m_UI->importLocalFolderToolButton,	&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionImportFolder);
	connect(m_UI->importDBToolButton,			&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionImportDatabase);
	connect(m_UI->importDeleteToolButton,		&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionDelete);
	connect(m_UI->searchToolButton,				&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionSearch);
	connect(m_UI->searchCancleToolButton,		&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionSearchCancle);
	connect(m_UI->selectAllToolButton,			&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionSelectAll);
	connect(m_UI->toggleAllToolButton,			&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionToggleAll);
	connect(m_UI->selectToolButton,				&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionSelect);
	connect(m_UI->toggleToolButton,				&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionToggle);

	connect(m_UI->previewToolButton,			&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionPreview);
	
}

bdrProjectDlg::~bdrProjectDlg()
{

	delete m_UI;
}

void bdrProjectDlg::linkWithProject(DataBaseHObject * proj)
{
	if (m_associateProject != proj)	{
		m_associateProject = proj;
		//! CLEAR
		resetObjects();
		HObjectToList();
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
QTableWidget * bdrProjectDlg::getTableWidget(importDataType type)
{
	switch (type)
	{
	case IMPORT_POINTS:
		return m_UI->pointsTableWidget;
	case IMPORT_IMAGES:
		return m_UI->imagesTableWidget;
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
	default:
		throw std::runtime_error("internal error");
		assert(false);
		break;
	}
	return std::vector<listData*>();
}

void bdrProjectDlg::onLevelChanged(int)
{
}

void bdrProjectDlg::onDataFilesChanged(int index)
{

}

void bdrProjectDlg::onItemChanged(QTableWidgetItem * item)
{
	QTableWidget* tableWidget = item->tableWidget();
	
}

void bdrProjectDlg::onSelectionChanged(QTableWidget * table)
{
	if (m_UI->previewFrame->isVisible()) {

	}
}

void bdrProjectDlg::diaplayMessage(QString message, PRJ_ERROR_CODE error_code)
{
	switch (error_code)
	{
	case bdrProjectDlg::PRJMSG_STATUS:

		break;
	case bdrProjectDlg::PRJMSG_WARNING:
		break;
	case bdrProjectDlg::PRJMSG_ERROR:
		break;
	case bdrProjectDlg::PRJMSG_CRITICAL:
		QMessageBox::critical(this, "Error!", message);
		break;
	default:
		break;
	}
}

void bdrProjectDlg::closeEvent(QCloseEvent * e)
{
	e->accept();
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
	for (size_t i = 0; i < data_list_column[data_type]; i++) {
		tableWidget->setItem(table_index, i, new QTableWidgetItem);
	}
	//! fill index, and name
	tableWidget->item(table_index, ListCol_ID)->setCheckState(Qt::Checked);
	tableWidget->item(table_index, ListCol_ID)->setText(QString::number(data->m_index));
	tableWidget->item(table_index, ListCol_Name)->setText(data->m_name);

	switch (data_type)
	{
	case IMPORT_POINTS:
	{
		tableWidget->item(table_index, PointsCol_Path)->setText(data->m_path);
// 		QComboBox* combox_level = new QComboBox();
// 		combox_level->addItem("---");
// 		tableWidget->setCellWidget(table_index, PointsCol_Level, combox_level);
		break;
	}
	case IMPORT_IMAGES:
		tableWidget->item(table_index, ImagesCol_Path)->setText(data->m_path);
		break;
	default:
		assert(false);
		break;
	}
	tableWidget->scrollToBottom();

	return true;
}

bool bdrProjectDlg::addDataToTable(QString path, importDataType data_type)
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
		return false;
	}
	list_datas.push_back(data);
	return true;
}

bool bdrProjectDlg::ListToHObject(bool preview_control)
{
	//! points
	bool update = !preview_control || (preview_control && 1);
	if (update) {
		for (listData* data : m_points_data) {
			if (!data) continue;
			pointsListData* pData = static_cast<pointsListData*>(data); if (!pData) continue;
			ccHObject* object = pData->createObject();
			if (object)	{
				m_ownProject->addData(object, pData->getDataType(), pData->level);
			}
		}
	}

	//! images
	update = !preview_control || (preview_control && 1);
	if (update) {

	}

	//! miscs
	update = !preview_control || (preview_control && 1);
	if (preview_control && 1) {

	}

	//! postgis
	update = !preview_control || (preview_control && 1);
	if (0) {

	}

	return true;
}

bool bdrProjectDlg::HObjectToList()
{
	resetLists();


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

void bdrProjectDlg::doActionOpenProject()
{
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString path = QFileDialog::getExistingDirectory(this,
		tr("project directory"),
		QFileInfo(currentPath).absolutePath(),
		QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
	

	m_UI->projDirLineEdit->setText(path);
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
		if (!addDataToTable(file_path, getCurrentTab())) {

		}
	}
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
	switch (getCurrentTab())
	{
	case IMPORT_POINTS:
		nameFilters << "*.las" << "*.laz" << "*.ply" << "*.obj" << "*.pcd";
		break;
	case IMPORT_IMAGES:
		nameFilters << "*.jpg" << "*.tiff" << "*.tif" << "*.png" << "*.bmp";
		break;
	default:
		break;
	}

	QDirIterator dir_iter(dirname, nameFilters, QDir::Files | QDir::NoSymLinks | QDir::Readable, QDirIterator::Subdirectories);
	while (dir_iter.hasNext()) {
		files.append(dir_iter.next());
	}

	for (auto file_path : files) {
		if (!addDataToTable(file_path, getCurrentTab())) {

		}
	}
}

void bdrProjectDlg::doActionImportDatabase()
{
}

void bdrProjectDlg::doActionSearch()
{
}

void bdrProjectDlg::doActionSearchCancle()
{
}

void bdrProjectDlg::doActionDelete()
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
	if (m_UI->previewToolButton->isChecked()) {
		if (!m_preview)	{
			m_preview = new bdr2Point5DEditor();
			m_preview->create2DView(m_UI->previewFrame);
		}
	}
}

void bdrProjectDlg::acceptAndExit()
{
	if (!generateProject()) {
		diaplayMessage(QString::fromUtf8("无法生成工程文件"), PRJMSG_CRITICAL);
		return;
	}
	else {
		resetObjects();
		
		accept();
	}
}

bool bdrProjectDlg::generateProject()
{
	QString project_path = getProjectPath();
	if (project_path.isEmpty() || !QFileInfo(project_path).isDir()) {
		diaplayMessage(QString::fromUtf8("请设置正确的工程文件夹路径"), PRJMSG_ERROR);
		return false;
	}
	if (!m_associateProject) { return false; }
	if (project_path != m_associateProject->getPath()) {
		m_associateProject->setPath(project_path);
	}
	if (ListToHObject() && m_ownProject) {
		return m_ownProject->save();
	}
	else {
		return false;
	}
}

