#include "bdrProjectDlg.h"
#include "stocker_parser.h"
#include <QFileDialog>
#include <QFileInfo>
#include <QSettings>
#include <QDirIterator>
#include <QCheckBox>
#include "ccPersistentSettings.h"
#include "ccFileUtils.h"
#include "ccHObject.h"
#include "ccHObjectCaster.h"
#include "FileIOFilter.h"
#include "FileIO.h"


bdrProjectDlg::bdrProjectDlg(QWidget* parent)
	: m_UI(new Ui::bdrProjectDlg)
	, m_associateProject(nullptr)
	, m_import_data_type(IMPORT_POINTS)
{
	m_UI->setupUi(this);

	m_UI->previewFrame->setVisible(false);
	m_UI->previewInfoGroupBox->setVisible(false);
	
	m_UI->pointCloudsTableWidget->horizontalHeader()->sectionsMovable();
	m_UI->pointCloudsTableWidget->horizontalHeader()->resizeSection(0, 50);

	m_UI->imagesTableWidget->horizontalHeader()->sectionsMovable();
	m_UI->imagesTableWidget->horizontalHeader()->resizeSection(0, 50);

	connect(m_UI->productLevelComboBox,			SIGNAL(currentIndexChanged(int)),	this, SLOT(onLevelChanged(int)));
	connect(m_UI->dataFilesTabWidget,			SIGNAL(currentChanged(int)),		this, SLOT(onDataFilesChanged(int)));
	connect(m_UI->buttonBox,					SIGNAL(accepted()),					this, SLOT(generateProject()));
	connect(m_UI->importLocalFileToolButton,	&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionImportFile);
	connect(m_UI->importLocalFolderToolButton,	&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionImportFolder);
	connect(m_UI->importDBToolButton,			&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionImportDatabase);
	connect(m_UI->importDeleteToolButton,		&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionDelete);
	connect(m_UI->searchToolButton,				&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionSearch);
	connect(m_UI->searchCancleToolButton,		&QAbstractButton::clicked,			this, &bdrProjectDlg::doActionSearchCancle);
}

bdrProjectDlg::~bdrProjectDlg()
{

	delete m_UI;
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
	if (!loadProject(path)) {
		return;
	}
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
		if (!addDataToTable(file_path, m_import_data_type)) {

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
	switch (m_import_data_type)
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
		if (!addDataToTable(file_path, m_import_data_type)) {

		}
	}
}

void bdrProjectDlg::doActionImportDatabase()
{
}

void bdrProjectDlg::onLevelChanged(int)
{
}

void bdrProjectDlg::onDataFilesChanged(int index)
{
	m_import_data_type = importDataType(index);
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
}

void bdrProjectDlg::doActionToggleAll()
{
}

void bdrProjectDlg::doActionSelect()
{
}

void bdrProjectDlg::doActionToggle()
{
}

void bdrProjectDlg::acceptAndExit()
{

}

bool bdrProjectDlg::generateProject()
{
	QString project_path = getProjectPath();
	if (project_path.isEmpty()) {
		return false;
	}
	return true;
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
	
	tableWidget->setRowCount(table_index + 1);
	tableWidget->setVerticalHeaderItem(table_index, new QTableWidgetItem);
	tableWidget->setCellWidget(table_index, 0, new QCheckBox);

	for (size_t i = 0; i < data_list_column[data_type]; i++) {
		tableWidget->setItem(table_index, i, new QTableWidgetItem);
	}
	
	tableWidget->verticalHeaderItem(table_index)->setText(QString::number(table_index + 1));
	QCheckBox* index_checkbox = static_cast<QCheckBox*>(tableWidget->cellWidget(table_index, 0));
	if (index_checkbox) index_checkbox->setText(QString::number(data->m_index)); else return false;
	tableWidget->item(table_index, 1)->setText(data->m_name);
	switch (data_type)
	{
	case IMPORT_POINTS:
		tableWidget->item(table_index, 6)->setText(data->m_path);
		break;
	case IMPORT_IMAGES:
		break;
	default:
		assert(false);
		break;
	}
	tableWidget->scrollToBottom();

	return true;
}

QTableWidget * bdrProjectDlg::getTableWidget(importDataType type)
{
	switch (type)
	{
	case IMPORT_POINTS:
		return m_UI->pointCloudsTableWidget;
	case IMPORT_IMAGES:
		return m_UI->imagesTableWidget;
	default:
		throw std::runtime_error("internal error");
		assert(false);
		break;
	}
	return nullptr;
}

std::vector<listData*> bdrProjectDlg::getListDatas(importDataType type)
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

void bdrProjectDlg::linkWithProject(ccHObject * proj)
{
	m_associateProject = proj;
}

bool bdrProjectDlg::loadProject(QString path)
{
	return true;
}

QString bdrProjectDlg::getProjectPath()
{
	return m_UI->projDirLineEdit->text();
}

int bdrProjectDlg::getProjectID()
{
	return m_UI->projIDSpinBox->value();
}

int bdrProjectDlg::getProjetcGroupID()
{
	return m_UI->groupIDSpinBox->value();
}

bool bdrProjectDlg::addDataToTable(QString path, importDataType data_type)
{
	listData::Container list_datas = getListDatas(data_type);
	listData* data = nullptr;
	switch (data_type)
	{
	case IMPORT_POINTS:
		data = new pointsListData();
		list_datas.push_back(data);
		break;
	case IMPORT_IMAGES:
		data = new imagesListData();
		list_datas.push_back(data);
		break;
	default:
		return false;
	}
	//! load data
	

	//! fill the data
	data->m_name = QFileInfo(path).completeBaseName();
	data->m_path = path;
	data->m_index = list_datas.empty() ? 0 : (list_datas.back()->m_index + 1);
	data->m_groupID;

	if (!data) { return false; }
	try {
		if (!insertItemToTable(data)) {
			
			list_datas.pop_back();
			delete data;
			data = nullptr;
		}
	}
	catch (const std::exception&) {
		return false;
	}
	
	return true;
}
