#ifndef BDR_PROJECT_DLG_HEADER
#define BDR_PROJECT_DLG_HEADER
#include <vector>
#include "ui_bdrProjectDlg.h"

#include "stocker_parser.h"

class ccHObject;
class bdr2Point5DEditor;

enum listHeaderIdx
{
	ListCol_ID,
	ListCol_Name,
	ListCol_END,
};
static const char* listHeaderName[] = { "ID", "Name" };
class listData
{
public:
	listData()
		: m_index(-1)
		, m_groupID(-1)
		, m_object(nullptr)
	{}
	~listData() {
		if (m_object) {
			delete m_object;
			m_object = nullptr;
		}
	}
	using Container = std::vector<listData*>;
	static listData* New(importDataType type);
public:
	virtual importDataType getDataType() const { return IMPORT_POINTS; };
	ccHObject* getObject() { return m_object; }
	virtual ccHObject* createObject();
	int m_index;
	int m_groupID;
	QString m_name;
	QString m_path;
protected:
	ccHObject* m_object;
};

enum pointsHeaderIdx
{
	PointsCol_PtsCnt = ListCol_END,
	PointsCol_Level,
	PointsCol_AssLv,
	PointsCol_GroupID,
	PointsCol_Path,
	PointsCol_END,
};
static const char* pointsHeaderName[] = { "Points", "Level", "Assoc.Levels", "GroupID", "Path" };

class pointsListData : public listData
{
public:
	pointsListData()
		: m_pointCnt(0)
	{}
	~pointsListData() {}
	virtual importDataType getDataType() const override { return IMPORT_POINTS; }
	virtual ccHObject* createObject() override;
	size_t m_pointCnt;
	QString level;
	QStringList AssLevels;
protected:
private:
};

enum imagesHeaderIdx
{
	ImagesCol_PosXs = ListCol_END,
	ImagesCol_PosYs,
	ImagesCol_PosZs,
	ImagesCol_PosPhi,
	ImagesCol_PosOmega,
	ImagesCol_PosKappa,
	ImagesCol_GPSTime,
	ImagesCol_GroupID,
	ImagesCol_Path,
	ImagesCol_End,
};
static const char* imagesHeaderName[] = { "Xs", "Ys", "Zs", "Phi", "Omega", "Kappa", "GPS time", "GroupID", "Path" };
class imagesListData : public listData
{
public:
	imagesListData()
		: pos_x(0)
	{}
	~imagesListData() {}
	virtual importDataType getDataType() const override { return IMPORT_IMAGES; }
	virtual ccHObject* createObject() override;
	double pos_x;
protected:
private:
};

//! should be the same size of importDataType
static const char** data_list_names[] = { pointsHeaderName, imagesHeaderName };
static const size_t data_list_column[] = { PointsCol_END, ImagesCol_End };

namespace Ui
{
	class bdrProjectDlg;
}

//! Section extraction tool
class bdrProjectDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrProjectDlg(QWidget* parent);
	//! Destructor
	~bdrProjectDlg() override;


protected slots:
	
	void doActionOpenProject();
	void doActionImportFile();
	void doActionImportFolder();
	void doActionImportDatabase();
	void doActionDelete();

	void onLevelChanged(int);
	void onDataFilesChanged(int);
	void onItemChanged(QTableWidgetItem* item);
	void onSelectionChanged(QTableWidget* table);

	void doActionSearch();
	void doActionSearchCancle();
	void doActionSelectAll();
	void doActionToggleAll();
	void doActionSelect();
	void doActionToggle();
	void doActionPreview();
	void acceptAndExit();	

private: //members
	Ui::bdrProjectDlg	*m_UI;
	bdr2Point5DEditor* m_preview;

	enum PRJ_ERROR_CODE { PRJMSG_STATUS, PRJMSG_WARNING, PRJMSG_ERROR, PRJMSG_CRITICAL };
	void diaplayMessage(QString message, PRJ_ERROR_CODE error_code = PRJMSG_STATUS);
protected:
	void closeEvent(QCloseEvent *) override;

	DataBaseHObject* m_associateProject;
	DataBaseHObject* m_ownProject;
	ccHObject* m_postGIS;

	bool insertItemToTable(listData* data);
	importDataType getCurrentTab();
	QTableWidget* getTableWidget(importDataType type);
	listData::Container& getListDatas(importDataType type);
public:
	void linkWithProject(DataBaseHObject* proj);
	bool generateProject();
	QString getProjectPath();
	void setProjectPath(QString path, bool enable = true);
	int getProjectID();
	int getProjetcGroupID();
	bool addDataToTable(QString path, importDataType data_type);
	//! set list data to HObject before preview and 
	bool ListToHObject(bool preview_control = false);
	//! set the list data from HObject
	bool HObjectToList();
	void resetLists();
	void resetObjects();

	listData::Container m_points_data;
	listData::Container m_images_data;
	listData::Container m_miscs_data;
	listData::Container m_postgis_data;
};

#endif //BDR_PROJECT_DLG_HEADER
