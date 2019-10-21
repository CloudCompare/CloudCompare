/*
File	:		bdrProjectDlg.h
Brief	:		The header file of bdrProjectDlg

Author	:		Xinyi Liu
Date	:		2019/10/20
E-mail	:		liuxy0319@outlook.com
*/

#ifndef BDR_PROJECT_DLG_HEADER
#define BDR_PROJECT_DLG_HEADER
#include <vector>
#include "ui_bdrProjectDlg.h"

#include "stocker_parser.h"
#include "ccBBox.h"

class StHObject;
class bdr2Point5DEditor;
class bdrViewerControlDlg;
class bdrPosImportDlg;
class bdrPGConnDlg;
class bdrLasTilesDlg;
class bdrCameraParaDlg;

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
		, m_displayed(false)
	{}
	~listData() {}
	using Container = std::vector<listData*>;
	static listData* New(importDataType type);
public:
	virtual importDataType getDataType() const { return IMPORT_TYPE_END; };
	StHObject* getObject() { return m_object; }
	void setObject(StHObject* obj) { m_object = obj; }
	virtual void createObject(BlockDB::blkDataInfo* info);

	virtual void toBlkDataInfo(BlockDB::blkDataInfo* info);
	virtual void fromBlkDataInfo(BlockDB::blkDataInfo* info);

	bool isDisplayed() { return m_displayed; }
	void setDisplayed(bool dis) { m_displayed = dis; }
	int m_index;
	int m_groupID;
	QString m_name;
	QString m_path;
	QString m_level;
protected:
	StHObject* m_object;
private:
	bool m_displayed;
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
	virtual void createObject(BlockDB::blkDataInfo* info) override;

	virtual void toBlkDataInfo(BlockDB::blkDataInfo* info) override;
	virtual void fromBlkDataInfo(BlockDB::blkDataInfo* info) override;

	size_t m_pointCnt;
	QStringList m_assLevels;
protected:
private:
};

enum imagesHeaderIdx
{
	ImagesCol_CamName = ListCol_END,
	ImagesCol_PosXs,
	ImagesCol_PosYs,
	ImagesCol_PosZs,
	ImagesCol_PosPhi,
	ImagesCol_PosOmega,
	ImagesCol_PosKappa,
	ImagesCol_GpsLat,
	ImagesCol_GpsLot,
	ImagesCol_GpsHgt,
	ImagesCol_GpsTime,
	ImagesCol_GroupID,
	ImagesCol_Level,
	ImagesCol_Path,
	ImagesCol_End,
};
static const char* imagesHeaderName[] = { "CAM", "Xs", "Ys", "Zs", "Phi", "Omega", "Kappa", "GPS lat","GPS lot","GPS hgt", "GPS time", "GroupID", "level", "Path" };
class imagesListData : public listData
{
public:
	imagesListData()
		: posXs(NAN), posYs(NAN), posZs(NAN), posPhi(NAN), posOmega(NAN), posKappa(NAN)
		, gpsLat(NAN), gpsLon(NAN), gpsHeight(NAN), gps_time(NAN)
	{}
	~imagesListData() {}
	virtual importDataType getDataType() const override { return IMPORT_IMAGES; }
	virtual void createObject(BlockDB::blkDataInfo* info) override;

	virtual void toBlkDataInfo(BlockDB::blkDataInfo* info) override;
	virtual void fromBlkDataInfo(BlockDB::blkDataInfo* info) override;

	QString m_cam;
	double posXs, posYs, posZs, posPhi, posOmega, posKappa;
	double gpsLat, gpsLon, gpsHeight, gps_time;
protected:
private:
};

enum miscsHeaderIdx
{
	MiscsCol_Level = ListCol_END,
	MiscsCol_MetaKey,
	MiscsCol_MetaValue,
	MiscsCol_GroupID,
	MiscsCol_Path,
	MicscCol_End,
};
static const char* miscsHeaderName[] = { "Type", "Meta Key", "Meta Value", "GroupID", "Path" };
class miscsListData : public listData
{
public:
	miscsListData()
	{}
	~miscsListData() {}
	virtual importDataType getDataType() const override { return IMPORT_MISCS; }
	virtual void createObject(BlockDB::blkDataInfo* info) override;
	QString m_meta_key;
	QString m_meta_value;
};

enum modelsHeaderIdx
{
	ModelsCol_Level = ListCol_END,
	ModelsCol_Origin,
	ModelsCol_MetaKey,
	ModelsCol_MetaValue,
	ModelsCol_GroupID,
	ModelsCol_Path,
	ModelsCol_End,
};
static const char* modelsHeaderName[] = { "Type", "Origin", "Meta Key", "Meta Value", "GroupID", "Path" };
class modelsListData : public listData
{
public:
	modelsListData()
	{}
	~modelsListData() {}
	virtual importDataType getDataType() const override { return IMPORT_MODELS; }
	virtual void createObject(BlockDB::blkDataInfo* info) override;
	QString m_origin;
	QString m_meta_key;
	QString m_meta_value;
};

enum postGISHeaderIdx
{
	PostgisCol_Level = ListCol_END,
	PostgisCol_MetaKey,
	PostgisCol_MetaValue,
	PostgisCol_Path,
	PostgisCol_End,
};
static const char* postgisHeaderName[] = { "Type", "Meta Key", "Meta Value", "Path" };
class postGISLiistData : public listData
{
public:
	postGISLiistData()
	{}
	~postGISLiistData() {}
	virtual importDataType getDataType() const override { return IMPORT_POSTGIS; }
	virtual void createObject(BlockDB::blkDataInfo* info) override;
	QString m_meta_key;
	QString m_meta_value;
};

//! should be the same size of importDataType
static const char** data_list_names[] = { pointsHeaderName, imagesHeaderName, miscsHeaderName, modelsHeaderName, postgisHeaderName };
static const size_t data_list_column[] = { PointsCol_END, ImagesCol_End, MicscCol_End, ModelsCol_End, PostgisCol_End };

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

	void clear();
protected slots:
	
	void doActionOpenProject();
	void doActionImportFile();
	void doActionImportFolder();
	void doActionExportDB();
	
	void doActionDelete();

	void onLevelChanged(int);
	void onDataFilesChanged(int);
	void onItemChanged(QTableWidgetItem* item);
	void onSelectionChanged(QTableWidget* table);

	void doActionSearch();
	void doActionSearchCancle();
	void doActionLevelFilter();
	void doActionLevelSetup();
	// para
	void doActionGroupIDSetup();
	void doActionReorganizeIndex();

	// points tab
	void doActionLasTiles();
	void doActionPointsBoundary();
	void doActionSearchAssLevel();
	// images tab
	void doActionPosFile();
	void doActionGpsInfo();
	void doActionImgCoorCvt();
	void doActionImageUndistort();
	void clicked_CameraComboBox(int);
	void doActionCameraOptions();
	void doActionCameraSetup();
	// miscs tab
	void doActionAddGcp();
	void doActionProductCoor();
	void doActionAddNewMisc();
	// models tab
	void doActionDisplayModelBdry();
	void doActionViewModel();
	void doActionExportModel();
	// postgis tab
	void doActionImportDatabase();
	void doActionSettingDatabase();
	void doActionImportDBAll();
	void doActionImportDBRegion();
	void doActionQueryDB();

	void doActionSelectAll();
	void doActionToggleAll();
	void doActionSelect();
	void doActionToggle();

	void doActionPreview();
	void acceptAndExit();
	void doActionApply();

	void apply();

private: //members
	Ui::bdrProjectDlg	*m_UI;
	bdr2Point5DEditor* m_preview;

	enum PRJ_ERROR_CODE { PRJMSG_STATUS, PRJMSG_WARNING, PRJMSG_ERROR, PRJMSG_CRITICAL };
	void diaplayMessage(QString message, PRJ_ERROR_CODE error_code = PRJMSG_STATUS);
	void diaplayMessage(const char * message, PRJ_ERROR_CODE error_code = PRJMSG_STATUS);
	void updatePreview();
protected:

	DataBaseHObject* m_associateProject;
	DataBaseHObject* m_ownProject;
	StHObject* m_postGIS;

	bool insertItemToTable(listData* data);
	importDataType getCurrentTab();
	importDataType getCurrentTab(QTableWidget* widget);
	QTableWidget* getTableWidget(importDataType type);
	listData::Container& getListDatas(importDataType type);

	//void mouseMoveEvent(QMouseEvent *event);
	bool event(QEvent* evt) override;
	void echoMouseMoved(int x, int y, Qt::MouseButtons buttons);
public:
	void linkWithProject(DataBaseHObject* proj);
	bool generateProject();
	QString getProjectPath();
	void setProjectPath(QString path, bool enable = true);
	int getProjectID();
	int getProjetcGroupID();
	listData* addFilePathToTable(QString path, importDataType data_type);
	//! set list data to HObject before preview and 
	bool ListToHObject(bool preview_control = false);
	//! set the list data from HObject
	bool HObjectToList(StHObject* projObj);
	void resetLists();
	void resetObjects();
	bool isPreviewEnable();

	void addCameraData(const BlockDB::blkCameraInfo & info);
	std::vector<BlockDB::blkCameraInfo> getCameraData();

	listData::Container m_points_data;
	listData::Container m_images_data;
	listData::Container m_miscs_data;
	listData::Container m_models_data;
	listData::Container m_postgis_data;

	//! dialogs
	bdrViewerControlDlg* m_viewerCtrlDlg;
	bdrPosImportDlg* m_posImportDlg;
	bdrPGConnDlg* m_pgConnDlg;
	bdrLasTilesDlg*	m_lasTilesDlg;
};

#endif //BDR_PROJECT_DLG_HEADER
