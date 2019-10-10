#ifndef BDR_PROJECT_DLG_HEADER
#define BDR_PROJECT_DLG_HEADER

#include "ui_bdrProjectDlg.h"
#include <vector>
class ccHObject;

enum importDataType
{
	IMPORT_POINTS,
	IMPORT_IMAGES,
};
static const size_t data_list_column[] = { 9, 10 };

class listData
{
public:
	listData()
		: m_index(-1)
		, m_groupID(-1)
	{}
	using Container = std::vector<listData*>;
public:
	virtual importDataType getDataType() const { return IMPORT_POINTS; };
	int m_index;
	int m_groupID;
	QString m_name;
	QString m_path;
protected:
};

class pointsListData : public listData
{
public:
	pointsListData()
		: m_pointCnt(0)
	{}
	~pointsListData() {}
	virtual importDataType getDataType() const override { return IMPORT_POINTS; }
	size_t m_pointCnt;
	QString level;
	QStringList AssLevels;
protected:
private:
};

class imagesListData : public listData
{
public:
	imagesListData()
		: pos_x(0)
	{}
	~imagesListData() {}
	virtual importDataType getDataType() const override { return IMPORT_IMAGES; }
	double pos_x;
protected:
private:
};

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

	void doActionSearch();
	void doActionSearchCancle();
	void doActionSelectAll();
	void doActionToggleAll();
	void doActionSelect();
	void doActionToggle();
	void acceptAndExit();	

private: //members
	Ui::bdrProjectDlg	*m_UI;

protected:
	ccHObject* m_associateProject;

	bool insertItemToTable(listData* data);
	QTableWidget* getTableWidget(importDataType type);
	listData::Container getListDatas(importDataType type);
public:
	void linkWithProject(ccHObject* proj);
	bool loadProject(QString path);
	bool generateProject();
	QString getProjectPath();
	int getProjectID();
	int getProjetcGroupID();
	bool addDataToTable(QString path, importDataType data_type);

	importDataType	m_import_data_type;
	listData::Container m_points_data;
	listData::Container m_images_data;
};

#endif //BDR_PROJECT_DLG_HEADER
