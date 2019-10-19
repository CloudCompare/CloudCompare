#ifndef BDR_CAMERAPARA_DLG_HEADER
#define BDR_CAMERAPARA_DLG_HEADER

#include "ui_bdrCameraParaDlg.h"
#include "BlockDBaseIO.h"

namespace Ui
{
	class bdrCameraParaDlg;
}

class bdrCameraParaDlg : public QDialog
{
	Q_OBJECT

public:

	explicit bdrCameraParaDlg(QWidget* parent = 0);
	~bdrCameraParaDlg() { m_camData.clear(); }

private:
	Ui::bdrCameraParaDlg	*m_UI;
	std::vector<BlockDB::blkCameraInfo> m_camData;
protected slots:

	void saveSettings();

public:
	
	void setCameraData(std::vector<BlockDB::blkCameraInfo> camData) { m_camData = camData; }
	void setCameraInfo(BlockDB::blkCameraInfo info);
	BlockDB::blkCameraInfo getCameraInfo();

};

#endif
