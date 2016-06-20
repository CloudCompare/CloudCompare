//#######################################################################################
//#                                                                                     #
//#                              CLOUDCOMPARE PLUGIN: qCSF                              #
//#                                                                                     #
//#  Please cite the following paper, If you use this plugin in your work.              #
//#                                                                                     #
//#  Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. An Easy-to-Use Airborne LiDAR  #
//#  Data Filtering Method Based on Cloth Simulation. Remote Sensing. 2016; 8(6):501.   #
//#                                                                                     #
//#                                     Copyright ©                                     #
//#               RAMM laboratory, School of Geography, Beijing Normal University       #
//#                               (http://ramm.bnu.edu.cn/)                             #
//#                                                                                     #
//#                      Wuming Zhang; Jianbo Qi; Peng Wan; Hongtao Wang                #
//#                                                                                     #
//#                      contact us: 2009zwm@gmail.com; wpqjbzwm@126.com                #
//#                                                                                     #
//#######################################################################################

#ifndef CC_CSF_DLG_HEADER
#define CC_CSF_DLG_HEADER

#include "ui_CSFDlg.h"

//! Dialog for qCSF plugin
class ccCSFDlg : public QDialog, public Ui::CSFDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccCSFDlg(QWidget* parent = 0);

protected slots:

	//! Saves (temporarily) the dialog paramters on acceptation
	void saveSettings();

};

#endif
