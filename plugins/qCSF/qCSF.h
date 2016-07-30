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

// A mex version for programming in Matlab is at File Exchange of Mathworks website:
// http://www.mathworks.com/matlabcentral/fileexchange/58139-csf--ground-filtering-of-point-cloud-based-on-cloth-simulation

#ifndef Q_CSF_PLUGIN_HEADER
#define Q_CSF_PLUGIN_HEADER

#include "../ccStdPluginInterface.h"

//! A point-clouds filtering algorithm utilize cloth simulation process. 
class qCSF : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qCSF")

public:

	//! Default constructor
	explicit qCSF(QObject* parent = 0);

	//inherited from ccPluginInterface
	virtual QString getName() const { return "CSF Filter"; }
	virtual QString getDescription() const { return "A pointclouds filtering algorithm utilize cloth simulation process(Wuming Zhang; Jianbo Qi; Peng Wan,2015)."; }
	virtual QIcon getIcon() const;

	//inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities);
	virtual void getActions(QActionGroup& group);

protected slots:

	//! Slot called when associated ation is triggered
	void doAction();

protected:

	//! Associated action
	QAction* m_action;
};

#endif
