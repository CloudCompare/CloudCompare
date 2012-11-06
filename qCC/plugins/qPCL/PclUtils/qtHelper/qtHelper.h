#ifndef Q_PCL_PLUGIN_QTHELPER_H
#define Q_PCL_PLUGIN_QTHELPER_H

#include <QComboBox>
#include <ccPointCloud.h>
#include <QWidget>




class qPCLComboBox : public QComboBox
{
  Q_OBJECT
public :
  qPCLComboBox(QWidget * parent);
  
  ~qPCLComboBox();
  
  
  void populateByScalarsInCloud(const ccPointCloud * cloud);
};

#endif //Q_PCL_PLUGIN_QTHELPER_H