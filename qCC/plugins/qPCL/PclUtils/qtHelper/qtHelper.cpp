#include "qtHelper.h"



qPCLComboBox::qPCLComboBox(QWidget * parent) : QComboBox( parent ) 
{
  int i = 1;
};

void qPCLComboBox::populateByScalarsInCloud(const ccPointCloud* cloud)
{ 
  int n_scalars = cloud->getNumberOfScalarFields();
  for (int i = 0; i < n_scalars; ++i)
  {
    QString name = cloud->getScalarFieldName(i);    
    this->addItem(name, i);
  }
}
