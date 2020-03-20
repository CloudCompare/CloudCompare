// sip implementation of ccPointCloud

//%Import QtCore/QtCoremod.sip
//%Import QtGui/QtGuimod.sip

%ExportedHeaderCode
#include <ccPointCloud.h>
%End

class ccPointCloud
{
%TypeHeaderCode
#include <ccPointCloud.h>
%End

public:
    ccPointCloud(QString name = QString(), unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID) throw();

    QString getName() const;
};
    