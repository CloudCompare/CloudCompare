#include "ccSetClassificationField.h"
#include "ui_setClassificationFieldDlg.h"

//qCC_db
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>

ccSetClassificationField::ccSetClassificationField(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ccSetClassificationField)
{
    ui->setupUi(this);
}

ccSetClassificationField::~ccSetClassificationField()
{
    delete ui;
}

bool ccSetClassificationField::setClassificationField(const ccHObject::Container &selectedEntities)
{
    QString errorMessage;
    int class_ = getClass();
    for (ccHObject* ent : selectedEntities)
    {
        ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
        unsigned int N = cloud->size();
        if (cloud != nullptr)
        {
            // check that the scalar field Classification exists
            int idx = cloud->getScalarFieldIndexByName("Classification");
            CCCoreLib::ScalarField *sf = nullptr;

            // create the scalar field Classification if needed
            if (idx == -1)
            {
                idx = cloud->addScalarField("Classification");
                sf = cloud->getScalarField(idx);
            }
            else
            {
                sf = cloud->getScalarField(idx);
            }

            for (unsigned index = 0; index < N; index++)
            {
                sf->setValue(index, class_);
            }

            if (sf != nullptr)
            {
                sf->computeMinAndMax();
                cloud->setCurrentDisplayedScalarField(idx);
            }
        }
    }

    return true;
}


int ccSetClassificationField::getClass()
{
    return ui->spinBox_classification->value();
}
