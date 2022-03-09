#include "ccSetClassificationField.h"
#include "ui_setClassificationFieldDlg.h"

//qCC_db
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <QSettings>

ccSetClassificationFieldDlg::ccSetClassificationFieldDlg(QWidget *parent) : QDialog(parent)
  , m_ui(new Ui::ccSetClassificationField)
{
    m_ui->setupUi(this);
    readSettings();
}

ccSetClassificationFieldDlg::~ccSetClassificationFieldDlg()
{
    writeSettings();
    delete m_ui;
}

void ccSetClassificationFieldDlg::readSettings()
{
    QSettings settings("osur", "ccSetClassificationField");

    m_class = settings.value("m_class", 0).toInt();
    this->m_ui->spinBox_classification->setValue(m_class);
}

void ccSetClassificationFieldDlg::writeSettings()
{
    QSettings settings("osur", "ccSetClassificationField");

    int m_class = settings.value("m_class", 0).toInt();
    this->m_ui->spinBox_classification->setValue(m_class);
}

bool ccSetClassificationFieldDlg::setClassificationField(const ccHObject::Container &selectedEntities)
{
    QString errorMessage;
    m_class = getClass(); // update the internal class_ value
    for (ccHObject* ent : selectedEntities)
    {
        ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
        if (cloud != nullptr)
        {
            unsigned int N = cloud->size();

            // check that the scalar field Classification exists
            int idx = cloud->getScalarFieldIndexByName("Classification");

            // create the scalar field Classification if needed
            if (idx == -1)
                idx = cloud->addScalarField("Classification");

            CCCoreLib::ScalarField *sf = cloud->getScalarField(idx);

            if (sf != nullptr)
            {
                for (unsigned index = 0; index < N; index++)
                    sf->setValue(index, m_class);

                sf->computeMinAndMax();
                cloud->setCurrentDisplayedScalarField(idx);
            }
            else
                return false;
        }
    }

    return true;
}

int ccSetClassificationFieldDlg::getClass() const
{
    return m_ui->spinBox_classification->value();
}
