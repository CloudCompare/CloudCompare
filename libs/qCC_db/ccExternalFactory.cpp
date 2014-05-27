#include "ccExternalFactory.h"

ccExternalFactory::ccExternalFactory(QString facotry_name)
{
    m_factoryName = facotry_name;

}

QString ccExternalFactory::getFactoryName()
{
    return m_factoryName;
}


