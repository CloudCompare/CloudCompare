#include "ccExternalFactoriesContainer.h"

static ccExternalFactoriesContainer * s_externalFactories = new ccExternalFactoriesContainer();

ccExternalFactoriesContainer::ccExternalFactoriesContainer()
{

}

ccExternalFactory *ccExternalFactoriesContainer::getFactoryByName(const QString factory_name) const
{
    int id = this->getFactoryIndexByName(factory_name);
    if (id >= 0)
        return m_factories.at(id);
    return 0;
}

void ccExternalFactoriesContainer::addFactory(ccExternalFactory *factory)
{
    if (getFactoryIndexByName(factory->getFactoryName()) == -1)
        m_factories.push_back(factory);
}

int ccExternalFactoriesContainer::getFactoryIndexByName(const QString factory_name) const
{
    int id =0;
    foreach (ccExternalFactory * fac, m_factories)
    {
        if (factory_name == fac->getFactoryName())
        {
            return id;
        }
        id++;
    }

    return -1;
}

ccExternalFactoriesContainer *ccExternalFactoriesContainer::getExternalFactoriesContainer()
{
    return s_externalFactories;
}

void ccExternalFactoriesContainer::setExternalFactoriesContainer(ccExternalFactoriesContainer * container)
{
    s_externalFactories = container;
}


