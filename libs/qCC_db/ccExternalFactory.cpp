#include "ccExternalFactory.h"

static ccExternalFactory::Container * s_externalFactories = new ccExternalFactory::Container();

ccExternalFactory::ccExternalFactory(QString factoryName)
{
    m_factoryName = factoryName;

}

QString ccExternalFactory::getFactoryName()
{
    return m_factoryName;
}

///// CONTAINER STUFF


ccExternalFactory *ccExternalFactory::Container::getFactoryByName(const QString factoryName) const
{
    if (m_factories.contains(factoryName))
        return m_factories.value(factoryName);
    else
        return 0;
}

void ccExternalFactory::Container::addFactory(ccExternalFactory *factory)
{
    if (!factory) //do nothing
        return;

    QString name = factory->getFactoryName();

    m_factories[name] = factory;
}


ccExternalFactory::Container *ccExternalFactory::Container::GetExternalFactoriesContainer()
{
    return s_externalFactories;
}

void ccExternalFactory::Container::SetExternalFactoriesContainer(ccExternalFactory::Container * container)
{
    if (container)
    {
        s_externalFactories = container;
    }
}



