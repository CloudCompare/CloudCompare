#ifndef CC_EXTERNAL_FACTORIES_CONTAINER_H
#define CC_EXTERNAL_FACTORIES_CONTAINER_H

#include "ccExternalFactory.h"


//! a convenience holder for all factories
/** a static pointer to this class is declared in ccExternalFactoriesContainer.cpp
  * external tools using qCC_db lib may want to make this static pointer to point to their own
  * ccExternalFactoriesContainer
 **/
class ccExternalFactoriesContainer
{
public:
    //! def const will simply be an empty facotry set
    ccExternalFactoriesContainer();

    ccExternalFactory * getFactoryByName(const QString factory_name) const;

    void addFactory(ccExternalFactory * factory);

    int getFactoryIndexByName(const QString plugin_name) const;

    //! this is the static getFactoriesContainer which returns the pointer to the
    /** currently used external factories.
     */
    static ccExternalFactoriesContainer * getExternalFactoriesContainer();

    static void setExternalFactoriesContainer(ccExternalFactoriesContainer * container);

protected:
    std::vector<ccExternalFactory *> m_factories;

};

#endif // CC_EXTERNAL_FACTORIES_CONTAINER_H
