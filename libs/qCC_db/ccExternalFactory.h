#ifndef CC_EXTERNAL_FACTORY_H
#define CC_EXTERNAL_FACTORY_H

#include <ccHObject.h>

/** \brief Provide new objects with an external factory
  *  This is intendend to be used into plugins.
  *  Each plugin may define o new factory by subclassing this class
  *  factories are then managed in cloudcompare and used to load custom types.
 **/
class ccExternalFactory
{
public:
    ccExternalFactory(QString facotry_name);

    //! this will normally be the name of the plugin subclassing this class to provide a factory
    QString getFactoryName();

    //! a new operator, just like in ccHObject, but virtual (and not-static, of course)
    virtual ccHObject * New(const QString metaname, const char * name = 0) = 0;

protected:
    QString m_factoryName;
};

#endif // CC_EXTERNAL_FACTORY_H
