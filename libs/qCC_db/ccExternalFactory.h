#ifndef CC_EXTERNAL_FACTORY_H
#define CC_EXTERNAL_FACTORY_H

#include <ccHObject.h>
#include <QMap>

/**  Provide new objects with an external factory
  *  This is intendend to be used into plugins.
  *  Each plugin may define o new factory by subclassing this class
  *  factories are then managed in cloudcompare and used to load custom types.
 **/
class ccExternalFactory
{
public:

    /** a convenience holder for all factories
      * a static pointer to this class is declared in ccExternalFactoriesContainer.cpp
      * external tools using qCC_db lib may want to make this static pointer to point to their own
      * ccExternalFactoriesContainer
     **/
    class Container
    {
    public:
        //! def const will simply be an empty facotry set
        Container() { }

        /** get factory using a name as key
         *  null pointer if not found
         */
        ccExternalFactory * getFactoryByName(const QString factoryName) const;

        /** add a new factory to the container
         * it will overwrite a previously existent factory with the same name
         */
        void addFactory(ccExternalFactory * factory);

        /** returns the static getFactoriesContainer which returns the pointer to the
         * currently used external factories.
         */
        static ccExternalFactory::Container *GetExternalFactoriesContainer();

        /** set the static pointer to external factories
         *  the previous one will be deleted
         */
        static void SetExternalFactoriesContainer(ccExternalFactory::Container * container);

    protected:
        QMap<QString, ccExternalFactory *> m_factories;

    };

    //! constructor with name
    ccExternalFactory(QString factoryName);

    //! this will normally be the name of the plugin subclassing this class to provide a factory
    QString getFactoryName();

    //! a new operator, just like in ccHObject, but virtual (and not-static, of course)
    virtual ccHObject * buildObject(const QString metaname) = 0;

protected:
    QString m_factoryName;

};

#endif // CC_EXTERNAL_FACTORY_H
