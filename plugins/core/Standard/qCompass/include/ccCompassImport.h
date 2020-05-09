#ifndef CCCOMPASSIMPORT_H
#define CCCOMPASSIMPORT_H

class QString;

class ccMainAppInterface;


namespace ccCompassImport
{
	void importFoliations( ccMainAppInterface *app ); //import foliation data
	void importLineations( ccMainAppInterface *app ); //import lineation data
};

#endif // CCCOMPASSIMPORT_H
