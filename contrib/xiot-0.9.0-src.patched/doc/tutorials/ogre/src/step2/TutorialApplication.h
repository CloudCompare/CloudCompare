#ifndef __TutorialApplication_H__
#define __TutorialApplication_H__

#include "Ogre.h"
#include "OgreConfigFile.h"
#include "ExampleApplication.h"


class TutorialApplication : public ExampleApplication
{
public:
    /// Standard constructor
    TutorialApplication() {};

	/// Standard destructor
	virtual ~TutorialApplication(){};

    virtual void createScene(void);    // pure virtual - this has to be overridden
	void setX3DFile(const char* filename);

	std::string _filename;
};


#endif
