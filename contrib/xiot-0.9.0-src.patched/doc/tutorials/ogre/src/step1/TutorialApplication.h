#ifndef __TutorialApplication_H__
#define __TutorialApplication_H__

#include "Ogre.h"
#include "OgreConfigFile.h"
#include "ExampleFrameListener.h"
#include "ExampleApplication.h"


class TutorialApplication : public ExampleApplication
{
public:
    /// Standard constructor
    TutorialApplication() {};

	/// Standard destructor
	virtual ~TutorialApplication(){};

	// pure virtual - this has to be overridden
    virtual void createScene(void);    

};


#endif
