//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef GENERIC_PROGRESS_CALLBACK_HEADER
#define GENERIC_PROGRESS_CALLBACK_HEADER

//system
#include <assert.h>
#include <math.h>

namespace CCLib
{

//! A generic progress indicator interface to notify algorithms progress to the client application

#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"

class CC_DLL_API GenericProgressCallback
#else
class GenericProgressCallback
#endif
{
public:

	//! Default destructor
	virtual ~GenericProgressCallback() {};

	//! Resets the progress status
	/** The progress (percentage) is set to 0, and the title/infos are cleared.
	**/
	virtual void reset()=0;

	//! Notifies the algorithm progress
	/** The notification is sent by the ongoing algorithm (on the library side).
        This virtual method shouldn't be called too often, as the real process
        behind it is unspecified and may be time consuming. Typically it shouldn't
        be called more than once per percent.
		\param percent the current progress, between 0 and 100%
	**/
	virtual void update(float percent)=0;

	//! Notifies the algorithm title
	/** The notification is sent by the ongoing algorithm (on the library side).
		\param methodTitle the algorithm title
	**/
	virtual void setMethodTitle(const char* methodTitle)=0;

	//! Notifies some information about the ongoing process
	/** The notification is sent by the ongoing algorithm (on the library side).
		\param infoStr some textual information about the ongoing process
	**/
	virtual void setInfo(const char* infoStr)=0;

	//! Notifies the fact that every information has been sent and that the process begins
	/** Once start() is called, the progress bar and other informations could be displayed (for example).
	**/
	virtual void start()=0;

	//! Notifies the fact that the process has ended
	/** Once end() is called, the progress bar and other informations could be hidden (for example).
	**/
	virtual void stop()=0;

	//! Checks if the process should be canceled
	/** This method is called by some process from time to time to know if it
	should halt before its normal ending. This is a way for the client application
	to cancel an ongoing process (but it won't work with all algorithms).
	Process results may be incomplete/void. The cancel requirement mechanism must
	be implemented (typically a simple "cancel()" method that will be called by the
	client application).
	**/
	virtual bool isCancelRequested()=0;

};

//! Efficient management of progress based on a total number of steps different than 100
class NormalizedProgress
{
public:
	//! Default constructor
	/** \param callback associated GenericProgressCallback
		\param totalSteps total number of steps
		\param totalPercentage equivalent percentage
	**/
	NormalizedProgress(GenericProgressCallback* callback, unsigned totalSteps, unsigned totalPercentage=100)
		: percent(0.0f)
		, step(1)
      , percentAdd(1.0f)
		, counter(0)
		, progressCallback(callback)
	{
		assert(progressCallback);

		scale(totalSteps, totalPercentage);
	}

	//! Scales inner parameters so that 'totalSteps' calls of the 'oneStep' method correspond to 'totalPercentage' percents
	void scale(unsigned totalSteps, unsigned totalPercentage=100, bool updateCurrentProgress=false)
	{
		if (totalSteps*totalPercentage==0)
		{
			step = 1;
			percentAdd = 0.0f;
			return;
		}

        if (totalSteps >= 2*totalPercentage)
		{
            step = static_cast<unsigned>(ceil(static_cast<float>(totalSteps) / static_cast<float>(totalPercentage)));
			assert(step!=0 && step<totalSteps);
            percentAdd = static_cast<float>(totalPercentage) / static_cast<float>(totalSteps/step);
		}
        else
		{
			step = 1;
            percentAdd = static_cast<float>(totalPercentage) / static_cast<float>(totalSteps);
		}

		if (updateCurrentProgress)
		{
			percent = (float)counter*(float)totalPercentage/(float)totalSteps;
			progressCallback->update(percent);
		}
		counter = 0;
	}

	//! Resets progress state
	void reset()
	{
		percent = 0.0;
		counter = 0;
		progressCallback->update(0.0);
	}

	//! Increments total progress value
	inline bool oneStep()
	{
		if (((++counter) % step)==0)
		{
			percent += percentAdd;
			progressCallback->update(percent);
		}

		return !progressCallback->isCancelRequested();
	}

protected:

	//! Total progress value (in percent)
	float percent;

	//! Number of necessary calls to 'oneStep' to actually call progress callback
    unsigned step;

	//! Percentage added to total progress value at each step
	float percentAdd;

	//! Current number of calls to 'oneStep'
	unsigned counter;

	//! associated GenericProgressCallback
	GenericProgressCallback* progressCallback;

};

}

#endif //GENERIC_PROGRESS_CALLBACK_HEADER
