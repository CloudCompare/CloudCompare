//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef GENERIC_PROGRESS_CALLBACK_HEADER
#define GENERIC_PROGRESS_CALLBACK_HEADER

//Local
#include "CCConst.h"
#include "CCCoreLib.h"

class AtomicCounter;

namespace CCLib
{

//! A generic progress indicator interface to notify algorithms progress to the client application
class CC_CORE_LIB_API GenericProgressCallback
{
public:

	//! Default destructor
	virtual ~GenericProgressCallback() = default;

	//! Notifies the algorithm progress
	/** The notification is sent by the running algorithm (on the library side).
        This virtual method shouldn't be called too often, as the real process
        behind it is unspecified and may be time consuming. Ideally it shouldn't
        be called more than a few hundreds time.
		\param percent current progress, between 0.0 and 100.0
	**/
	virtual void update(float percent) = 0;

	//! Notifies the algorithm title
	/** The notification is sent by the ongoing algorithm (on the library side).
		\param methodTitle the algorithm title
	**/
	virtual void setMethodTitle(const char* methodTitle) = 0;

	//! Notifies some information about the ongoing process
	/** The notification is sent by the ongoing algorithm (on the library side).
		\param infoStr some textual information about the ongoing process
	**/
	virtual void setInfo(const char* infoStr) = 0;

	//! Notifies the fact that every information has been sent and that the process begins
	/** Once start() is called, the progress bar and other informations could be displayed (for example).
	**/
	virtual void start() = 0;

	//! Notifies the fact that the process has ended
	/** Once end() is called, the progress bar and other informations could be hidden (for example).
	**/
	virtual void stop() = 0;

	//! Checks if the process should be canceled
	/** This method is called by some process from time to time to know if it
		should halt before its normal ending. This is a way for the client application
		to cancel an ongoing process (but it won't work with all algorithms).
		Process results may be incomplete/void. The cancel requirement mechanism must
		be implemented (typically a simple "cancel()" method that will be called by the
		client application).
	**/
	virtual bool isCancelRequested() = 0;

	//! Returns whether the dialog title and info can be updated or not
	virtual bool textCanBeEdited() const { return true; }

};

//! Efficient management of progress based on a total number of steps different than 100
/** DGM: can now be associated to a null 'callback' pointer to simplify the client code.
**/
class CC_CORE_LIB_API NormalizedProgress
{
public:
	//! Default constructor
	/** \param callback associated GenericProgressCallback instance (can be null)
		\param totalSteps total number of steps (> 0)
		\param totalPercentage equivalent percentage (> 0)
	**/
	NormalizedProgress(GenericProgressCallback* callback, unsigned totalSteps, unsigned totalPercentage = 100);

	//! Destructor
	virtual ~NormalizedProgress();

	//! Scales inner parameters so that 'totalSteps' calls of the 'oneStep' method correspond to 'totalPercentage' percents
	void scale(unsigned totalSteps, unsigned totalPercentage = 100, bool updateCurrentProgress = false);

	//! Resets progress state
	void reset();

	//! Increments total progress value of a single unit
	bool oneStep();

	//! Increments total progress value of more than a single unit
	bool steps(unsigned n);

protected:

	//! Total progress value (in percent)
	float m_percent;

	//! Number of necessary calls to 'oneStep' to actually call progress callback
    unsigned m_step;

	//! Percentage added to total progress value at each step
	float m_percentAdd;

	//! Current number of calls to 'oneStep'
	/** Thread safe if CC_CORE_LIB is compiled with Qt.
	**/
	AtomicCounter* m_counter;

	//! associated GenericProgressCallback
	GenericProgressCallback* progressCallback;
};

}

#endif //GENERIC_PROGRESS_CALLBACK_HEADER
