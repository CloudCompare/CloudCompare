//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                 COPYRIGHT: Daniel Girardeau-Montaut                    #
//#                                                                        #
//##########################################################################

#ifndef CC_OPTIONS_HEADER
#define CC_OPTIONS_HEADER

//Qt
#include <QString>

//! Main application options
class ccOptions
{
public: //parameters

	//! Whether to display the normals by default or not
	bool normalsDisplayedByDefault;

	//! Use native load/save dialogs
	bool useNativeDialogs;

public: //methods

	//! Default constructor
	ccOptions();

	//! Resets parameters to default values
	void reset();

	//! Loads from persistent DB
	void fromPersistentSettings();

	//! Saves to persistent DB
	void toPersistentSettings() const;

public: //static methods

	//! Returns the stored values of each parameter.
	static const ccOptions& Instance() { return InstanceNonConst(); }

	//! Release unique instance (if any)
	static void ReleaseInstance();

	//! Sets parameters
	static void Set(const ccOptions& options);

protected: //methods
	
   //! Returns the stored values of each parameter.
	static ccOptions& InstanceNonConst();
};

#endif //CC_OPTIONS_HEADER
