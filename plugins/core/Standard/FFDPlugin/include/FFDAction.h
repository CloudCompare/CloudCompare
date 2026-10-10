//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: FFDPlugin                       #
//#           Free Form Deformation - Non-rigid Transformation             #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//##########################################################################

#ifndef FFD_ACTION_HEADER
#define FFD_ACTION_HEADER

class ccMainAppInterface;

//! Free Form Deformation Action Handler
namespace FFDAction
{
	//! Perform the FFD deformation action
	/*!
	 * Called when the user clicks the FFD deformation action.
	 * \param appInterface the main application interface
	 */
	void performDeformation( ccMainAppInterface *appInterface );
}

#endif // FFD_ACTION_HEADER
