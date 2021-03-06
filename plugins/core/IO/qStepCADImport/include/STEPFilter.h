#pragma once

//##########################################################################
//#                                                                        #
//#                 CLOUDCOMPARE PLUGIN: qSTEPCADImport                    #
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
//#                          COPYRIGHT: EDF R&D                            #
//#                                                                        #
//##########################################################################

#include <FileIOFilter.h>

class STEPFilter : public FileIOFilter
{
public:
	STEPFilter();
	
	// inherited from FileIOFilter
	CC_FILE_ERROR loadFile( const QString& fullFilename, ccHObject& container, LoadParameters& parameters ) override;

	//! Specific loading method
	CC_FILE_ERROR importStepFile(	ccHObject& container,
									const QString& fullFilename,
									double linearDeflection,
									LoadParameters& parameters);

	//! Sets the default linear deflection
	/** \param value linear deflection (in [1e-2, 1e-6])
			The smaller this value is, the smaller the triangles will be.
			In some cases, if this linear deflection is too big, the tesselation may crash
			(precisely the instruction BRep_Tool::Triangulation(face, location)).
			But we don't know how to anticipate this.
	**/
	static void SetDefaultLinearDeflection(double value);
};
