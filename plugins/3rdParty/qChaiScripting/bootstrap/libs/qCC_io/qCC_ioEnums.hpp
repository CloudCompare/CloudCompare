#ifndef CHAISCRIPTING_BOOTSTRAP_QCC_IO_ENUMS_HPP
#define CHAISCRIPTING_BOOTSTRAP_QCC_IO_ENUMS_HPP

//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: ChaiScripting                      #
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
//#                     COPYRIGHT: Chris S Brown                           #
//#                                                                        #
//##########################################################################


#include <chaiscript/chaiscript.hpp>
#include <chaiscript/utility/utility.hpp>


namespace chaiscript
{
	namespace cloudCompare
	{
		namespace libs
		{
			namespace qCC_io
			{
				ModulePtr bs_CC_FILE_ERROR(ModulePtr m = std::make_shared<Module>())
				{
					chaiscript::utility::add_class<CC_FILE_ERROR>(*m,
						"CC_FILE_ERROR",
						{
							{ CC_FILE_ERROR::CC_FERR_NO_ERROR, "CC_FERR_NO_ERROR" },
							{ CC_FILE_ERROR::CC_FERR_BAD_ARGUMENT, "CC_FERR_BAD_ARGUMENT" },
							{ CC_FILE_ERROR::CC_FERR_UNKNOWN_FILE, "CC_FERR_UNKNOWN_FILE" },
							{ CC_FILE_ERROR::CC_FERR_WRONG_FILE_TYPE, "CC_FERR_WRONG_FILE_TYPE" },
							{ CC_FILE_ERROR::CC_FERR_WRITING, "CC_FERR_WRITING" },
							{ CC_FILE_ERROR::CC_FERR_READING, "CC_FERR_READING" },
							{ CC_FILE_ERROR::CC_FERR_NO_SAVE, "CC_FERR_NO_SAVE" },
							{ CC_FILE_ERROR::CC_FERR_NO_LOAD, "CC_FERR_NO_LOAD" },
							{ CC_FILE_ERROR::CC_FERR_BAD_ENTITY_TYPE, "CC_FERR_BAD_ENTITY_TYPE" },
							{ CC_FILE_ERROR::CC_FERR_CANCELED_BY_USER, "CC_FERR_CANCELED_BY_USER" },
							{ CC_FILE_ERROR::CC_FERR_NOT_ENOUGH_MEMORY, "CC_FERR_NOT_ENOUGH_MEMORY" },
							{ CC_FILE_ERROR::CC_FERR_MALFORMED_FILE, "CC_FERR_MALFORMED_FILE" },
							{ CC_FILE_ERROR::CC_FERR_CONSOLE_ERROR, "CC_FERR_CONSOLE_ERROR" },
							{ CC_FILE_ERROR::CC_FERR_BROKEN_DEPENDENCY_ERROR, "CC_FERR_BROKEN_DEPENDENCY_ERROR" },
							{ CC_FILE_ERROR::CC_FERR_FILE_WAS_WRITTEN_BY_UNKNOWN_PLUGIN, "CC_FERR_FILE_WAS_WRITTEN_BY_UNKNOWN_PLUGIN" },
							{ CC_FILE_ERROR::CC_FERR_THIRD_PARTY_LIB_FAILURE, "CC_FERR_THIRD_PARTY_LIB_FAILURE" },
							{ CC_FILE_ERROR::CC_FERR_THIRD_PARTY_LIB_EXCEPTION, "CC_FERR_THIRD_PARTY_LIB_EXCEPTION" },
							{ CC_FILE_ERROR::CC_FERR_NOT_IMPLEMENTED, "CC_FERR_NOT_IMPLEMENTED" }
						}
					);

					return m;
				}



				ModulePtr bootstrap_enum(ModulePtr m = std::make_shared<Module>())
				{
					bs_CC_FILE_ERROR(m);
					return m;
				}
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_QCC_IO_ENUMS_HPP