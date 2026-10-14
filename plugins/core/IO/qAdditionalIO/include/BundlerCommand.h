#pragma once

// ##########################################################################
// #                                                                        #
// #                      CLOUDCOMPARE PLUGIN                               #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 of the License.               #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: CloudCompare project                               #
// #                                                                        #
// ##########################################################################

// CCPluginAPI
#include <ccCommandLineInterface.h>

class BundlerCommand : public ccCommandLineInterface::Command
{
  public:
	BundlerCommand();
	~BundlerCommand() override = default;

	bool process(ccCommandLineInterface& cmd) override;
};
