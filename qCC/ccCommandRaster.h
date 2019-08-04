#ifndef COMMAND_LINE_RASTER_HEADER
#define COMMAND_LINE_RASTER_HEADER

#include "ccCommandLineInterface.h"

struct CommandRasterize : public ccCommandLineInterface::Command
{
	CommandRasterize();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandVolume25D : public ccCommandLineInterface::Command
{
	CommandVolume25D();

	bool process(ccCommandLineInterface& cmd) override;
};

#endif //COMMAND_LINE_RASTER_HEADER
