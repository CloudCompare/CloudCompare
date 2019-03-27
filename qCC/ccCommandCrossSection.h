#ifndef COMMAND_CROSS_SECTION_HEADER
#define COMMAND_CROSS_SECTION_HEADER

#include "ccCommandLineInterface.h"

class QString;
class QXmlStreamAttributes;

struct CommandCrossSection : public ccCommandLineInterface::Command
{
	CommandCrossSection();

	bool process(ccCommandLineInterface& cmd) override;
	
private:
	bool readVector(const QXmlStreamAttributes& attributes, CCVector3& P, QString element, const ccCommandLineInterface& cmd);
};

#endif //COMMAND_CROSS_SECTION_HEADER
