#include <xiot/X3DWriter.h>

#include <xiot/X3DTypes.h>
#include <ctime>

using namespace XIOT;


//-----------------------------------------------------------------------------
void X3DWriter::startX3DDocument(X3DProfile profile, X3DVersion version, const std::multimap<std::string, std::string>* const meta, bool writeDefault)
{
  startDocument();
  startNode(ID::X3D);
  setSFString(ID::profile, X3DTypes::getProfileString(profile));
  setSFString(ID::version, X3DTypes::getVersionString(version));
  if (meta || writeDefault)
    {
    startNode(ID::head);
    if (meta)
      {
      for(std::multimap<std::string, std::string>::const_iterator I = meta->begin(); I != meta->end(); I++)
        {
        startNode(ID::meta);
        setSFString(ID::name, I->first);
        setSFString(ID::content, I->second);
        endNode(); // meta
        }
      }
    if (writeDefault)
      {
        startNode(ID::meta);
        setSFString(ID::name, "generator");
        setSFString(ID::content, "XIOT library (0.9.0), http://forge.collaviz.org/community/xiot");
        endNode(); // meta

        time_t rawtime;
        time ( &rawtime );
        struct tm *timeinfo = localtime ( &rawtime );
        char buffer [80];
        strftime (buffer,80,"%Y-%m-%d",timeinfo);

        startNode(ID::meta);
        setSFString(ID::name, "created");
        setSFString(ID::content, buffer);
        endNode(); // meta
      }

    endNode(); // head
    }
  startNode(ID::Scene);
}

void X3DWriter::endX3DDocument()
{
  endNode(); // Scene
  endNode(); // X3D
  endDocument();
}
