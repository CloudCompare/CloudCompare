#include <xiot/X3DWriterXML.h>

#include <cstring>

#include <xiot/X3DTypes.h>

using namespace XIOT;

X3DWriterXML::~X3DWriterXML()
{
  delete this->InfoStack;
}
 
//-----------------------------------------------------------------------------
X3DWriterXML::X3DWriterXML()
{
  this->InfoStack = new std::vector<XMLInfo>;
  this->Depth = 0;
  this->ActTab = "";
  this->type = X3DXML;
  this->OutputStream = NULL;
  X3DTypes::initMaps();
}

bool X3DWriterXML::setProperty(const char* const , void* )
{
	return false;
}

void* X3DWriterXML::getProperty(const char* const ) const
{
	return 0;
}

int X3DWriterXML::openFile(const char* file)
{
  this->closeFile();
  this->OutputStream = fopen(file, "w");
  
  return ((this->OutputStream == NULL) ? 0 : 1);
}

//----------------------------------------------------------------------------
void X3DWriterXML::closeFile()
{
  if (this->OutputStream)
    {
    fclose(OutputStream);
    }
}

//-----------------------------------------------------------------------------
void X3DWriterXML::startDocument()
{
  this->Depth = 0;
  fprintf(this->OutputStream, "<?xml version=\"1.0\" encoding =\"UTF-8\"?>\n\n");
}

//-----------------------------------------------------------------------------
void X3DWriterXML::endDocument()
{
  assert(this->Depth == 0);
  
}

//-----------------------------------------------------------------------------
void X3DWriterXML::startNode(int elementID)
{
  // End last tag, if this is the first child
  if (!this->InfoStack->empty())
    {
    if (!this->InfoStack->back().endTagWritten)
      {
		  fprintf(this->OutputStream, ">\n");
      this->InfoStack->back().endTagWritten = true;
      }
    }

  this->InfoStack->push_back(XMLInfo(elementID));
   
  fprintf(this->OutputStream, "%s<%s", this->ActTab.c_str(), X3DTypes::getElementByID(elementID));
  this->addDepth();
}

//-----------------------------------------------------------------------------
void X3DWriterXML::endNode()
{
  assert(!this->InfoStack->empty());
  this->subDepth();
  int elementID = this->InfoStack->back().elementId;

  // There were no childs
  if (!this->InfoStack->back().endTagWritten)
    {
	fprintf(this->OutputStream, "/>\n");
    }
  else
    {
	fprintf(this->OutputStream, "%s</%s>\n", this->ActTab.c_str(), X3DTypes::getElementByID(elementID));
    }

  this->InfoStack->pop_back();
}

//-----------------------------------------------------------------------------
void X3DWriterXML::setSFFloat(int attributeID, float fValue)
{
  fprintf(this->OutputStream, " %s=\"%g\"", X3DTypes::getAttributeByID(attributeID), fValue);
}

//-----------------------------------------------------------------------------
void X3DWriterXML::setSFInt32(int attributeID, int iValue)
{
  fprintf(this->OutputStream, " %s=\"%i\"", X3DTypes::getAttributeByID(attributeID), iValue);
}

//-----------------------------------------------------------------------------
void X3DWriterXML::setSFBool(int attributeID, bool bValue)
{
	fprintf(this->OutputStream, " %s=\"%s\"", X3DTypes::getAttributeByID(attributeID), (bValue ? "true" : "false"));
}

//-----------------------------------------------------------------------------
void X3DWriterXML::setSFVec3f(int attributeID, float x, float y, float z)
{
  fprintf(this->OutputStream, " %s=\"%g %g %g\"", X3DTypes::getAttributeByID(attributeID), x, y, z);

}

//-----------------------------------------------------------------------------
void X3DWriterXML::setSFVec2f(int attributeID, float s, float t)
{
  fprintf(this->OutputStream, " %s=\"%g %g\"", X3DTypes::getAttributeByID(attributeID), s, t);
}

//-----------------------------------------------------------------------------
void X3DWriterXML::setSFImage(int attributeID, const std::vector<int>& values)
{
  fprintf(this->OutputStream, " %s=\"\n%s", X3DTypes::getAttributeByID(attributeID), this->ActTab.c_str());

  unsigned int i = 0;
  
  assert(values.size() > 2);
  char buffer[20];
  //this->OutputStream << values[0] << " "; // width
  //this->OutputStream << values[1] << " "; // height
  //int bpp = values[2]; this->OutputStream << bpp << "\n"; // bpp
  fprintf(this->OutputStream, "%i %i %i\n", values[0], values[1], values[2]);
    
  i = 3;
  unsigned int j = 0;

  while (i < values.size())
  {
    sprintf(buffer,"0x%.8x",values[i]);
	fwrite(buffer, 1, strlen(buffer), this->OutputStream);
      
    if (j%(8*values[2]))
    {
      fprintf(this->OutputStream, " ");
    }
    else
    {
      fprintf(this->OutputStream, "\n");
    }
    i++; j+=values[2];
  }
  
  fprintf(this->OutputStream, "\"");
}

//-----------------------------------------------------------------------------
void X3DWriterXML::setSFColor(int attributeID, float r, float g, float b)
{
  this->setSFVec3f(attributeID, r, g, b);
}

//-----------------------------------------------------------------------------
// wieso -angle?
void X3DWriterXML::setSFRotation(int attributeID, float x, float y, float z, float angle)
{
  fprintf(this->OutputStream, " %s=\"%g %g %g %g\"", X3DTypes::getAttributeByID(attributeID), x, y, z, angle);
}

//-----------------------------------------------------------------------------
void X3DWriterXML::setSFString(int attributeID, const std::string &s)
{
  fprintf(this->OutputStream, " %s=\"%s\"", X3DTypes::getAttributeByID(attributeID), s.c_str());
}

//-----------------------------------------------------------------------------
void X3DWriterXML::setMFFloat(int attributeID, const std::vector<float>& values)
{
  fprintf(this->OutputStream, " %s=\"\n%s", X3DTypes::getAttributeByID(attributeID), this->ActTab.c_str());
  
  unsigned int i = 0;
  while (i < values.size())
    {
	fprintf(this->OutputStream, "%g", values[i]);
    if ((i+1)%3)
      {
      fprintf(this->OutputStream, " ");
      }
    else
      {
	  fprintf(this->OutputStream, ",\n%s", this->ActTab.c_str());
      }
    i++;
    }
  fprintf(this->OutputStream, "\"");
}

//-----------------------------------------------------------------------------
void X3DWriterXML::setMFRotation(int attributeID, const std::vector<float>& values)
{
  fprintf(this->OutputStream, " %s=\"\n%s", X3DTypes::getAttributeByID(attributeID), this->ActTab.c_str());

  unsigned int i = 0;
  while (i < values.size())
    {
    fprintf(this->OutputStream, "%g", values[i]);
    if ((i+1)%4)
      {
      fprintf(this->OutputStream, " ");
      }
    else
      {
	  fprintf(this->OutputStream, ",\n%s", this->ActTab.c_str());
      }
    i++;
    }
  fprintf(this->OutputStream, "\"");

}

//-----------------------------------------------------------------------------
void X3DWriterXML::setMFColor(int attributeID, const std::vector<float>& values)
{
  this->setMFFloat(attributeID, values);
}


//-----------------------------------------------------------------------------
void X3DWriterXML::setMFInt32(int attributeID, const std::vector<int>& values)
{
  fprintf(this->OutputStream, " %s=\"\n%s", X3DTypes::getAttributeByID(attributeID), this->ActTab.c_str());
  
  unsigned int i = 0;
  while (i < values.size())
    {
	fprintf(this->OutputStream, "%i ", values[i]);
    if (values[i] == -1) 
      {
	  fprintf(this->OutputStream, "\n%s", this->ActTab.c_str());
      }
    i++;
    }
  fprintf(this->OutputStream, "\"");
}

// Not implemented for FI encoding yet
/*void X3DWriterXML::setMFBool(int attributeID, std::vector<bool>& values)
{
  fprintf(this->OutputStream, " %s=\"", X3DTypes::getAttributeByID(attributeID));
  for(unsigned int i = 0; i < values.size(); i++)
  {
	if (i != 0)
		fprintf(this->OutputStream, " ");
    if (values[i])
		fprintf(this->OutputStream, "true");
	else
		fprintf(this->OutputStream, "false");
  }
  fprintf(this->OutputStream, "\"");
}*/

//-----------------------------------------------------------------------------
void X3DWriterXML::setMFVec2f(int attributeID, const std::vector<float>& values)
{
  fprintf(this->OutputStream, " %s=\"\n", X3DTypes::getAttributeByID(attributeID));
  
  assert((values.size() % 2) == 0);

  for(unsigned int i = 0; i < values.size(); i+=2)
  {
    fprintf(this->OutputStream, "%s%g %g,\n", this->ActTab.c_str(), values[i], values[i+1]);
  }

  fprintf(this->OutputStream, "%s\"", this->ActTab.c_str());
}

//-----------------------------------------------------------------------------
void X3DWriterXML::setMFVec3f(int attributeID, const std::vector<float>& values)
{
  fprintf(this->OutputStream, " %s=\"\n", X3DTypes::getAttributeByID(attributeID));
  
  assert((values.size() % 3) == 0);

  for(unsigned int i = 0; i < values.size(); i+=3)
  {
	fprintf(this->OutputStream, "%s%g %g %g,\n", this->ActTab.c_str(), values[i], values[i+1], values[i+2]);
  }

  fprintf(this->OutputStream, "%s\"", this->ActTab.c_str());
}

//-----------------------------------------------------------------------------
void X3DWriterXML::setMFString(int attributeID, const std::vector<std::string>& strings)
{
  fprintf(this->OutputStream, " %s='", X3DTypes::getAttributeByID(attributeID));

  for(unsigned int i = 0; i < strings.size(); i++)
  {
	fprintf(this->OutputStream, "\"%s\"", strings[i].c_str());
	if(i < (strings.size() - 1))
		fprintf(this->OutputStream, " ");
  }
  
  fprintf(this->OutputStream, "'");
}

//-----------------------------------------------------------------------------
void X3DWriterXML::flush()
{
  fflush(this->OutputStream);
}

//-----------------------------------------------------------------------------
void X3DWriterXML::addDepth()
{
  this->ActTab += "  ";
}

//-----------------------------------------------------------------------------
void X3DWriterXML::subDepth()
{
  this->ActTab.erase(0, 2);
}

void X3DWriterXML::printAttributeString(int )
{
  //this->OutputStream << " " << X3DTypes::getAttributeByID(attributeID) << "=\"" << this->GetNewline() << this->ActTab;
  //fprintf(this->OutputStream, " %s=\"\n%s", this->ActTab);
}
