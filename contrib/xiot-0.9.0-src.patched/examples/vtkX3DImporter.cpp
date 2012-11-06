#include "vtkX3DImporter.h"
#include "vtkObjectFactory.h"

#include <xiot/X3DLoader.h>
#include <xiot/X3DAttributes.h>
#include <xiot/X3DParseException.h>

#include "vtkX3DNodeHandler.h"


//-----------------------------------------------------------------------------
// vtkX3DImporter
//-----------------------------------------------------------------------------
vtkCxxRevisionMacro(vtkX3DImporter, "$Revision: 1.19 $");
vtkStandardNewMacro(vtkX3DImporter);

vtkX3DImporter::vtkX3DImporter()
{
	this->FileName = NULL;
  this->CalculateNormals = 1;
}

vtkX3DImporter::~vtkX3DImporter()
{
	if (this->FileName)
    {
    delete [] this->FileName;
    }
}

int vtkX3DImporter::ImportBegin()
{
	XIOT::X3DLoader loader;
	XIOT::X3DTypes::initMaps();
  vtkX3DNodeHandler handler(this->Renderer, this);
	loader.setNodeHandler(&handler);
	try {
		if (!loader.load(this->FileName))
			return 0;
	} 
	catch (XIOT::X3DParseException& e)
	{	
	  vtkErrorMacro( << "Error while parsing file " << this->FileName << ":" << e.getMessage().c_str()
					  << " (Line: " << e.getLineNumber() << ", Column: " << e.getColumnNumber() << ")");
      return 0;
	}
	return 1;
}

void vtkX3DImporter::ImportEnd ()
{
}

void vtkX3DImporter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
  os << indent << "File Name: " 
     << (this->FileName ? this->FileName : "(none)") << "\n";
  os << indent << "CalculateNormals: " 
     << this->CalculateNormals << "\n";
}

