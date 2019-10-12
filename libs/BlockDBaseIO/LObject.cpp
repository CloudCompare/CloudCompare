
//#include "stdafx.h"
#include "LObject.h"

ClassTreeInit::ClassTreeInit( LObInfo* pChildClass, LObInfo* pBaseClass)
{
	pChildClass->pSiblingOb = pBaseClass->pChildOb;
	pBaseClass->pChildOb	= pChildClass;
}

CLObject* LObInfo::CreateLObject()
{
	if( !pfnCreateLObject) return NULL;

	return (*pfnCreateLObject)();
}

bool LObInfo::IsDerivedFrom(const struct tagLObInfo* pBaseClass) const
{
	if( !pBaseClass || !this ) return false;

	const LObInfo* pClassThis = this;
	while(pClassThis)
	{
		if(pClassThis == pBaseClass)
			return true;

		pClassThis = pClassThis->pBaseOb;
	}

	return false;
}

////////////////////////////////////////////////////////////////////CLObject
LObInfo CLObject::classCLObject =
{ "CLObject",  NULL, NULL,NULL,NULL,NULL };

CLObject::CLObject()
{
	m_pLOb = NULL;	
}

CLObject::~CLObject()
{
	if(m_pLOb) delete m_pLOb; m_pLOb=NULL;
}

LObInfo* CLObject::GetLObInfo() const
{
	return GET_LOBINFO(CLObject);
}

bool CLObject::IsKindOf(const LObInfo* pBaseObj) const
{
	if(!pBaseObj) return false;

	LObInfo* pObjThis = GetLObInfo();
	return  pObjThis->IsDerivedFrom(pBaseObj);
}

IMPLEMENT_LOBINFO_DYNAMIC(CLFile,CLObject)

CLFile::CLFile()
{
	memset(m_strFilePath,0,MAX_PATHNAME);	memset(m_strErrMsg,0,1024);	m_bShowErrMsg = true;
}

CLFile::~CLFile()
{

}

int CLFile::Load4File(LPCSTR lpstrPathName)
{
	strcpy(m_strFilePath,lpstrPathName);
	return ERR_NONE;
}


IMPLEMENT_DYNARCH(CArchFile,CLFile)

CArchFile::CArchFile()
{

}

CArchFile::~CArchFile()
{

}

int CArchFile::Load4File(LPCSTR lpstrPathName)
{
	return CLFile::Load4File(lpstrPathName);
}



