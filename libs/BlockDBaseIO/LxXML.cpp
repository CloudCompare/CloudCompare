#include <stdio.h>
#include <string.h>
#include "LxXML.h"
#include <stdarg.h>

///////////////////////////////////////////////////////////////////////////////////////////
//CXmlFile
IMPLEMENT_DYNARCH(CXmlFile,CArchFile)

CXmlFile::CXmlFile()
{

}

CXmlFile::~CXmlFile()
{

}

int CXmlFile::Load4File(LPCSTR lpstrPath)
{
	return CArchFile::Load4File(lpstrPath);
}

bool CXmlFile::Open(LPCSTR	lpstrPathName)
{
	Close();
	if( !IsExist(lpstrPathName) ) { return false; }//ErrorPrint("Error:%s is not Exist!",lpstrPathName);
	bool bXml = m_xml.Load(lpstrPathName);
	
	if(!bXml)	{	ErrorPrint("%s", GetXmlError() );	}
		
	return bXml;
}

void CXmlFile::Close()
{
	m_xml.ResetPos();
	m_xml.SetDoc(NULL);
}

int	CXmlFile::FindSibingNode(LPCSTR lpstrName,char* strText/* =NULL */,bool bNextNode/* =true */ )
{
	if(strText) *strText = 0;
	if(!m_xml.GetElemContent().c_str()) return NOT_GET;
	MCD_STR str;
	
	if( !bNextNode && !strcmp(lpstrName,m_xml.GetTagName().c_str()) ) {
		if(strText) { str = m_xml.GetData(); strcpy( strText, str.c_str()); }
		return FORWARD_GET;
	}

	if( m_xml.FindElem(lpstrName) ) 
	{
		if(strText) { str = m_xml.GetData(); strcpy( strText, str.c_str()); }
		return FORWARD_GET;
	}else
	{
		m_xml.ResetMainPos();//
		if( m_xml.FindElem(lpstrName) ) {
			if(strText) { str = m_xml.GetData(); strcpy( strText, str.c_str()); }
			return BACKWARD_GET;
		}else{
			return NOT_GET;
		}
	}
	return NOT_GET;

}

int CXmlFile::FindChildNode(LPCSTR lpstrName,char* strText/* =NULL */,bool bNextNode/* =true */ )
{
	if(strText) *strText = 0;
	if(!m_xml.GetElemContent().c_str()) return NOT_GET;
	
	MCD_STR str;
	if( !bNextNode && !strcmp(lpstrName,m_xml.GetChildTagName().c_str()) ) {
		if(strText) { str = m_xml.GetChildData(); strcpy( strText, str.c_str()); }
		return FORWARD_GET;
	}


	if( m_xml.FindChildElem(lpstrName) ) 
	{
		if(strText) { str = m_xml.GetChildData(); strcpy( strText, str.c_str()); }
		return FORWARD_GET;
	}else
	{
		m_xml.ResetChildPos();
		if( m_xml.FindChildElem(lpstrName) ) {
			if(strText) { str = m_xml.GetChildData(); strcpy( strText, str.c_str()); }
			return BACKWARD_GET;
		}else{
			return NOT_GET;
		}
	}
	return NOT_GET;
}

bool CXmlFile::FindNode(PathMode mode,char* szText,const int fmt,...){
	if(szText) *szText = 0;
	if(mode == Mode_Absolute) m_xml.ResetPos();

	int num = fmt,argno = 0;
	char* a;
	va_list ap;
	va_start(ap,fmt);
	
	a = va_arg(ap, char *);
	if( FindSibingNode(a,NULL,false) == NOT_GET ) return false;

	INTO_NODE(m_xml)	
	argno++;
	while (a != 0 && argno < num)
	{
		a = va_arg(ap, char *);
		if( !m_xml.FindElem(a) ) return false;
		INTO_NODE(m_xml)
		argno++;
	}
	OUT_NODE(m_xml)

	va_end(ap);
	
	if(szText) { strcpy( szText, m_xml.GetData().c_str()); }
	
	return true;
	
}

int CXmlFile::FindSiblingAttrib( LPCSTR lpstrName, LPCSTR lpstrAttrib, char* strText ,bool bNextNode/* =true */ )
{
	int ret = FindSibingNode(lpstrName,NULL,bNextNode);
	if(  ret == NOT_GET ) return NOT_GET;
	
	GetAttrib(lpstrAttrib,strText);
	
	return ret;
}



// IMPLEMENT_DYNARCH(CPrjXml,CXmlFile)
// 
// CPrjXml::CPrjXml()
// {
// 	memset(&m_hdr,0,sizeof(PrjHdr));
// 	memset(&m_prodInfo,0,sizeof(ProdInfo));
// 	m_nBlockID = 0;
// 	m_tskTypeList = m_tskTypeCur = 0;
// 
// 	memset(m_strProcessSpace,0,sizeof(m_strProcessSpace));
// 	memset(m_strProductSpace,0,sizeof(m_strProductSpace));
// 
// 	memset(&m_msgPort,0,sizeof(MessagePort));
// }
// 
// CPrjXml::~CPrjXml()
// {
// 	Reset();	
// }
// 
// void CPrjXml::Reset()
// {
// 	memset(&m_hdr,0,sizeof(PrjHdr));
// 	memset(&m_prodInfo,0,sizeof(ProdInfo));
// 	m_scList.Reset(NULL,0);
// 	m_import.Reset(NULL,0);	m_importEx.Reset(NULL,0);
// 	m_export.Reset(NULL,0);	m_exportEx.Reset(NULL,0);
// }
// 
// bool CPrjXml::Open(LPCSTR lpstrPathName)
// {
// 	Reset();
// 	if( !CXmlFile::Open(lpstrPathName) ) return false;
// 	return true;
// }
// 
// int CPrjXml::Load4File(LPCSTR lpstrPath)
// {
// 	char strF[512];	strcpy(strF,lpstrPath);	Dos2Unix(strF);
// 	*strrchr(strF,'/') = 0;
// 
// 	if( !m_strProcessSpace[0] ) strcpy(m_strProcessSpace,strF);
// 	if( !m_strProductSpace[0] ) strcpy(m_strProductSpace,strF);
// 
// 	return CXmlFile::Load4File(lpstrPath);
// }
// 
// void CPrjXml::SetTskList(int tskType)
// {
// 	if( !(tskType&TSK_END) ) m_tskTypeList = tskType;
// 	else{
// 		int nTmp=0;
// 		for (int i=1; i<TTID_END;i++){
// 			nTmp += 1;
// 			nTmp <<= 1;
// 		}
// 		m_tskTypeList = nTmp;
// 	}
// }
// 
// int CPrjXml::Save(LPCSTR lpstrPath)
// {
// 	LogPrint(0,"No Code to Save xml File!");
// 
// 	return ERR_NONE;
// }
// 
// int CPrjXml::CreateSymbolicLink4Import(int lengLimit /* = 256 */)
// {
// 	char strConf[512];	::GetPrivateProfilePath(strConf,CONFIG_FILE);
// 	char strPathCap[100];
// 	::GetPrivateProfileString("PathSetting","linuxpath","",strPathCap,100,strConf);
// 	if ( !IsDir(strPathCap) ) {
// 		::GetPrivateProfileString("PathSetting","windowspath","",strPathCap,100,strConf);
// 		if( !IsDir(strPathCap) ) strcpy(strPathCap,"");
// 	}
// 
// 	char strLink[512];	strcpy(strLink,strPathCap);		AddEndSlash(strLink);	
// 	
// 	srand(time(0));	
// 	int i,sz;
// 	ImageInfo* pInfo = m_import.GetData(sz);
// 	
// 	if (sz>0 && strlen(strLink)<1 && pInfo->path[1] == ':'){
// 		memcpy(strLink,pInfo->path,3*sizeof(char));
// 	}
// 	char* pLnk = strLink+strlen(strLink);
// 
// 	int nLinkNum = 0;
// 	for (i=0; i<sz; i++,pInfo++){
// 		if ( strlen(pInfo->path)<lengLimit ) continue;
// 		int mkey=rand();
// 		sprintf(pLnk,"%d",mkey);
// 		while(1) { if( IsExist(strLink) ) strcat(strLink,"a");else break; }
// 		char* pS = strrpath(pInfo->path);	char c = *pS;	*pS = 0;
// //		if( !::CreateLink(pInfo->path,strLink) ) continue;	*pS = c;
// 		::CreateLink(pInfo->path,strLink);	*pS = c;
// 		sprintf(pInfo->path,"%s%s",strLink,pS);
// 		nLinkNum++;
// 	}
// 
// 	return nLinkNum;
// 	
// }
// 
// int CPrjXml::ReleaseSymbolicLink4Import()
// {
// 	int sz,i;
// 	ImageInfo* pInfo = m_import.GetData(sz);
// 	char strImagePath[512];
// 
// 	int nLinkNum = 0;
// 	for (i=0; i<sz; i++,pInfo++){
// 		strcpy(strImagePath,pInfo->path);	char* pS = strrpath(strImagePath);
// 		if(!pS) continue;	char c = *pS;	*pS = 0;
// 		char strPath[512];	memset(strPath,0,512);
// 		if( readlink(strImagePath,strPath,512)<0 ) continue;
// 		RemoveLink(strImagePath);
// 		*pS = c;	sprintf(pInfo->path,"%s%s",strPath,pS);
// 		nLinkNum++;
// 	}
// 
// 	return nLinkNum;
// }

/////////////////////////////////////////////////////////////////
#define _NODEVAL_LENGTH	20

#define _INTSIZEOF(n)   ( (sizeof(n) + sizeof(int) - 1) & ~(sizeof(int) - 1) )




