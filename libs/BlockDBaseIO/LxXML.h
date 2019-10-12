////LxXML.h
/********************************************************************
	LxXML
	created:	2013/06/27
	author:		LX 
	purpose:	This file is for LxXML function
*********************************************************************/
#if !defined LxXML_h__LX_2013_6_27
#define LxXML_h__LX_2013_6_27

#include "Markup.h"
//#include "PrjDef.h"
#include "LObject.h"
typedef		CMarkup		CXml;

#define INTO_NODE(xml)	xml.IntoElem();
#define OUT_NODE(xml)	xml.OutOfElem();

#define NOT_GET			0x00
#define FORWARD_GET		0x01
#define BACKWARD_GET	0x02

#define MIN_DEMGSD		0.001
#define MIN_DOMGSD		0.001 

enum PathMode{
	Mode_Absolute,
	Mode_Relative
};
	

class CXmlFile	:	public	CArchFile
{
	DECLARE_DYNARCH(CXmlFile)
public:
	CXmlFile();
	virtual ~CXmlFile();
	bool	Open(LPCSTR	lpstrPathName);
	void	Close();
	bool	FindNode(PathMode mode,char* szText,const int fmt,...);
	int		FindSibingNode(LPCSTR lpstrName,char* strText=NULL,bool bNextNode=true );
	int		FindChildNode(LPCSTR lpstrName,char* strText=NULL,bool bNextNode=true );
	int		FindSiblingAttrib( LPCSTR lpstrName, LPCSTR lpstrAttrib, char* strText ,bool bNextNode=true );
	void	IntoNode() { INTO_NODE(m_xml) };
	void	OutNode() { OUT_NODE(m_xml) };
	void	GetData(char* strText)  {  strcpy(strText,m_xml.GetData().c_str()); }
	const char*	GetXmlError() const { return m_xml.GetError().c_str();	}

	bool	RemoveNode(LPCSTR lpstrName=NULL) { if(!lpstrName) return m_xml.RemoveElem();	if( FindSibingNode(lpstrName) == NOT_GET )	return true; return m_xml.RemoveElem(); }
	bool	RemoveChildNode(LPCSTR lpstrName=NULL) { if(!lpstrName) return m_xml.RemoveChildElem();	if( FindChildNode(lpstrName) == NOT_GET )	return true; return m_xml.RemoveChildElem(); }
	bool	CompleteNode(LPCSTR lpstrName,LPCSTR lpstrContent=NULL,bool bInsert=true ) { 
		if( FindSibingNode(lpstrName)==NOT_GET ) {
			if(bInsert)	return InsertNode(lpstrName,lpstrContent);
			else AddNode(lpstrName,lpstrContent);
		}
		if(lpstrContent) return SetData(lpstrContent);	 return true;
	}
	bool	CompleteNode(LPCSTR lpstrName,int nVal ) { 
		char strVal[256];	sprintf(strVal,"%d",nVal);
		return CompleteNode(lpstrName,strVal);
	}
	bool	CompleteNode(LPCSTR lpstrName,double fVal ) { 
		char strVal[256];	sprintf(strVal,"%lf",fVal);
		return CompleteNode(lpstrName,strVal);
	}

	bool	AddSubDoc(LPCSTR lpstrName) { return m_xml.AddSubDoc(lpstrName);	}
	bool	AddNode(LPCSTR lpstrName) { return m_xml.AddElem(lpstrName);	} 
	bool	AddNode(LPCSTR lpstrName,const char *fmt,...){
		char strMsg[2048]; va_list ap; va_start( ap, fmt ); vsprintf( strMsg,fmt,ap ); va_end(ap);
		return m_xml.AddElem(lpstrName,strMsg);
	}
	bool	AddChildNode(LPCSTR lpstrName,LPCSTR lpstrContent=NULL ) { return m_xml.AddChildElem(lpstrName,lpstrContent);	};
	bool	AddNode(LPCSTR lpstrName,int nVal ) { return m_xml.AddElem(lpstrName,nVal);	} 
	bool	AddChildNode(LPCSTR lpstrName,int nVal ) { return m_xml.AddChildElem(lpstrName,nVal);	};
//	bool	InsertNode(LPCSTR lpstrName,LPCSTR lpstrContent=NULL ) { return m_xml.InsertElem(lpstrName,lpstrContent);	} 
	bool	InsertNode(LPCSTR lpstrName ) { return m_xml.InsertElem(lpstrName);	} 
	bool	InsertChildNode(LPCSTR lpstrName,LPCSTR lpstrContent=NULL ) { return m_xml.InsertChildElem(lpstrName,lpstrContent);	};
	bool	InsertNode(LPCSTR lpstrName,int nVal ) { return m_xml.InsertElem(lpstrName,nVal);	} 
	bool	InsertNode(LPCSTR lpstrName,const char *fmt,...){
		char strMsg[2048]; va_list ap; va_start( ap, fmt ); vsprintf( strMsg,fmt,ap ); va_end(ap);
		return m_xml.InsertElem(lpstrName,strMsg);
	}
	bool	InsertChildNode(LPCSTR lpstrName,int nVal ) { return m_xml.InsertChildElem(lpstrName,nVal);	};

	bool	SetDoc(LPCSTR lpstrName) { return m_xml.SetDoc(lpstrName);	}
	LPCSTR	GetNodeContent() const	{ 
		static MCD_STR str; 
		str = m_xml.GetElemContent();
		return str.c_str(); 
	};
	bool	Save(LPCSTR lpstrPathName) { return m_xml.Save(lpstrPathName);	};
	void	GetAttrib(LPCSTR lpstrName,char* strText) { if(strText) strcpy(strText,m_xml.GetAttrib(lpstrName).c_str() );	}
	bool	SetAttrib(LPCSTR lpstrName,char* strText) { return m_xml.SetAttrib(lpstrName,strText);	}
	bool	SetChildData(LPCSTR lpstrName) { return m_xml.SetChildData(lpstrName); }
	bool	SetData(LPCSTR lpstrName) { return m_xml.SetData(lpstrName);	}
	bool	SetData(int nVal) { return m_xml.SetData(nVal);	}


	void operator=( const CXmlFile& object ) {	m_xml = object.m_xml;	*((CArchFile*)this)= *((CArchFile*)&object);	}
protected:
	CXml	m_xml;
		
};

#define MAX_LINK_PATH	256
/*
class CPrjXml	:	public	CXmlFile
{
	DECLARE_DYNARCH(CPrjXml);
public:
	CPrjXml();
	virtual	~CPrjXml();
	void	Reset();
	bool	Open(LPCSTR lpstrPathName);
	virtual	int		Save(LPCSTR lpstrPath);
	LPCSTR GetVersion() const { return m_hdr.vesion;	}
	const  char*	GetGcdPath()	const { return m_prodInfo.strGcdPath;	}

	void	UpdataTskList() { m_tskTypeList = m_tskTypeCur;	}
	void	SetTskList(int tskType);	// TSK_END represent all
	void	SetCurTsk(TASK tskType)	{ m_tskTypeCur = tskType;	}
	void	AddTsk2List(TASK tskType) { if(tskType<TSK_ATMCH || tskType>=TSK_END) return;	if(!(m_tskTypeList&tskType)) m_tskTypeList = m_tskTypeList|tskType; }
	void	RmTsk4List(TASK tskType)	{if(tskType<TSK_ATMCH || tskType>=TSK_END) return;	if( (m_tskTypeList&tskType) ) m_tskTypeList = m_tskTypeList|tskType; }

	int CreateSymbolicLink4Import(int lengLimit = MAX_LINK_PATH );
	int ReleaseSymbolicLink4Import();

	void operator=( const CPrjXml& object ) {
		memcpy(&m_hdr,&(object.m_hdr),sizeof(PrjHdr));		memcpy(&(m_prodInfo),&(object.m_prodInfo),sizeof(ProdInfo));

		int num;	ImageInfo* pInfo; ImgExInfo* pInfoEx;	SceneInfo* psc;
		pInfo = ((class CArray_ImageInfo*)&(object.m_import) )->GetData(num);
		m_import.Reset(pInfo,num);	
		pInfoEx= ((class CArray_ImgExInfo*)&(object.m_importEx) )->GetData(num);
		m_importEx.Reset(pInfoEx,num);	

		pInfo = ((class CArray_ImageInfo*)&(object.m_export) )->GetData(num);
//		pInfo = object.m_export.GetData(num);
		m_export.Reset(pInfo,num);
		pInfoEx= ((class CArray_ImgExInfo*)&(object.m_exportEx) )->GetData(num);
		m_exportEx.Reset(pInfoEx,num);

		psc = ((class CArray_SceneInfo*)&(object.m_scList) )->GetData(num);
//		psc = object.m_scList.GetData(num);
		m_scList.Reset(psc,num);

		m_tskTypeCur = object.m_tskTypeCur;	m_tskTypeList = object.m_tskTypeList;	m_nBlockID = object.m_nBlockID;

		*((CXmlFile*)this)= *((CXmlFile*)&object);	
	}
public:
	virtual int	AnalysisHead()	{ ErrorPrint("No Code to Analysis Xml Head!");	return ERR_UNKNOW; };
	virtual int AnalysisImport() { ErrorPrint("No Code to Analysis Xml Import!");	return ERR_UNKNOW; };
	virtual int AnalysisExport() { ErrorPrint("No Code to Analysis Xml Export!");	return ERR_UNKNOW; };
	virtual int AnalysisExtraPar() { ErrorPrint("No Code to Analysis Xml Extra parameter!");	return ERR_UNKNOW; };

public:
	int			m_tskTypeCur;
	int			m_tskTypeList;
	PrjHdr		m_hdr;
	CArray_ImageInfo	m_import;
	CArray_ImgExInfo	m_importEx;
	CArray_ImageInfo	m_export;
	CArray_ImgExInfo	m_exportEx;
	ProdInfo	m_prodInfo;
	CArray_SceneInfo	m_scList;
	int			m_nBlockID;		

	char		m_strProductSpace[512];
	char		m_strProcessSpace[512];
	MessagePort	m_msgPort;
};
*/


#endif // LxXML_h__LX_2013_6_27