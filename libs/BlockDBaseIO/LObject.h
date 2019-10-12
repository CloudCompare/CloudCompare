////CLObject.h
/********************************************************************
	CLObject
	created:	2013/10/01
	author:		LX 
	purpose:	This file is for CLObject function
*********************************************************************/
#if !defined CLObject_h__LX_2013_10_1
#define CLObject_h__LX_2013_10_1

#include <stdio.h>
#include "PrjLog.hpp"


#define		MAX_PATHNAME	512
#define		MAX_FILENAME	128

#define GET_ARRAY_LEN(array) ( sizeof(array) / sizeof(array[0]) )

// #define VERF_LINE(s) if ( s[strlen(s)-1]=='\n' ) s[strlen(s)-1]=0
// #define VERF_SLASH(s) if ( s[strlen(s)-1]=='\\' || s[strlen(s)-1]=='/' ) s[strlen(s)-1]=0


class CLObject;

typedef struct tagLObInfo
{
	LPCSTR						lpszName;
	CLObject*					(*pfnCreateLObject)();
	struct tagLObInfo*			pBaseOb;
	struct tagLObInfo*			pChildOb;
	struct tagLObInfo*			pSiblingOb;

	CLObject*					CreateLObject();
	bool						IsDerivedFrom(const struct tagLObInfo* pBaseClass) const;

	CLObject*					(*pfnArch)(LPCSTR);
}LObInfo;

struct ClassTreeInit
{ ClassTreeInit( LObInfo* pChildClass, LObInfo* pBaseClass); };


#define GET_LOBINFO(class_name)		((LObInfo*)(&class_name::class##class_name ))

#define _DECLARE_FILE_IDENTIFY(fun_name_identify)	\
	public:	\
	static CLObject* fun_name_identify(LPCSTR lpstrPath);	\

#define DECLARE_FILE_IDENTIFY()	\
	_DECLARE_FILE_IDENTIFY(Identify)	\

#define _IMPLEMENT_FILE_IDENTIFY(class_name,fun_name_identify)	\
	CLObject* class_name::fun_name_identify(LPCSTR lpstrPath) \
	{ 	LObInfo* pInfo = GET_LOBINFO(class_name)->pChildOb;	CLObject* pOb = NULL;	\
	while(pInfo){ pOb = pInfo->pfnArch(lpstrPath); if(  pOb != NULL ) { return pOb; } pInfo = pInfo->pSiblingOb; }	\
	 return NULL;}	\

#define DECLARE_FILE_LOAD(fun_name_load,fun_name_identify)	\
	_DECLARE_FILE_IDENTIFY(fun_name_identify)	\
public:	\
	virtual int	fun_name_load(LPCSTR lpstrPath);	\
	virtual	int	Load4File(LPCSTR lpstrPath);	\
	
#define IMPLEMENT_FILE_LOAD(class_name,base_class_name,fun_name_load,fun_name_identify)	\
	int	class_name::fun_name_load(LPCSTR lpstrPath) { 	\
	if(m_pLOb) delete m_pLOb; m_pLOb = NULL;	\
	m_pLOb = fun_name_identify(lpstrPath);	\
	if( m_pLOb ){	return ( (class_name*)m_pLOb )->Load4File(lpstrPath);	\
	}	\
	return ERR_FILE_TYPE; } 	\

//




#define DECLARE_LOBINFO_DYNAMIC(class_name)			\
	public:											\
	static	LObInfo	class##class_name ;				\
	virtual LObInfo*	GetLObInfo() const;			\


#define DECLARE_LOBINFO_DYNCREATE(class_name)			\
	DECLARE_LOBINFO_DYNAMIC(class_name)		\
	public:							\
	static	CLObject*	CreateLObject();		\



#define _DECLARE_DYNFILELOAD(class_name,fun_name_load,fun_name_identify)			\
	DECLARE_LOBINFO_DYNCREATE(class_name)	\
	DECLARE_FILE_LOAD(fun_name_load,fun_name_identify)	\

#define DECLARE_DYNFILELOAD(class_name)			\
	_DECLARE_DYNFILELOAD( class_name,load ,identify )	\

#define _DECLARE_DYNARCH(class_name,fun_name_load,fun_name_identify)	\
	_DECLARE_DYNFILELOAD(class_name,fun_name_load,fun_name_identify)	\
	
#define DECLARE_DYNARCH(class_name)	\
	_DECLARE_DYNFILELOAD(class_name,load,identify )	\




#define IMPLEMENT_LOBINFO(class_name, base_class_name,pfnNew,pfnArch) \
	LObInfo class_name::class##class_name = { \
				#class_name,pfnNew , GET_LOBINFO(base_class_name),NULL,GET_LOBINFO(base_class_name)->pChildOb, pfnArch }; \
	ClassTreeInit _init_##class_name(GET_LOBINFO(class_name),GET_LOBINFO(base_class_name)); \
	LObInfo* class_name::GetLObInfo() const \
			{ return GET_LOBINFO(class_name); } \

#define IMPLEMENT_LOBINFO_DYNAMIC(class_name, base_class_name) \
	IMPLEMENT_LOBINFO(class_name, base_class_name,NULL,NULL)\

#define _IMPLEMENT_LOBINFO_DYNCREATE(class_name, base_class_name,pfnArch) \
	IMPLEMENT_LOBINFO(class_name, base_class_name,class_name::CreateLObject,pfnArch)\
	CLObject*  class_name::CreateLObject() \
		{ return new class_name; }	\

#define IMPLEMENT_LOBINFO_DYNCREATE(class_name, base_class_name) \
	_IMPLEMENT_LOBINFO_DYNCREATE(class_name, base_class_name,NULL)\


//class_name##_  

#define _IMPLEMENT_DYNFILELOAD(class_name, base_class_name,fun_name_load,fun_name_identify)	\
	_IMPLEMENT_LOBINFO_DYNCREATE(class_name, base_class_name,class_name::fun_name_identify) \
	IMPLEMENT_FILE_LOAD(class_name,base_class_name,fun_name_load,fun_name_identify)	\

#define IMPLEMENT_DYNFILELOAD(class_name, base_class_name)	\
	_IMPLEMENT_DYNFILELOAD(class_name, base_class_name,load ,identify )	\

#define _IMPLEMENT_DYNARCH(class_name, base_class_name,fun_name_load,fun_name_identify)	\
		_IMPLEMENT_DYNFILELOAD(class_name, base_class_name,fun_name_load,fun_name_identify)	\
		_IMPLEMENT_FILE_IDENTIFY(class_name,fun_name_identify)	\


#define IMPLEMENT_DYNARCH(class_name, base_class_name)	\
	_IMPLEMENT_DYNARCH(class_name, base_class_name,load , identify )	\
	

class CLObject
{
public:
	CLObject(); 
	virtual ~CLObject();
public:
	static	LObInfo	classCLObject;
//	virtual	int		Run(int argc, char* argv[])=0;
public:
	CLObject* GetInterObject() const { return m_pLOb;	}
	void operator=( const CLObject& object ) { 	};	//m_pLOb = object.GetInterObject();
	bool IsKindOf(const LObInfo* pObjInfo) const;
	virtual LObInfo* GetLObInfo() const;
protected:
	CLObject*	m_pLOb;
};

class CLFile	: public CLObject
{
	DECLARE_LOBINFO_DYNAMIC(CLFile);
public:
	CLFile();
	virtual ~CLFile();
	char*		GetLastError()	{ return m_strErrMsg;	};
	void		SetShowErrorMsg(bool bMsg)	{	m_bShowErrMsg = bMsg; }
	void		ErrorPrint(const char *fmt, ... ){
		va_list ap; va_start( ap, fmt ); vsprintf( m_strErrMsg,fmt,ap ); va_end(ap);
		strcat(m_strErrMsg,"\n");	if(m_bShowErrMsg)	printf("%s",m_strErrMsg);
	}
	char*		GetFilePath()	{ return m_strFilePath;	};
	void operator=( const CLFile& object ) { strcpy(m_strFilePath,object.m_strFilePath); m_bShowErrMsg = object.m_bShowErrMsg;	*((CLObject*)this)= *((CLObject*)&object);	}
protected:
	virtual int Load4File(LPCSTR lpstrPathName);
protected:
	char		m_strFilePath[MAX_PATHNAME];
	char		m_strErrMsg[1024];
	char		m_bShowErrMsg;
};

class	CArchFile	:	public CLFile
{
	DECLARE_DYNARCH(CArchFile)
public:
	CArchFile();
	virtual ~CArchFile();
	void operator=( const CArchFile& object ) {	*((CLFile*)this)= *((CLFile*)&object);	}

};

////////////////////////////////////////////////////////////////////////////
inline CArchFile* LxLoadFile(LObInfo* pInfo,LPCSTR lpstrPath)
{
	CArchFile* pFile	= (CArchFile*)(pInfo->pfnArch(lpstrPath));

	if(!pFile) return NULL;

	int ret = pFile->Load4File(lpstrPath);	
//	if(ret == ERR_NONE  )  Load4File(lpstrPath);	
	
	if( ret!=ERR_NONE ) return NULL;
	return pFile;

}

// class CFileLoad
// {
// public:
// 	CFileLoad() { m_pLOb = NULL;};
// 	~CFileLoad() { if(m_pLOb) delete m_pLOb;	m_pLOb = NULL; }
// 	int	load(LPCSTR lpstrPath);
// protected:
// 	LObInfo* m_pLOb;
// };

#endif // CLObject_h__LX_2013_10_1