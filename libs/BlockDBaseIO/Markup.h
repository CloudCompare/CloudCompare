// Markup.h: interface for the CMarkup class.
//
// Markup Release 11.5
// Copyright (C) 2011 First Objective Software, Inc. All rights reserved
// Go to www.firstobject.com for the latest CMarkup and EDOM documentation
// Use in commercial applications requires written permission
// This software is provided "as is", with no warranty.

#if !defined(_MARKUP_H_INCLUDED_)
#define _MARKUP_H_INCLUDED_

#include <stdlib.h>
#include <string.h> // memcpy, memset, strcmp...

// Major build options
// MARKUP_WCHAR wide char (2-byte UTF-16 on Windows, 4-byte UTF-32 on Linux and OS X)
// MARKUP_MBCS ANSI/double-byte strings on Windows
// MARKUP_STL (default except VC++) use STL strings instead of MFC strings
// MARKUP_SAFESTR to use string _s functions in VC++ 2005 (_MSC_VER >= 1400)
// MARKUP_WINCONV (default for VC++) for Windows API character conversion
// MARKUP_ICONV (default for GNU) for character conversion on Linux and OS X and other platforms
// MARKUP_STDCONV to use neither WINCONV or ICONV, falls back to setlocale based conversion for ANSI
//
#if ! defined(MARKUP_WINDOWS)
#if defined(_WIN32) || defined(WIN32)
#define MARKUP_WINDOWS
#endif // WIN32 or _WIN32
#endif // not MARKUP_WINDOWS
#if _MSC_VER > 1000 // VC++
#pragma once
#if ! defined(MARKUP_SAFESTR) // not VC++ safe strings
#pragma warning(disable:4996) // VC++ 2005 deprecated function warnings
#endif // not VC++ safe strings
#if defined(MARKUP_STL) && _MSC_VER < 1400 // STL pre VC++ 2005
#pragma warning(disable:4786) // std::string long names
#endif // VC++ 2005 STL
#else // not VC++
#if ! defined(MARKUP_STL)
#define MARKUP_STL
#endif // not STL
#if defined(__GNUC__) && ! defined(MARKUP_ICONV) && ! defined(MARKUP_STDCONV) && ! defined(MARKUP_WINCONV)
#if ! defined(MARKUP_WINDOWS)
#define MARKUP_ICONV
#endif // not Windows
#endif // GNUC and not ICONV not STDCONV not WINCONV
#endif // not VC++
#if (defined(_UNICODE) || defined(UNICODE)) && ! defined(MARKUP_WCHAR)
#define MARKUP_WCHAR
#endif // _UNICODE or UNICODE
#if (defined(_MBCS) || defined(MBCS)) && ! defined(MARKUP_MBCS)
#define MARKUP_MBCS
#endif // _MBCS and not MBCS
#if ! defined(MARKUP_SIZEOFWCHAR)
#if __SIZEOF_WCHAR_T__ == 4 || __WCHAR_MAX__ > 0x10000
#define MARKUP_SIZEOFWCHAR 4
#else // sizeof(wchar_t) != 4
#define MARKUP_SIZEOFWCHAR 2
#endif // sizeof(wchar_t) != 4
#endif // not MARKUP_SIZEOFWCHAR
#if ! defined(MARKUP_WINCONV) && ! defined(MARKUP_STDCONV) && ! defined(MARKUP_ICONV)
#define MARKUP_WINCONV
#endif // not WINCONV not STDCONV not ICONV
#if ! defined(MARKUP_FILEBLOCKSIZE)
#define MARKUP_FILEBLOCKSIZE 16384
#endif

// Text type and function defines (compiler and build-option dependent)
// 
#define MCD_ACP 0
#define MCD_UTF8 65001
#define MCD_UTF16 1200
#define MCD_UTF32 65005
#if defined(MARKUP_WCHAR)
#define MCD_CHAR wchar_t
#define MCD_PCSZ const wchar_t*
#define MCD_PSZLEN (int)wcslen
#define MCD_PSZCHR wcschr
#define MCD_PSZSTR wcsstr
#define MCD_PSZTOL wcstol
#if defined(MARKUP_SAFESTR) // VC++ safe strings
#define MCD_SSZ(sz) sz,(sizeof(sz)/sizeof(MCD_CHAR))
#define MCD_PSZCPY(sz,p) wcscpy_s(MCD_SSZ(sz),p)
#define MCD_PSZNCPY(sz,p,n) wcsncpy_s(MCD_SSZ(sz),p,n)
#define MCD_SPRINTF swprintf_s
#define MCD_FOPEN(f,n,m) {if(_wfopen_s(&f,n,m)!=0)f=NULL;}
#else // not VC++ safe strings
#if defined(__GNUC__) && ! defined(MARKUP_WINDOWS) // non-Windows GNUC
#define MCD_SSZ(sz) sz,(sizeof(sz)/sizeof(MCD_CHAR))
#else // not non-Windows GNUC
#define MCD_SSZ(sz) sz
#endif // not non-Windows GNUC
#define MCD_PSZCPY wcscpy
#define MCD_PSZNCPY wcsncpy
#define MCD_SPRINTF swprintf
#define MCD_FOPEN(f,n,m) f=_wfopen(n,m)
#endif // not VC++ safe strings
#define MCD_T(s) L ## s
#if MARKUP_SIZEOFWCHAR == 4 // sizeof(wchar_t) == 4
#define MCD_ENC MCD_T("UTF-32")
#else // sizeof(wchar_t) == 2
#define MCD_ENC MCD_T("UTF-16")
#endif
#define MCD_CLEN(p) 1
#else // not MARKUP_WCHAR
#define MCD_CHAR char
#define MCD_PCSZ const char*
#define MCD_PSZLEN (int)strlen
#define MCD_PSZCHR strchr
#define MCD_PSZSTR strstr
#define MCD_PSZTOL strtol
#if defined(MARKUP_SAFESTR) // VC++ safe strings
#define MCD_SSZ(sz) sz,(sizeof(sz)/sizeof(MCD_CHAR))
#define MCD_PSZCPY(sz,p) strcpy_s(MCD_SSZ(sz),p)
#define MCD_PSZNCPY(sz,p,n) strncpy_s(MCD_SSZ(sz),p,n)
#define MCD_SPRINTF sprintf_s
#define MCD_FOPEN(f,n,m) {if(fopen_s(&f,n,m)!=0)f=NULL;}
#else // not VC++ safe strings
#define MCD_SSZ(sz) sz
#define MCD_PSZCPY strcpy
#define MCD_PSZNCPY strncpy
#define MCD_SPRINTF sprintf
#define MCD_FOPEN(f,n,m) f=fopen(n,m)
#endif // not VC++ safe strings
#define MCD_T(s) s
#if defined(MARKUP_MBCS) // MBCS/double byte
#define MCD_ENC MCD_T("")
#if defined(MARKUP_WINCONV)
#define MCD_CLEN(p) (int)_mbclen((const unsigned char*)p)
#else // not WINCONV
#define MCD_CLEN(p) (int)mblen(p,MB_CUR_MAX)
#endif // not WINCONV
#else // not MBCS/double byte
#define MCD_ENC MCD_T("UTF-8")
#define MCD_CLEN(p) 1
#endif // not MBCS/double byte
#endif // not MARKUP_WCHAR
#if _MSC_VER < 1000 // not VC++
#define MCD_STRERROR strerror(errno)
#endif // not VC++

// String type and function defines (compiler and build-option dependent)
// Define MARKUP_STL to use STL strings
//
#if defined(MARKUP_STL) // STL
#include <string>
#if defined(MARKUP_WCHAR)
#define MCD_STR std::wstring
#else // not MARKUP_WCHAR
#define MCD_STR std::string
#endif // not MARKUP_WCHAR
#define MCD_2PCSZ(s) s.c_str()
#define MCD_STRLENGTH(s) (int)s.size()
#define MCD_STRCLEAR(s) s.erase()
#define MCD_STRCLEARSIZE(s) MCD_STR t; s.swap(t)
#define MCD_STRISEMPTY(s) s.empty()
#define MCD_STRMID(s,n,l) s.substr(n,l)
#define MCD_STRASSIGN(s,p,n) s.assign(p,n)
#define MCD_STRCAPACITY(s) (int)s.capacity()
#define MCD_STRINSERTREPLACE(d,i,r,s) d.replace(i,r,s)
#define MCD_GETBUFFER(s,n) new MCD_CHAR[n+1]; if ((int)s.capacity()<(int)n) s.reserve(n)
#define MCD_RELEASEBUFFER(s,p,n) s.replace(0,s.size(),p,n); delete[]p
#define MCD_BLDRESERVE(s,n) s.reserve(n)
#define MCD_BLDCHECK(s,n,d) ;
#define MCD_BLDRELEASE(s) ;
#define MCD_BLDAPPENDN(s,p,n) s.append(p,n)
#define MCD_BLDAPPEND(s,p) s.append(p)
#define MCD_BLDAPPEND1(s,c) s+=(MCD_CHAR)(c)
#define MCD_BLDLEN(s) (int)s.size()
#define MCD_BLDTRUNC(s,n) s.resize(n)
#else // not STL, i.e. MFC
// afx.h provides CString, to avoid "WINVER not defined" #include stdafh.x in Markup.cpp
#include <afx.h>
#define MCD_STR CString
#define MCD_2PCSZ(s) ((MCD_PCSZ)s)
#define MCD_STRLENGTH(s) s.GetLength()
#define MCD_STRCLEAR(s) s.Empty()
#define MCD_STRCLEARSIZE(s) s=MCD_STR()
#define MCD_STRISEMPTY(s) s.IsEmpty()
#define MCD_STRMID(s,n,l) s.Mid(n,l)
#define MCD_STRASSIGN(s,p,n) memcpy(s.GetBuffer(n),p,(n)*sizeof(MCD_CHAR));s.ReleaseBuffer(n);
#define MCD_STRCAPACITY(s) (((CStringData*)((MCD_PCSZ)s)-1)->nAllocLength)
#define MCD_GETBUFFER(s,n) s.GetBuffer(n)
#define MCD_RELEASEBUFFER(s,p,n) s.ReleaseBuffer(n)
#define MCD_BLDRESERVE(s,n) MCD_CHAR*pD=s.GetBuffer(n); int nL=0
#define MCD_BLDCHECK(s,n,d) if(nL+(int)(d)>n){s.ReleaseBuffer(nL);n<<=2;pD=s.GetBuffer(n);}
#define MCD_BLDRELEASE(s) s.ReleaseBuffer(nL)
#define MCD_BLDAPPENDN(s,p,n) MCD_PSZNCPY(&pD[nL],p,n);nL+=n
#define MCD_BLDAPPEND(s,p) MCD_PSZCPY(&pD[nL],p);nL+=MCD_PSZLEN(p)
#define MCD_BLDAPPEND1(s,c) pD[nL++]=(MCD_CHAR)(c)
#define MCD_BLDLEN(s) nL
#define MCD_BLDTRUNC(s,n) nL=n
#endif // not STL
#define MCD_STRTOINT(s) MCD_PSZTOL(MCD_2PCSZ(s),NULL,10)

// Allow function args to accept string objects as constant string pointers
struct MCD_CSTR
{
	MCD_CSTR() { pcsz=NULL; };
	MCD_CSTR( MCD_PCSZ p ) { pcsz=p; };
	MCD_CSTR( const MCD_STR& s ) { pcsz = MCD_2PCSZ(s); };
	operator MCD_PCSZ() const { return pcsz; };
	MCD_PCSZ pcsz;
};

// On Linux and OS X, filenames are not specified in wchar_t
#if defined(MARKUP_WCHAR) && defined(__GNUC__)
#undef MCD_FOPEN
#define MCD_FOPEN(f,n,m) f=fopen(n,m)
#define MCD_T_FILENAME(s) s
#define MCD_PCSZ_FILENAME const char*
struct MCD_CSTR_FILENAME
{
	MCD_CSTR_FILENAME() { pcsz=NULL; };
	MCD_CSTR_FILENAME( MCD_PCSZ_FILENAME p ) { pcsz=p; };
	MCD_CSTR_FILENAME( const std::string& s ) { pcsz = s.c_str(); };
	operator MCD_PCSZ_FILENAME() const { return pcsz; };
	MCD_PCSZ_FILENAME pcsz;
};
#else // not WCHAR GNUC
#define MCD_CSTR_FILENAME MCD_CSTR
#define MCD_T_FILENAME MCD_T
#define MCD_PCSZ_FILENAME MCD_PCSZ
#endif // not WCHAR GNUC

// File fseek, ftell and offset type
#if defined(__GNUC__) && ! defined(MARKUP_WINDOWS) // non-Windows GNUC
#define MCD_FSEEK fseeko
#define MCD_FTELL ftello
#define MCD_INTFILEOFFSET off_t
#elif _MSC_VER >= 1000 && defined(MARKUP_HUGEFILE) // VC++ HUGEFILE
#if _MSC_VER < 1400 // before VC++ 2005
extern "C" int __cdecl _fseeki64(FILE *, __int64, int);
extern "C" __int64 __cdecl _ftelli64(FILE *);
#endif // before VC++ 2005
#define MCD_FSEEK _fseeki64
#define MCD_FTELL _ftelli64
#define MCD_INTFILEOFFSET __int64
#else // not non-Windows GNUC or VC++ HUGEFILE
#define MCD_FSEEK fseek
#define MCD_FTELL ftell
#define MCD_INTFILEOFFSET long
#endif // not non-Windows GNUC or VC++ HUGEFILE

// End of line choices: none, return, newline, or CRLF
#if defined(MARKUP_EOL_NONE)
#define MCD_EOL MCD_T("")
#elif defined(MARKUP_EOL_RETURN) // rare; only used on some old operating systems
#define MCD_EOL MCD_T("\r")
#elif defined(MARKUP_EOL_NEWLINE) // Unix standard
#define MCD_EOL MCD_T("\n")
#elif defined(MARKUP_EOL_CRLF) || defined(MARKUP_WINDOWS) // Windows standard
#define MCD_EOL MCD_T("\r\n")
#else // not Windows and not otherwise specified
#define MCD_EOL MCD_T("\n")
#endif // not Windows and not otherwise specified
#define MCD_EOLLEN (sizeof(MCD_EOL)/sizeof(MCD_CHAR)-1) // string length of MCD_EOL

struct FilePos;
struct TokenPos;
struct NodePos;
struct PathPos;
struct SavedPosMapArray;
struct ElemPosTree;

class CMarkup
{
public:
	CMarkup() { x_InitMarkup(); SetDoc( NULL ); };
	CMarkup( MCD_CSTR szDoc ) { x_InitMarkup(); SetDoc( szDoc ); };
	CMarkup( int nFlags ) { x_InitMarkup(); SetDoc( NULL ); m_nDocFlags = nFlags; };
	CMarkup( const CMarkup& markup ) { x_InitMarkup(); *this = markup; };
	void operator=( const CMarkup& markup );
	~CMarkup();

	// Navigate
	bool Load( MCD_CSTR_FILENAME szFileName );
	bool SetDoc( MCD_PCSZ pDoc );
	bool SetDoc( const MCD_STR& strDoc );
	bool IsWellFormed();
	bool FindElem( MCD_CSTR szName=NULL );
	bool FindChildElem( MCD_CSTR szName=NULL );
	bool IntoElem();
	bool OutOfElem();
	void ResetChildPos() { x_SetPos(m_iPosParent,m_iPos,0); };
	void ResetMainPos() { x_SetPos(m_iPosParent,0,0); };
	void ResetPos() { x_SetPos(0,0,0); };
	MCD_STR GetTagName() const;
	MCD_STR GetChildTagName() const { return x_GetTagName(m_iPosChild); };
	MCD_STR GetData() { return x_GetData(m_iPos); };
	MCD_STR GetChildData() { return x_GetData(m_iPosChild); };
	MCD_STR GetElemContent() const { return x_GetElemContent(m_iPos); };
	MCD_STR GetAttrib( MCD_CSTR szAttrib ) const { return x_GetAttrib(m_iPos,szAttrib); };
	MCD_STR GetChildAttrib( MCD_CSTR szAttrib ) const { return x_GetAttrib(m_iPosChild,szAttrib); };
	bool GetNthAttrib( int n, MCD_STR& strAttrib, MCD_STR& strValue ) const;
	MCD_STR GetAttribName( int n ) const;
	int FindNode( int nType=0 );
	int GetNodeType() { return m_nNodeType; };
	bool SavePos( MCD_CSTR szPosName=MCD_T(""), int nMap = 0 );
	bool RestorePos( MCD_CSTR szPosName=MCD_T(""), int nMap = 0 );
	bool SetMapSize( int nSize, int nMap = 0 );
	MCD_STR GetError() const;
	const MCD_STR& GetResult() const { return m_strResult; };
	int GetDocFlags() const { return m_nDocFlags; };
	void SetDocFlags( int nFlags ) { m_nDocFlags = (nFlags & ~(MDF_READFILE|MDF_WRITEFILE|MDF_APPENDFILE)); };
	enum MarkupDocFlags
	{
		MDF_UTF16LEFILE = 1,
		MDF_UTF8PREAMBLE = 4,
		MDF_IGNORECASE = 8,
		MDF_READFILE = 16,
		MDF_WRITEFILE = 32,
		MDF_APPENDFILE = 64,
		MDF_UTF16BEFILE = 128,
		MDF_TRIMWHITESPACE = 256,
		MDF_COLLAPSEWHITESPACE = 512
	};
	enum MarkupNodeFlags
	{
		MNF_WITHCDATA      = 0x01,
		MNF_WITHNOLINES    = 0x02,
		MNF_WITHXHTMLSPACE = 0x04,
		MNF_WITHREFS       = 0x08,
		MNF_WITHNOEND      = 0x10,
		MNF_ESCAPEQUOTES  = 0x100,
		MNF_NONENDED   = 0x100000,
		MNF_ILLDATA    = 0x200000
	};
	enum MarkupNodeType
	{
		MNT_ELEMENT					= 1,    // 0x0001
		MNT_TEXT					= 2,    // 0x0002
		MNT_WHITESPACE				= 4,    // 0x0004
		MNT_TEXT_AND_WHITESPACE     = 6,    // 0x0006
		MNT_CDATA_SECTION			= 8,    // 0x0008
		MNT_PROCESSING_INSTRUCTION	= 16,   // 0x0010
		MNT_COMMENT					= 32,   // 0x0020
		MNT_DOCUMENT_TYPE			= 64,   // 0x0040
		MNT_EXCLUDE_WHITESPACE		= 123,  // 0x007b
		MNT_LONE_END_TAG			= 128,  // 0x0080
		MNT_NODE_ERROR              = 32768 // 0x8000
	};

	// Create
	bool Save( MCD_CSTR_FILENAME szFileName );
	const MCD_STR& GetDoc() const { return m_strDoc; };
	bool AddElem( MCD_CSTR szName, MCD_CSTR szData=NULL, int nFlags=0 ) { return x_AddElem(szName,szData,nFlags); };
	bool InsertElem( MCD_CSTR szName, MCD_CSTR szData=NULL, int nFlags=0 ) { return x_AddElem(szName,szData,nFlags|MNF_INSERT); };
	bool AddChildElem( MCD_CSTR szName, MCD_CSTR szData=NULL, int nFlags=0 ) { return x_AddElem(szName,szData,nFlags|MNF_CHILD); };
	bool InsertChildElem( MCD_CSTR szName, MCD_CSTR szData=NULL, int nFlags=0 ) { return x_AddElem(szName,szData,nFlags|MNF_INSERT|MNF_CHILD); };
	bool AddElem( MCD_CSTR szName, int nValue, int nFlags=0 ) { return x_AddElem(szName,nValue,nFlags); };
	bool InsertElem( MCD_CSTR szName, int nValue, int nFlags=0 ) { return x_AddElem(szName,nValue,nFlags|MNF_INSERT); };
	bool AddChildElem( MCD_CSTR szName, int nValue, int nFlags=0 ) { return x_AddElem(szName,nValue,nFlags|MNF_CHILD); };
	bool InsertChildElem( MCD_CSTR szName, int nValue, int nFlags=0 ) { return x_AddElem(szName,nValue,nFlags|MNF_INSERT|MNF_CHILD); };
	bool AddAttrib( MCD_CSTR szAttrib, MCD_CSTR szValue ) { return x_SetAttrib(m_iPos,szAttrib,szValue); };
	bool AddChildAttrib( MCD_CSTR szAttrib, MCD_CSTR szValue ) { return x_SetAttrib(m_iPosChild,szAttrib,szValue); };
	bool AddAttrib( MCD_CSTR szAttrib, int nValue ) { return x_SetAttrib(m_iPos,szAttrib,nValue); };
	bool AddChildAttrib( MCD_CSTR szAttrib, int nValue ) { return x_SetAttrib(m_iPosChild,szAttrib,nValue); };
	bool AddSubDoc( MCD_CSTR szSubDoc ) { return x_AddSubDoc(szSubDoc,0); };
	bool InsertSubDoc( MCD_CSTR szSubDoc ) { return x_AddSubDoc(szSubDoc,MNF_INSERT); };
	MCD_STR GetSubDoc() { return x_GetSubDoc(m_iPos); };
	bool AddChildSubDoc( MCD_CSTR szSubDoc ) { return x_AddSubDoc(szSubDoc,MNF_CHILD); };
	bool InsertChildSubDoc( MCD_CSTR szSubDoc ) { return x_AddSubDoc(szSubDoc,MNF_CHILD|MNF_INSERT); };
	MCD_STR GetChildSubDoc() { return x_GetSubDoc(m_iPosChild); };
	bool AddNode( int nType, MCD_CSTR szText ) { return x_AddNode(nType,szText,0); };
	bool InsertNode( int nType, MCD_CSTR szText ) { return x_AddNode(nType,szText,MNF_INSERT); };

	// Modify
	bool RemoveElem();
	bool RemoveChildElem();
	bool RemoveNode();
	bool SetAttrib( MCD_CSTR szAttrib, MCD_CSTR szValue, int nFlags=0 ) { return x_SetAttrib(m_iPos,szAttrib,szValue,nFlags); };
	bool SetChildAttrib( MCD_CSTR szAttrib, MCD_CSTR szValue, int nFlags=0 ) { return x_SetAttrib(m_iPosChild,szAttrib,szValue,nFlags); };
	bool SetAttrib( MCD_CSTR szAttrib, int nValue, int nFlags=0 ) { return x_SetAttrib(m_iPos,szAttrib,nValue,nFlags); };
	bool SetChildAttrib( MCD_CSTR szAttrib, int nValue, int nFlags=0 ) { return x_SetAttrib(m_iPosChild,szAttrib,nValue,nFlags); };
	bool SetData( MCD_CSTR szData, int nFlags=0 ) { return x_SetData(m_iPos,szData,nFlags); };
	bool SetChildData( MCD_CSTR szData, int nFlags=0 ) { return x_SetData(m_iPosChild,szData,nFlags); };
	bool SetData( int nValue ) { return x_SetData(m_iPos,nValue); };
	bool SetChildData( int nValue ) { return x_SetData(m_iPosChild,nValue); };
	bool SetElemContent( MCD_CSTR szContent ) { return x_SetElemContent(szContent); };


	// Utility
	static bool ReadTextFile( MCD_CSTR_FILENAME szFileName, MCD_STR& strDoc, MCD_STR* pstrResult=NULL, int* pnDocFlags=NULL, MCD_STR* pstrEncoding=NULL );
	static bool WriteTextFile( MCD_CSTR_FILENAME szFileName, const MCD_STR& strDoc, MCD_STR* pstrResult=NULL, int* pnDocFlags=NULL, MCD_STR* pstrEncoding=NULL );
	static MCD_STR EscapeText( MCD_CSTR szText, int nFlags = 0 );
	static MCD_STR UnescapeText( MCD_CSTR szText, int nTextLength = -1, int nFlags = 0 );
	static int UTF16To8( char *pszUTF8, const unsigned short* pwszUTF16, int nUTF8Count );
	static int UTF8To16( unsigned short* pwszUTF16, const char* pszUTF8, int nUTF8Count );
	static MCD_STR UTF8ToA( MCD_CSTR pszUTF8, int* pnFailed = NULL );
	static MCD_STR AToUTF8( MCD_CSTR pszANSI );
	static void EncodeCharUTF8( int nUChar, char* pszUTF8, int& nUTF8Len );
	static int DecodeCharUTF8( const char*& pszUTF8, const char* pszUTF8End = NULL );
	static void EncodeCharUTF16( int nUChar, unsigned short* pwszUTF16, int& nUTF16Len );
	static int DecodeCharUTF16( const unsigned short*& pwszUTF16, const unsigned short* pszUTF16End = NULL );
	static bool DetectUTF8( const char* pText, int nTextLen, int* pnNonASCII = NULL, bool* bErrorAtEnd = NULL );
	static MCD_STR GetDeclaredEncoding( MCD_CSTR szDoc );
	static int GetEncodingCodePage( MCD_CSTR pszEncoding );

protected:

#if defined(_DEBUG)
	MCD_PCSZ m_pDebugCur;
	MCD_PCSZ m_pDebugPos;
#endif // DEBUG

	MCD_STR m_strDoc;
	MCD_STR m_strResult;

	int m_iPosParent;
	int m_iPos;
	int m_iPosChild;
	int m_iPosFree;
	int m_iPosDeleted;
	int m_nNodeType;
	int m_nNodeOffset;
	int m_nNodeLength;
	int m_nDocFlags;

	FilePos* m_pFilePos;
	SavedPosMapArray* m_pSavedPosMaps;
	ElemPosTree* m_pElemPosTree;

	enum MarkupNodeFlagsInternal
	{
		MNF_INSERT     = 0x002000,
		MNF_CHILD      = 0x004000
	};

#if defined(_DEBUG) // DEBUG 
	void x_SetDebugState();
#define MARKUP_SETDEBUGSTATE x_SetDebugState()
#else // not DEBUG
#define MARKUP_SETDEBUGSTATE
#endif // not DEBUG

	void x_InitMarkup();
	void x_SetPos( int iPosParent, int iPos, int iPosChild );
	int x_GetFreePos();
	bool x_AllocElemPos( int nNewSize = 0 );
	int x_GetParent( int i );
	bool x_ParseDoc();
	int x_ParseElem( int iPos, TokenPos& token );
	int x_FindElem( int iPosParent, int iPos, PathPos& path ) const;
	MCD_STR x_GetPath( int iPos ) const;
	MCD_STR x_GetTagName( int iPos ) const;
	MCD_STR x_GetData( int iPos );
	MCD_STR x_GetAttrib( int iPos, MCD_PCSZ pAttrib ) const;
	static MCD_STR x_EncodeCDATASection( MCD_PCSZ szData );
	bool x_AddElem( MCD_PCSZ pName, MCD_PCSZ pValue, int nFlags );
	bool x_AddElem( MCD_PCSZ pName, int nValue, int nFlags );
	MCD_STR x_GetSubDoc( int iPos );
	bool x_AddSubDoc( MCD_PCSZ pSubDoc, int nFlags );
	bool x_SetAttrib( int iPos, MCD_PCSZ pAttrib, MCD_PCSZ pValue, int nFlags=0 );
	bool x_SetAttrib( int iPos, MCD_PCSZ pAttrib, int nValue, int nFlags=0 );
	bool x_AddNode( int nNodeType, MCD_PCSZ pText, int nNodeFlags );
	void x_RemoveNode( int iPosParent, int& iPos, int& nNodeType, int& nNodeOffset, int& nNodeLength );
	static bool x_CreateNode( MCD_STR& strNode, int nNodeType, MCD_PCSZ pText );
	int x_InsertNew( int iPosParent, int& iPosRel, NodePos& node );
	void x_AdjustForNode( int iPosParent, int iPos, int nShift );
	void x_Adjust( int iPos, int nShift, bool bAfterPos = false );
	void x_LinkElem( int iPosParent, int iPosBefore, int iPos );
	int x_UnlinkElem( int iPos );
	int x_UnlinkPrevElem( int iPosParent, int iPosBefore, int iPos );
	int x_ReleaseSubDoc( int iPos );
	int x_ReleasePos( int iPos );
	void x_CheckSavedPos();
	bool x_SetData( int iPos, MCD_PCSZ szData, int nFlags );
	bool x_SetData( int iPos, int nValue );
	int x_RemoveElem( int iPos );
	MCD_STR x_GetElemContent( int iPos ) const;
	bool x_SetElemContent( MCD_PCSZ szContent );
	void x_DocChange( int nLeft, int nReplace, const MCD_STR& strInsert );
};

#endif // !defined(_MARKUP_H_INCLUDED_)
