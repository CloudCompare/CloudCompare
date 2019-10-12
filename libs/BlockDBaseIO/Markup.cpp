// Markup.cpp: implementation of the CMarkup class.
//
// Markup Release 10.0
// Copyright (C) 2008 First Objective Software, Inc. All rights reserved
// Go to www.firstobject.com for the latest CMarkup and EDOM documentation
// Use in commercial applications requires written permission
// This software is provided "as is", with no warranty.
//

//#include "stdafx.h"
#include <stdio.h>
#include "Markup.h"

#if defined(MCD_STRERROR) // C error routine
#include <string.h>
#include <errno.h>
#endif // C error routine

#if defined(MARKUP_STL) && ((! defined(MARKUP_STDCONV)) || (! defined(MCD_STRERROR)))
#include <windows.h> // for MultiByteToWideChar, WideCharToMultiByte, FormatMessage
#endif // need Windows.h when STL and (not setlocale or not strerror), MFC afx.h includes it already 

#if defined(MBCS) // MBCS/double byte
#pragma message( "Note: MBCS build (not UTF-8)" )
// For UTF-8, remove MBCS from project settings C/C++ preprocessor definitions
#if defined (MARKUP_STDCONV)
#include <locale.h>
#else // not STDCONV
#include <mbstring.h> // for _mbclen
#endif // not STDCONV
#endif // MBCS/double byte

// #if defined(_DEBUG) && _MSC_VER > 1000 // VC++ DEBUG
// #undef THIS_FILE
// static char THIS_FILE[]=__FILE__;
// #if defined(DEBUG_NEW)
// #define new DEBUG_NEW
// #endif // DEBUG_NEW
// #endif // VC++ DEBUG

// Customization
#define x_EOL MCD_T("\r\n") // can be \r\n or \n or empty
#define x_EOLLEN (sizeof(x_EOL)/sizeof(MCD_CHAR)-1) // string length of x_EOL
#define x_ATTRIBQUOTE MCD_T("\"") // can be double or single quote


// Disable "while ( 1 )" warning in VC++ 2002
#if _MSC_VER >= 1300 // VC++ 2002 (7.0)
#pragma warning(disable:4127)
#endif // VC++ 2002 (7.0)

void CMarkup::operator=( const CMarkup& markup )
{
	m_iPosParent = markup.m_iPosParent;
	m_iPos = markup.m_iPos;
	m_iPosChild = markup.m_iPosChild;
	m_iPosFree = markup.m_iPosFree;
	m_iPosDeleted = markup.m_iPosDeleted;
	m_nNodeType = markup.m_nNodeType;
	m_nNodeOffset = markup.m_nNodeOffset;
	m_nNodeLength = markup.m_nNodeLength;
	m_strDoc = markup.m_strDoc;
	m_strError = markup.m_strError;
	m_nDocFlags = markup.m_nDocFlags;

	// Copy used part of the index array
	m_aPos.RemoveAll();
	m_aPos.nSize = m_iPosFree;
	if ( m_aPos.nSize < 8 )
		m_aPos.nSize = 8;
	m_aPos.nSegs = m_aPos.SegsUsed();
	if ( m_aPos.nSegs )
	{
		m_aPos.pSegs = (ElemPos**)(new char[m_aPos.nSegs*sizeof(char*)]);
		int nSegSize = 1 << m_aPos.PA_SEGBITS;
		for ( int nSeg=0; nSeg < m_aPos.nSegs; ++nSeg )
		{
			if ( nSeg + 1 == m_aPos.nSegs )
				nSegSize = m_aPos.GetSize() - (nSeg << m_aPos.PA_SEGBITS);
			m_aPos.pSegs[nSeg] = (ElemPos*)(new char[nSegSize*sizeof(ElemPos)]);
			memcpy( m_aPos.pSegs[nSeg], markup.m_aPos.pSegs[nSeg], nSegSize*sizeof(ElemPos) );
		}
	}

	// Copy SavedPos map(s)
	m_SavedPosMapArray.RemoveAll();
	if ( markup.m_SavedPosMapArray.pMaps )
	{
		int nMap = 0;
		SavedPosMap* pMap = NULL;
		while ( markup.m_SavedPosMapArray.pMaps[nMap] )
		{
			SavedPosMap* pMapSrc = markup.m_SavedPosMapArray.pMaps[nMap];
			x_GetMap( pMap, nMap, pMapSrc->nMapSize );
			for ( int nSlot=0; nSlot < pMap->nMapSize; ++nSlot )
			{
				SavedPos* pCopySavedPos = pMapSrc->pTable[nSlot];
				if ( pCopySavedPos )
				{
					int nCount = 0;
					while ( pCopySavedPos[nCount].nSavedPosFlags & SavedPos::SPM_USED )
					{
						++nCount;
						if ( pCopySavedPos[nCount-1].nSavedPosFlags & SavedPos::SPM_LAST )
							break;
					}
					if ( nCount )
					{
						SavedPos* pNewSavedPos = new SavedPos[nCount];
						for ( int nCopy=0; nCopy<nCount; ++nCopy )
							pNewSavedPos[nCopy] = pCopySavedPos[nCopy];
						pNewSavedPos[nCount-1].nSavedPosFlags |= SavedPos::SPM_LAST;
						pMap->pTable[nSlot] = pNewSavedPos;
					}
				}
			}
			++nMap;
		}
	}

	MARKUP_SETDEBUGSTATE;
}

bool CMarkup::SetDoc( MCD_PCSZ pDoc )
{
	// Set document text
	if ( pDoc )
		m_strDoc = pDoc;
	else
		MCD_STRCLEAR(m_strDoc);

	MCD_STRCLEAR(m_strError);
	return x_ParseDoc();
};

bool CMarkup::SetDoc( const MCD_STR& strDoc )
{
	m_strDoc = strDoc;
	MCD_STRCLEAR(m_strError);
	return x_ParseDoc();
}

bool CMarkup::IsWellFormed()
{
	if ( m_aPos.GetSize()
			&& ! (m_aPos[0].nFlags & MNF_ILLFORMED)
			&& m_aPos[0].iElemChild
			&& ! m_aPos[m_aPos[0].iElemChild].iElemNext )
		return true;
	return false;
}

bool CMarkup::Load( MCD_CSTR szFileName )
{
	if ( ! ReadTextFile(szFileName, m_strDoc, &m_strError, &m_nDocFlags) )
		return false;
	return x_ParseDoc();
}

bool CMarkup::ReadTextFile( MCD_CSTR szFileName, MCD_STR& strDoc, MCD_STR* pstrError, int* pnDocFlags )
{
	// Static utility method to load text file into strDoc
	//
	// Open file to read binary
	FILE* fp = NULL;
	MCD_FOPEN( fp, szFileName, MCD_T("rb") );
	if ( ! fp )
	{
		if ( pstrError )
			*pstrError = x_GetLastError();
		return false;
	}

	// Set flags to 0 unless flags argument provided
	int nDocFlags = pnDocFlags?*pnDocFlags:0;
	MCD_CHAR szDescBOM[20] = {0};
	MCD_CHAR szResult[100];
	MCD_STRCLEAR(strDoc);

	// Get file length
	fseek( fp, 0, SEEK_END );
	int nFileByteLen = ftell( fp );
	fseek( fp, 0, SEEK_SET );


#if defined(UNICODE) // UNICODE
	// Convert file to wide char
	int nWideLen = 0;
	if ( nFileByteLen )
	{
		char* pBuffer = new char[nFileByteLen];
		fread( pBuffer, nFileByteLen, 1, fp );
		// XML Declaration check in first 100 bytes
		MCD_CHAR szDeclCheck[201];
		int nDeclByteCheckLen = nFileByteLen;
		if ( nDeclByteCheckLen > 100 )
			nDeclByteCheckLen = 100;
		nDeclByteCheckLen = UTF8To16(szDeclCheck,pBuffer,nDeclByteCheckLen);
		szDeclCheck[nDeclByteCheckLen] = '\0';
		// If not UTF-8, we assume it is in the current code page
		int nCP = MCD_UTF8;
		MCD_STR strEncoding = GetDeclaredEncoding( szDeclCheck );
		if ( (! MCD_STRISEMPTY(strEncoding)) && x_StrNIACMP(MCD_2PCSZ(strEncoding),MCD_T("UTF-8"),5)!=0 )
			nCP = MCD_ACP;
		nWideLen = x_MBToWC(nCP,NULL,0,pBuffer,nFileByteLen);
		int nBufferSizeForGrow = nWideLen + nWideLen/100; // extra 1%
		MCD_CHAR* pUTF16Buffer = MCD_GETBUFFER(strDoc,nBufferSizeForGrow);
		x_MBToWC(nCP,pUTF16Buffer,nWideLen,pBuffer,nFileByteLen);
		MCD_RELEASEBUFFER( strDoc, pUTF16Buffer, nWideLen );
		delete [] pBuffer;
	}
	MCD_SPRINTF( MCD_SSZ(szResult), MCD_T("%s%d bytes to %d wide chars"), szDescBOM, nFileByteLen, nWideLen );
	if ( pstrError )
		*pstrError = szResult;
#else // not UNICODE
	// Read file directly
	int nCharsLost = 0;
	if ( nFileByteLen )
	{
		int nBufferSizeForGrow = nFileByteLen + nFileByteLen/100; // extra 1%
		MCD_CHAR* pUTF8Buffer = MCD_GETBUFFER(strDoc,nBufferSizeForGrow);
		fread( pUTF8Buffer, nFileByteLen, 1, fp );
		MCD_RELEASEBUFFER( strDoc, pUTF8Buffer, nFileByteLen );
#if defined(MBCS) // MBCS/double byte
		// Needs to be in memory as MBCS
		MCD_STR strEncoding = GetDeclaredEncoding( strDoc );
		if ( MCD_STRISEMPTY(strEncoding) || x_StrNIACMP(MCD_2PCSZ(strEncoding),MCD_T("UTF-8"),5)==0 )
			strDoc = UTF8ToA( strDoc, &nCharsLost );
#endif // MBCS/double byte
	}
	MCD_SPRINTF( MCD_SSZ(szResult), MCD_T("%s%d bytes%s"), szDescBOM, nFileByteLen,
			nCharsLost?MCD_T(" (chars lost in conversion!)"):MCD_T("") );
	if ( pstrError )
		*pstrError = szResult;
#endif // not UNICODE
	fclose( fp );
	if ( pnDocFlags )
		*pnDocFlags = nDocFlags;
	return true;
}

bool CMarkup::Save( MCD_CSTR szFileName )
{
//	printf("CMarkup<Save>:%s\nDoc:%s\n",szFileName,m_strDoc.c_str());
	return WriteTextFile( szFileName, m_strDoc, &m_strError, &m_nDocFlags );
}

bool CMarkup::WriteTextFile( MCD_CSTR szFileName, MCD_STR& strDoc, MCD_STR* pstrError, int* pnDocFlags )
{
	// Static utility method to save strDoc to text file
	//
	// Open file to write binary
	bool bSuccess = true;
	FILE* fp = NULL;
	MCD_FOPEN( fp, szFileName, MCD_T("wb") );
	if ( ! fp )
	{
		if ( pstrError )
			*pstrError = x_GetLastError();
		return false;
	}

	// Set flags to 0 unless flags argument provided
	int nDocFlags = pnDocFlags?*pnDocFlags:0;
	MCD_CHAR szDescBOM[20] = {0};
	MCD_CHAR szResult[100];

	// Get document length
	int nDocLength = MCD_STRLENGTH(strDoc);


#if defined( UNICODE ) // UNICODE
	int nMBLen = 0;
	int nCharsLost = 0;
	if ( nDocLength )
	{
		int nCP = MCD_UTF8;
		MCD_STR strEncoding = GetDeclaredEncoding( strDoc );
		if ( (! MCD_STRISEMPTY(strEncoding)) && x_StrNIACMP(MCD_2PCSZ(strEncoding),MCD_T("UTF-8"),5)!=0 )
			nCP = MCD_ACP;
		nMBLen = x_WCToMB(nCP,NULL,0,MCD_2PCSZ(strDoc),nDocLength);
		char* pMBBuffer = new char[nMBLen+1];
		x_WCToMB(nCP,pMBBuffer,nMBLen,MCD_2PCSZ(strDoc),nDocLength,&nCharsLost);
		bSuccess = ( fwrite( pMBBuffer, nMBLen, 1, fp ) == 1 );
		delete [] pMBBuffer;
	}
	MCD_SPRINTF( MCD_SSZ(szResult), MCD_T("%d wide chars to %s%d bytes%s"), nDocLength, szDescBOM, nMBLen,
			nCharsLost?MCD_T(" (chars lost in conversion!)"):MCD_T("") );
	if ( pstrError )
		*pstrError = szResult;
#else // not UNICODE
	if ( nDocLength )
	{
		MCD_STR strDocWrite = strDoc; // reference unless converted
#if defined(MBCS) // MBCS/double byte
		MCD_STR strEncoding = GetDeclaredEncoding( strDoc );
		if ( MCD_STRISEMPTY(strEncoding) || x_StrNIACMP(MCD_2PCSZ(strEncoding),MCD_T("UTF-8"),5)==0 )
			strDocWrite = AToUTF8( strDoc );
#endif // MBCS/double byte
		nDocLength = MCD_STRLENGTH(strDocWrite);
		bSuccess = ( fwrite( MCD_2PCSZ(strDocWrite), nDocLength, 1, fp ) == 1 );
	}
	MCD_SPRINTF( MCD_SSZ(szResult), MCD_T("%s%d bytes"), szDescBOM, nDocLength );
	if ( pstrError )
		*pstrError = szResult;
#endif // not UNICODE
	
	if ( ! bSuccess && pstrError )
		*pstrError = x_GetLastError();
	fclose(fp);
	if ( pnDocFlags )
		*pnDocFlags = nDocFlags;
	return bSuccess;
}

bool CMarkup::FindElem( MCD_CSTR szName )
{
	// Change current position only if found
	//
	if ( m_aPos.GetSize() )
	{
		int iPos = x_FindElem( m_iPosParent, m_iPos, szName );
		if ( iPos )
		{
			// Assign new position
			x_SetPos( m_aPos[iPos].iElemParent, iPos, 0 );
			return true;
		}
	}
	return false;
}

bool CMarkup::FindChildElem( MCD_CSTR szName )
{
	// Change current child position only if found
	//
	// Shorthand: call this with no current main position
	// means find child under root element
	if ( ! m_iPos )
		FindElem();

	int iPosChild = x_FindElem( m_iPos, m_iPosChild, szName );
	if ( iPosChild )
	{
		// Assign new position
		int iPos = m_aPos[iPosChild].iElemParent;
		x_SetPos( m_aPos[iPos].iElemParent, iPos, iPosChild );
		return true;
	}

	return false;
}

MCD_STR CMarkup::EscapeText( MCD_CSTR szText, int nFlags )
{
	// Convert text as seen outside XML document to XML friendly
	// replacing special characters with ampersand escape codes
	// E.g. convert "6>7" to "6&gt;7"
	//
	// &lt;   less than
	// &amp;  ampersand
	// &gt;   greater than
	//
	// and for attributes:
	//
	// &apos; apostrophe or single quote
	// &quot; double quote
	//
	static MCD_PCSZ apReplace[] = { MCD_T("&lt;"),MCD_T("&amp;"),MCD_T("&gt;"),MCD_T("&apos;"),MCD_T("&quot;") };
	MCD_PCSZ pFind = (nFlags&MNF_ESCAPEQUOTES)?MCD_T("<&>\'\""):MCD_T("<&>");
	MCD_STR strText;
	MCD_PCSZ pSource = szText;
	int nDestSize = MCD_PSZLEN(pSource);
	nDestSize += nDestSize / 10 + 7;
	MCD_BLDRESERVE(strText,nDestSize);
	MCD_CHAR cSource = *pSource;
	MCD_PCSZ pFound;
	int nCharLen;
	while ( cSource )
	{
		MCD_BLDCHECK(strText,nDestSize,6);
		if ( (pFound=MCD_PSZCHR(pFind,cSource)) != NULL )
		{
			bool bIgnoreAmpersand = false;
			if ( (nFlags&MNF_WITHREFS) && *pFound == '&' )
			{
				// Do not replace ampersand if it is start of any entity reference
				// &[#_:A-Za-zU][_:-.A-Za-z0-9U]*; where U is > 0x7f
				MCD_PCSZ pCheckEntity = pSource;
				++pCheckEntity;
				MCD_CHAR c = *pCheckEntity;
				if ( (c>='A'&&c<='Z') || (c>='a'&&c<='z')
						|| c=='#' || c=='_' || c==':' || ((unsigned int)c)>0x7f )
				{
					while ( 1 )
					{
						pCheckEntity += MCD_CLEN( pCheckEntity );
						c = *pCheckEntity;
						if ( c == ';' )
						{
							int nEntityLen = (int)(pCheckEntity - pSource) + 1;
							MCD_BLDAPPENDN(strText,pSource,nEntityLen);
							pSource = pCheckEntity;
							bIgnoreAmpersand = true;
						}
						else if ( (c>='A'&&c<='Z') || (c>='a'&&c<='z') || (c>='0'&&c<='9')
								|| c=='_' || c==':' || c=='-' || c=='.' || ((unsigned int)c)>0x7f )
							continue;
						break;
					}
				}
			}
			if ( ! bIgnoreAmpersand )
			{
				pFound = apReplace[pFound-pFind];
				MCD_BLDAPPEND(strText,pFound);
			}
			++pSource; // ASCII, so 1 byte
		}
		else
		{
			nCharLen = MCD_CLEN( pSource );
			MCD_BLDAPPENDN(strText,pSource,nCharLen);
			pSource += nCharLen;
		}
		cSource = *pSource;
	}

	MCD_BLDRELEASE(strText);
	return strText;
}

// Predefined character entities
// By default UnescapeText will decode standard HTML entities as well as the 5 in XML
// To unescape only the 5 standard XML entities, use this short table instead:
// MCD_PCSZ PredefEntityTable[4] =
// { MCD_T("20060lt"),MCD_T("40034quot"),MCD_T("30038amp"),MCD_T("20062gt40039apos") };
//
// This is a precompiled ASCII hash table for speed and minimum memory requirement
// Each entry consists of a 1 digit code name length, 4 digit code point, and the code name
// Each table slot can have multiple entries, table size 130 was chosen for even distribution
//
MCD_PCSZ PredefEntityTable[130] =
{
	MCD_T("60216oslash60217ugrave60248oslash60249ugrave"),
	MCD_T("50937omega60221yacute58968lceil50969omega60253yacute"),
	MCD_T("50916delta50206icirc50948delta50238icirc68472weierp"),MCD_T("40185sup1"),
	MCD_T("68970lfloor40178sup2"),
	MCD_T("50922kappa60164curren50954kappa58212mdash40179sup3"),
	MCD_T("59830diams58211ndash"),MCD_T("68855otimes58969rceil"),
	MCD_T("50338oelig50212ocirc50244ocirc50339oelig58482trade"),
	MCD_T("50197aring50931sigma50229aring50963sigma"),
	MCD_T("50180acute68971rfloor50732tilde"),MCD_T("68249lsaquo"),
	MCD_T("58734infin68201thinsp"),MCD_T("50161iexcl"),
	MCD_T("50920theta50219ucirc50952theta50251ucirc"),MCD_T("58254oline"),
	MCD_T("58260frasl68727lowast"),MCD_T("59827clubs60191iquest68250rsaquo"),
	MCD_T("58629crarr50181micro"),MCD_T("58222bdquo"),MCD_T(""),
	MCD_T("58243prime60177plusmn58242prime"),MCD_T("40914beta40946beta"),MCD_T(""),
	MCD_T(""),MCD_T(""),MCD_T("50171laquo50215times"),MCD_T("40710circ"),
	MCD_T("49001lang"),MCD_T("58220ldquo40175macr"),
	MCD_T("40182para50163pound48476real"),MCD_T(""),MCD_T("58713notin50187raquo"),
	MCD_T("48773cong50223szlig50978upsih"),
	MCD_T("58776asymp58801equiv49002rang58218sbquo"),
	MCD_T("50222thorn48659darr48595darr40402fnof58221rdquo50254thorn"),
	MCD_T("40162cent58722minus"),MCD_T("58707exist40170ordf"),MCD_T(""),
	MCD_T("40921iota58709empty48660harr48596harr40953iota"),MCD_T(""),
	MCD_T("40196auml40228auml48226bull40167sect48838sube"),MCD_T(""),
	MCD_T("48656larr48592larr58853oplus"),MCD_T("30176deg58216lsquo40186ordm"),
	MCD_T("40203euml40039apos40235euml48712isin40160nbsp"),
	MCD_T("40918zeta40950zeta"),MCD_T("38743and48195emsp48719prod"),
	MCD_T("30935chi38745cap30967chi48194ensp"),
	MCD_T("40207iuml40239iuml48706part48869perp48658rarr48594rarr"),
	MCD_T("38736ang48836nsub58217rsquo"),MCD_T(""),
	MCD_T("48901sdot48657uarr48593uarr"),MCD_T("40169copy48364euro"),
	MCD_T("30919eta30951eta"),MCD_T("40214ouml40246ouml48839supe"),MCD_T(""),
	MCD_T(""),MCD_T("30038amp30174reg"),MCD_T("48733prop"),MCD_T(""),
	MCD_T("30208eth30934phi40220uuml30240eth30966phi40252uuml"),MCD_T(""),MCD_T(""),
	MCD_T(""),MCD_T("40376yuml40255yuml"),MCD_T(""),MCD_T("40034quot48204zwnj"),
	MCD_T("38746cup68756there4"),MCD_T("30929rho30961rho38764sim"),
	MCD_T("30932tau38834sub30964tau"),MCD_T("38747int38206lrm38207rlm"),
	MCD_T("30936psi30968psi30165yen"),MCD_T(""),MCD_T("28805ge30168uml"),
	MCD_T("30982piv"),MCD_T(""),MCD_T("30172not"),MCD_T(""),MCD_T("28804le"),
	MCD_T("30173shy"),MCD_T("39674loz28800ne38721sum"),MCD_T(""),MCD_T(""),
	MCD_T("38835sup"),MCD_T("28715ni"),MCD_T(""),MCD_T("20928pi20960pi38205zwj"),
	MCD_T(""),MCD_T("60923lambda20062gt60955lambda"),MCD_T(""),MCD_T(""),
	MCD_T("60199ccedil60231ccedil"),MCD_T(""),MCD_T("20060lt"),
	MCD_T("20926xi28744or20958xi"),MCD_T("20924mu20956mu"),MCD_T("20925nu20957nu"),
	MCD_T("68225dagger68224dagger"),MCD_T("80977thetasym"),MCD_T(""),MCD_T(""),
	MCD_T(""),MCD_T("78501alefsym"),MCD_T(""),MCD_T(""),MCD_T(""),
	MCD_T("60193aacute60195atilde60225aacute60227atilde"),MCD_T(""),
	MCD_T("70927omicron60247divide70959omicron"),MCD_T("60192agrave60224agrave"),
	MCD_T("60201eacute60233eacute60962sigmaf"),MCD_T("70917epsilon70949epsilon"),
	MCD_T(""),MCD_T("60200egrave60232egrave"),MCD_T("60205iacute60237iacute"),
	MCD_T(""),MCD_T(""),MCD_T("60204igrave68230hellip60236igrave"),
	MCD_T("60166brvbar"),
	MCD_T("60209ntilde68704forall58711nabla60241ntilde69824spades"),
	MCD_T("60211oacute60213otilde60189frac1260183middot60243oacute60245otilde"),
	MCD_T(""),MCD_T("50184cedil60188frac14"),
	MCD_T("50198aelig50194acirc60210ograve50226acirc50230aelig60242ograve"),
	MCD_T("50915gamma60190frac3450947gamma58465image58730radic"),
	MCD_T("60352scaron60353scaron"),MCD_T("60218uacute69829hearts60250uacute"),
	MCD_T("50913alpha50202ecirc70933upsilon50945alpha50234ecirc70965upsilon"),
	MCD_T("68240permil")
};

MCD_STR CMarkup::UnescapeText( MCD_CSTR szText, int nTextLength /*=-1*/ )
{
	// Convert XML friendly text to text as seen outside XML document
	// ampersand escape codes replaced with special characters e.g. convert "6&gt;7" to "6>7"
	// ampersand numeric codes replaced with character e.g. convert &#60; to <
	// Conveniently the result is always the same or shorter in byte length
	//
	MCD_STR strText;
	MCD_PCSZ pSource = szText;
	if ( nTextLength == -1 )
		nTextLength = MCD_PSZLEN(szText);
	MCD_BLDRESERVE(strText,nTextLength);
	MCD_CHAR szCodeName[10];
	int nCharLen;
	int nChar = 0;
	while ( nChar < nTextLength )
	{
		if ( pSource[nChar] == '&' )
		{
			// Get corresponding unicode code point
			int nUnicode = 0;

			// Look for terminating semi-colon within 9 ASCII characters
			int nCodeLen = 0;
			MCD_CHAR cCodeChar = pSource[nChar+1];
			while ( nCodeLen < 9 && ((unsigned int)cCodeChar) < 128 && cCodeChar != ';' )
			{
				if ( cCodeChar >= 'A' && cCodeChar <= 'Z') // upper case?
					cCodeChar += ('a' - 'A'); // make lower case
				szCodeName[nCodeLen] = cCodeChar;
				++nCodeLen;
				cCodeChar = pSource[nChar+1+nCodeLen];
			}
			if ( cCodeChar == ';' ) // found semi-colon?
			{
				// Decode szCodeName
				szCodeName[nCodeLen] = '\0';
				if ( *szCodeName == '#' ) // numeric character reference?
				{
					// Is it a hex number?
					int nBase = 10; // decimal
					int nNumberOffset = 1; // after #
					if ( szCodeName[1] == 'x' )
					{
						nNumberOffset = 2; // after #x
						nBase = 16; // hex
					}
					nUnicode = MCD_PSZTOL( &szCodeName[nNumberOffset], NULL, nBase );
				}
				else // does not start with #
				{
					// Look for matching code name in PredefEntityTable
					MCD_PCSZ pEntry = PredefEntityTable[x_Hash(szCodeName,sizeof(PredefEntityTable)/sizeof(MCD_PCSZ))];
					while ( *pEntry )
					{
						// e.g. entry: 40039apos means length 4, code point 0039, code name apos
						int nEntryLen = (*pEntry - '0');
						++pEntry;
						MCD_PCSZ pCodePoint = pEntry;
						pEntry += 4;
						if ( nEntryLen == nCodeLen && MCD_PSZNCMP(szCodeName,pEntry,nEntryLen) == 0 )
						{
							// Convert digits to integer up to code name which always starts with alpha 
							nUnicode = MCD_PSZTOL( pCodePoint, NULL, 10 );
							break;
						}
						pEntry += nEntryLen;
					}
				}
			}

			// If a code point found, encode it into text
			if ( nUnicode )
			{
#if defined(UNICODE) // UNICODE
				MCD_BLDAPPEND1(strText,nUnicode);
#elif defined(MBCS) // MBCS/double byte
				// Locale code page
				MCD_CHAR szANSI[5];
				wchar_t wcUnicode = (wchar_t)nUnicode;
#if defined(MARKUP_STDCONV)
				int nMBLen = wctomb( szANSI, wcUnicode );
#else // not STDCONV
				int nUsedDefaultChar = 0;
				int nMBLen = WideCharToMultiByte( CP_ACP, 0, &wcUnicode, 1, szANSI, 5, NULL, &nUsedDefaultChar );
				if ( nUsedDefaultChar )
					nMBLen = 0;
#endif // not STDCONV
				if ( nMBLen > 0 )
				{
					MCD_BLDAPPENDN(strText,szANSI,nMBLen);
				}
				else
					nUnicode = 0;
#else // not UNICODE and not MBCS/double byte
				// UTF-8
				if ( nUnicode < 0x80 )
					MCD_BLDAPPEND1(strText,nUnicode);
				else if ( nUnicode < 0x800 )
				{
					// Convert to 2-byte UTF-8
					MCD_BLDAPPEND1(strText,((nUnicode&0x7c0)>>6)|0xc0);
					MCD_BLDAPPEND1(strText,(nUnicode&0x3f)|0x80);
				}
				else
				{
					// Convert to 3-byte UTF-8
					MCD_BLDAPPEND1(strText,((nUnicode&0xf000)>>12)|0xe0);
					MCD_BLDAPPEND1(strText,((nUnicode&0xfc0)>>6)|0x80);
					MCD_BLDAPPEND1(strText,(nUnicode&0x3f)|0x80);
				}
#endif // not UNICODE and not MBCS/double byte

				// Increment index past ampersand semi-colon
				if ( nUnicode ) // must check since MBCS case can clear it
					nChar += nCodeLen + 2;
			}
			if ( ! nUnicode )
			{
				// If the code is not converted, leave it as is
				MCD_BLDAPPEND1(strText,'&');
				++nChar;
			}
		}
		else // not &
		{
			nCharLen = MCD_CLEN(&pSource[nChar]);
			MCD_BLDAPPENDN(strText,&pSource[nChar],nCharLen);
			nChar += nCharLen;
		}
	}
	MCD_BLDRELEASE(strText);
	return strText;
}

int CMarkup::x_WCToMB( int nCP, char* pszMB, int nMBCount, const wchar_t* pwszWide, int nWideLen, int* pnFailed/*=NULL*/ )
{
	// If pnFailed, it will be set to >0 if characters failed (for STDCONV it returns count)
	if ( pnFailed )
		*pnFailed = 0;
#if defined(MARKUP_STDCONV)
	if ( nCP == MCD_UTF8 )
		return UTF16To8(pszMB,pwszWide,nMBCount);
	else
	{
		// return wcstombs(pszMB,pwszWide,nMBCount);
		// wcstombs gives up on the first char it can't convert, so use wctomb instead
		char szCharMB[5];
		int nCharLen = 0;
		int nMBLen = 0;
		// Like wcstombs, this does not verify within nMBCount
		while ( *pwszWide && nWideLen != 0 )
		{
			nCharLen = wctomb( szCharMB, *pwszWide );
			if ( nCharLen == -1 )
			{
				if ( pnFailed )
					++(*pnFailed);
				if ( pszMB )
					pszMB[nMBLen] = '?';
				++nMBLen;
			}
			else
			{
				if ( pszMB )
					memcpy( &pszMB[nMBLen], szCharMB, nCharLen );
				nMBLen += nCharLen;
			}
			--nWideLen;
			++pwszWide;
		}
		return nMBLen;
	}
#else // not STDCONV
	return WideCharToMultiByte(nCP,0,pwszWide,nWideLen,pszMB,
			nMBCount?nMBCount+1:0,NULL,
			(nCP==CP_UTF8)?NULL:pnFailed); // fails with lpUsedDefaultChar and CP_UTF8
#endif // not STDCONV
}

int CMarkup::UTF16To8( char* pszUTF8, const wchar_t* pwszUTF16, int nUTF8Count )
{
	// Supports the same arguments as wcstombs
	// the pwszUTF16 source must be a NULL-terminated UTF-16 string
	// if pszUTF8 is NULL, the number of bytes required is returned and nUTF8Count is ignored
	// otherwise pszUTF8 is filled with the result string and NULL-terminated if nUTF8Count allows
	// nUTF8Count is the byte size of pszUTF8 and must be large enough for the NULL if NULL desired
	// and the number of bytes (excluding NULL) is returned
	//
	int nUChar, nUTF8Len = 0;
	while ( *pwszUTF16 )
	{
		// Decode UTF-16
		nUChar = DecodeCharUTF16( pwszUTF16 );
		if ( nUChar == -1 )
			nUChar = '?';

		// Encode UTF-8
		if ( pszUTF8 && nUTF8Len + 4 > nUTF8Count )
		{
			int nUTF8LenSoFar = nUTF8Len;
			EncodeCharUTF8( nUChar, NULL, nUTF8Len );
			if ( nUTF8Len > nUTF8Count )
				return nUTF8LenSoFar;
			nUTF8Len = nUTF8LenSoFar;
		}
		EncodeCharUTF8( nUChar, pszUTF8, nUTF8Len );
	}
	if ( pszUTF8 && nUTF8Len < nUTF8Count )
		pszUTF8[nUTF8Len] = 0;
	return nUTF8Len;
}

int CMarkup::DecodeCharUTF8( const char*& pszUTF8 )
{
	// Return Unicode code point and increment pszUTF8 past 1-4 bytes
	int nUChar = (unsigned char)*pszUTF8;
	++pszUTF8;
	if ( nUChar & 0x80 )
	{
		int nExtraChars;
		if ( ! (nUChar & 0x20) )
		{
			nExtraChars = 1;
			nUChar &= 0x1f;
		}
		else if ( ! (nUChar & 0x10) )
		{
			nExtraChars = 2;
			nUChar &= 0x0f;
		}
		else if ( ! (nUChar & 0x08) )
		{
			nExtraChars = 3;
			nUChar &= 0x07;
		}
		else
			return -1;
		while ( nExtraChars-- )
		{
			if ( (*pszUTF8 & 0x80) )
			{
				nUChar = nUChar<<6;
				nUChar |= *pszUTF8 & 0x3f;
			}
			else
				return -1;
			++pszUTF8;
		}
	}
	return nUChar;
}

void CMarkup::EncodeCharUTF16( int nUChar, wchar_t* pwszUTF16, int& nWideLen )
{
	// Write UTF-16 sequence to pwszUTF16 for Unicode code point nUChar and update nWideLen
	// Be sure pwszUTF16 has room for up to 2 wide chars
	//
	if ( nUChar & ~0xffff )
	{
		if ( pwszUTF16 )
		{
			// Surrogate pair
			nUChar -= 0x10000;
			pwszUTF16[nWideLen++] = (wchar_t)(((nUChar>>10) & 0x3ff) | 0xd800); // W1
			pwszUTF16[nWideLen++] = (wchar_t)((nUChar & 0x3ff) | 0xdc00); // W2
		}
		else
			nWideLen += 2;
	}
	else
	{
		if ( pwszUTF16 )
			pwszUTF16[nWideLen++] = (wchar_t)nUChar;
		else
			++nWideLen;
	}
}

int CMarkup::x_MBToWC( int nCP, wchar_t* pwszWide, int nWideCount, const char* pszMB, int nMBLen )
{
#if defined(MARKUP_STDCONV)
	nWideCount; // unreferenced in this build
	if ( nCP == MCD_UTF8 )
		return UTF8To16(pwszWide,pszMB,nMBLen);
	else
		return mbstowcs(pwszWide,pszMB,nMBLen);
#else // not STDCONV
	return MultiByteToWideChar(nCP,0,pszMB,nMBLen,pwszWide,nWideCount);
#endif // not STDCONV
}

int CMarkup::UTF8To16( wchar_t* pwszUTF16, const char* pszUTF8, int nUTF8Count )
{
	// Supports the same arguments as mbstowcs
	// the pszUTF8 source must be a UTF-8 string which will be processed up to NULL-terminator or nUTF8Count
	// if pwszUTF16 is NULL, the number of wide chars required is returned
	// nUTF8Count is maximum UTF-8 bytes to convert and should include NULL if NULL desired in result
	// if pwszUTF16 is not NULL it is filled with the result string and it must be large enough
	// result will be NULL-terminated if NULL encountered in pszUTF8 before nUTF8Count
	// and the number of UTF-8 bytes converted is returned
	//
	const char* pszPosUTF8 = pszUTF8;
	int nUChar, nUTF8Len = 0, nWideLen = 0;
	while ( nUTF8Len < nUTF8Count )
	{
		// Decode UTF-8
		if ( nUTF8Len + 4 > nUTF8Count )
		{
			// Pre-examine UTF-8 character using temporary null-terminated copy
			// to see if this UTF-8 character boundary is within nUTF8Count
			char szUTF8Copy[5];
			const char* pszPosUTF8Copy = szUTF8Copy;
			int nUTF8EndCount = nUTF8Count - nUTF8Len;
			memcpy( szUTF8Copy, pszPosUTF8, nUTF8EndCount );
			szUTF8Copy[nUTF8EndCount] = '\0';
			nUChar = DecodeCharUTF8( pszPosUTF8Copy );
			int nUTF8EndLen = (int)(pszPosUTF8Copy - szUTF8Copy);
			if ( nUTF8Len + nUTF8EndLen > nUTF8Count )
				break;
		}
		nUChar = DecodeCharUTF8( pszPosUTF8 );
		nUTF8Len = (int)(pszPosUTF8 - pszUTF8);
		if ( ! nUChar )
		{
			if ( pwszUTF16 )
				pwszUTF16[nWideLen] = 0;
			break;
		}
		else if ( nUChar == -1 )
			nUChar = '?';

		// Encode UTF-16
		EncodeCharUTF16( nUChar, pwszUTF16, nWideLen );
	}
	if ( ! pwszUTF16 )
		return nWideLen;
	return nUTF8Len;
}

int CMarkup::DecodeCharUTF16( const wchar_t*& pwszUTF16 )
{
	// Return Unicode code point and increment pwszUTF16 past 1 or 2 (if surrogrates) wide chars
	int nUChar = *pwszUTF16;
	if ( (nUChar & ~0x000007ff) == 0xd800 ) // W1
	{
		++pwszUTF16;
		if ( ! *pwszUTF16 ) // W2
			return -1; // incorrect UTF-16
		nUChar = (((nUChar & 0x3ff) << 10) | (*pwszUTF16 & 0x3ff)) + 0x10000;
	}
	++pwszUTF16;
	return nUChar;
}

void CMarkup::EncodeCharUTF8( int nUChar, char* pszUTF8, int& nUTF8Len )
{
	// Write UTF-8 sequence to pszUTF8 for Unicode code point nUChar and update nUTF8Len
	// Be sure pszUTF8 has room for up to 4 bytes
	//
	if ( ! (nUChar & ~0x0000007f) ) // < 0x80
	{
		if ( pszUTF8 )
			pszUTF8[nUTF8Len++] = (char)nUChar;
		else
			++nUTF8Len;
	}
	else if ( ! (nUChar & ~0x000007ff) ) // < 0x800
	{
		if ( pszUTF8 )
		{
			pszUTF8[nUTF8Len++] = (char)(((nUChar&0x7c0)>>6)|0xc0);
			pszUTF8[nUTF8Len++] = (char)((nUChar&0x3f)|0x80);
		}
		else
			nUTF8Len += 2;
	}
	else if ( ! (nUChar & ~0x0000ffff) ) // < 0x10000
	{
		if ( pszUTF8 )
		{
			pszUTF8[nUTF8Len++] = (char)(((nUChar&0xf000)>>12)|0xe0);
			pszUTF8[nUTF8Len++] = (char)(((nUChar&0xfc0)>>6)|0x80);
			pszUTF8[nUTF8Len++] = (char)((nUChar&0x3f)|0x80);
		}
		else
			nUTF8Len += 3;
	}
	else // < 0x110000
	{
		if ( pszUTF8 )
		{
			pszUTF8[nUTF8Len++] = (char)(((nUChar&0x1c0000)>>18)|0xf0);
			pszUTF8[nUTF8Len++] = (char)(((nUChar&0x3f000)>>12)|0x80);
			pszUTF8[nUTF8Len++] = (char)(((nUChar&0xfc0)>>6)|0x80);
			pszUTF8[nUTF8Len++] = (char)((nUChar&0x3f)|0x80);
		}
		else
			nUTF8Len += 4;
	}
}

#if ! defined( UNICODE ) // not UNICODE
MCD_STR CMarkup::UTF8ToA( MCD_CSTR pszUTF8, int* pnFailed/*=NULL*/ )
{
	// Converts from UTF-8 to locale ANSI charset
	MCD_STR strANSI;
	int nCharLen = (int)strlen( pszUTF8 );
	if ( nCharLen )
	{
#if defined (MARKUP_STDCONV)
		// this uses wctomb which requires setlocale other than minimal "C" locale
		// e.g. setlocale(LC_ALL, "") enables the OS system locale settings
		int nBufferLen = nCharLen + 4;
		MCD_BLDRESERVE(strANSI,nBufferLen);
		int nUChar;
		MCD_CHAR szANSI[5];
		if ( pnFailed )
			*pnFailed = 0;
		MCD_PCSZ pUTF8 = pszUTF8;
		while ( *pUTF8 )
		{
			MCD_BLDCHECK(strANSI,nBufferLen,4); // was grow by (nBufferLen / 2 + 4)
			nUChar = DecodeCharUTF8( pUTF8 );
			if ( nUChar & ~0xffff )
				nCharLen = -1;
			else
				nCharLen = wctomb( szANSI, (wchar_t)nUChar );
			if ( nCharLen == -1 )
			{
				if ( pnFailed )
					++(*pnFailed);
				MCD_BLDAPPEND1(strANSI,'?');
			}
			else
			{
				MCD_BLDAPPENDN(strANSI,szANSI,nCharLen);
			}
		}
		MCD_BLDRELEASE(strANSI);
#else // not STDCONV
		wchar_t* pwszWide = new wchar_t[nCharLen+1];
		int nWideLen = MultiByteToWideChar(CP_UTF8,0,pszUTF8,nCharLen,pwszWide,nCharLen);
		pwszWide[nWideLen] = (wchar_t)0;
		nCharLen = nWideLen * 2 + 1;
		MCD_CHAR* pMBBuffer = MCD_GETBUFFER(strANSI,nCharLen);
		// pnFailed will not be count of failed characters, just 1 if any characters were unavailable
		nCharLen = WideCharToMultiByte(CP_ACP,0,pwszWide,nWideLen,pMBBuffer,nCharLen,NULL,pnFailed);
		delete[] pwszWide;
		MCD_RELEASEBUFFER(strANSI,pMBBuffer,nCharLen);
#endif
	}
	return strANSI;
}

MCD_STR CMarkup::AToUTF8( MCD_CSTR pszANSI )
{
	// Converts locale ANSI charset to UTF-8
	MCD_STR strUTF8;
	int nCharLen = (int)MCD_PSZLEN( pszANSI );
	if ( nCharLen )
	{
#if defined (MARKUP_STDCONV)
		// this uses mbtowc which requires setlocale other than minimal "C" locale
		// e.g. setlocale(LC_ALL, "") enables the OS system locale settings
		int nBufferLen = nCharLen * 2 + 4;
		MCD_BLDRESERVE(strUTF8,nBufferLen);
		int nUChar;
		wchar_t wcChar;
		MCD_CHAR szUTF8Char[4];
		MCD_PCSZ pANSI = pszANSI;
		while ( *pANSI )
		{
			MCD_BLDCHECK(strUTF8,nBufferLen,4);
			nCharLen = mbtowc( &wcChar, pANSI, 5 );
			if ( nCharLen < 1 )
			{
				nCharLen = 1;
				wcChar = (wchar_t)'?';
			}
			pANSI += nCharLen;
			nUChar = (int)wcChar;
			nCharLen = 0;
			EncodeCharUTF8( nUChar, szUTF8Char, nCharLen );
			MCD_BLDAPPENDN(strUTF8,szUTF8Char,nCharLen);
		}
		MCD_BLDRELEASE(strUTF8);
#else // not STDCONV
		wchar_t* pwszWide = new wchar_t[nCharLen+1];
		int nWideLen = MultiByteToWideChar(CP_ACP,0,pszANSI,nCharLen,pwszWide,nCharLen);
		pwszWide[nWideLen] = (wchar_t)0;
		nCharLen = nWideLen * 4 + 1;
		MCD_CHAR* pUTF8Buffer = MCD_GETBUFFER(strUTF8,nCharLen);
		nCharLen = WideCharToMultiByte(CP_UTF8,0,pwszWide,nWideLen,pUTF8Buffer,nCharLen,NULL,NULL);
		delete[] pwszWide;
		MCD_RELEASEBUFFER(strUTF8,pUTF8Buffer,nCharLen);
#endif
	}
	return strUTF8;
}
#endif // not UNICODE

MCD_STR CMarkup::GetDeclaredEncoding( MCD_CSTR szDoc )
{
	// Extract encoding attribute from XML Declaration
	MCD_STR strEncoding;
	MCD_PCSZ pStart = MCD_PSZCHR( szDoc, '<' );
	if ( pStart && pStart[1] == '?' )
	{
		MCD_PCSZ pEnd = MCD_PSZSTR( szDoc, MCD_T("?>") );
		if ( pEnd )
		{
			MCD_STR strXMLDecl( pStart, (int)(pEnd-pStart)+2 );
			CMarkup xmlDecl( strXMLDecl );
			if ( xmlDecl.FindNode() )
				strEncoding = xmlDecl.GetAttrib( MCD_T("encoding") );
		}
	}
	return strEncoding;
}


int CMarkup::FindNode( int nType )
{
	// Change current node position only if a node is found
	// If nType is 0 find any node, otherwise find node of type nType
	// Return type of node or 0 if not found
	// If found node is an element, change m_iPos

	// Determine where in document to start scanning for node
	int nTypeFound = 0;
	int nNodeOffset = m_nNodeOffset;
	if ( m_nNodeType > 1 )
	{
		// By-pass current node
		nNodeOffset += m_nNodeLength;
	}
	else
	{
		// Set position to begin looking for node
		nNodeOffset = 0; // default to start of document
		if ( m_iPos )
		{
			// After element
			nNodeOffset = m_aPos[m_iPos].StartAfter();
		}
		else if ( m_iPosParent )
		{
			// Immediately after start tag of parent
			if ( m_aPos[m_iPosParent].IsEmptyElement() )
				return 0;
			else
				nNodeOffset = m_aPos[m_iPosParent].StartContent();
		}
	}

	// Get nodes until we find what we're looking for
	int iPosNew = m_iPos;
	TokenPos token( m_strDoc, m_nDocFlags );
	NodePos node;
	token.nNext = nNodeOffset;
	do
	{
		nNodeOffset = token.nNext;
		nTypeFound = x_ParseNode( token, node );
		if ( nTypeFound == 0 )
		{
			// Check if we have reached the end of the parent element
			// Otherwise it is a lone end tag
			if ( m_iPosParent && nNodeOffset == m_aPos[m_iPosParent].StartContent()
					+ m_aPos[m_iPosParent].ContentLen() )
				return 0;
			nTypeFound = MNT_LONE_END_TAG;
		}
		else if ( nTypeFound < 0 )
		{
			if ( nTypeFound == -2 )
				return 0;
			// -1 is node error
			nTypeFound = MNT_NODE_ERROR;
		}
		else if ( nTypeFound == MNT_ELEMENT )
		{
			if ( iPosNew )
				iPosNew = m_aPos[iPosNew].iElemNext;
			else
				iPosNew = m_aPos[m_iPosParent].iElemChild;
			if ( ! iPosNew )
				return 0;
			if ( ! nType || (nType & nTypeFound) )
			{
				// Found element node, move position to this element
				x_SetPos( m_iPosParent, iPosNew, 0 );
				return m_nNodeType;
			}
			token.nNext = m_aPos[iPosNew].StartAfter();
		}
	}
	while ( nType && ! (nType & nTypeFound) );

	m_iPos = iPosNew;
	m_iPosChild = 0;
	m_nNodeOffset = nNodeOffset;
	m_nNodeLength = token.nNext - nNodeOffset;
	m_nNodeType = nTypeFound;
	MARKUP_SETDEBUGSTATE;
	return m_nNodeType;
}

bool CMarkup::RemoveNode()
{
	if ( m_iPos || m_nNodeLength )
	{
		x_RemoveNode( m_iPosParent, m_iPos, m_nNodeType, m_nNodeOffset, m_nNodeLength );
		m_iPosChild = 0;
		MARKUP_SETDEBUGSTATE;
		return true;
	}
	return false;
}

MCD_STR CMarkup::GetTagName() const
{
	// Return the tag name at the current main position
	MCD_STR strTagName;

	// This method is primarily for elements, however
	// it does return something for certain other nodes
	if ( m_nNodeLength )
	{
		switch ( m_nNodeType )
		{
		case MNT_PROCESSING_INSTRUCTION:
		case MNT_LONE_END_TAG:
			{
				// <?target or </tagname
				TokenPos token( m_strDoc, m_nDocFlags );
				token.nNext = m_nNodeOffset + 2;
				if ( x_FindName(token) )
					strTagName = x_GetToken( token );
			}
			break;
		case MNT_COMMENT:
			strTagName = MCD_T("#comment");
			break;
		case MNT_CDATA_SECTION:
			strTagName = MCD_T("#cdata-section");
			break;
		case MNT_DOCUMENT_TYPE:
			{
				// <!DOCTYPE name
				TokenPos token( m_strDoc, m_nDocFlags );
				token.nNext = m_nNodeOffset + 2;
				if ( x_FindName(token) && x_FindName(token) )
					strTagName = x_GetToken( token );
			}
			break;
		case MNT_TEXT:
		case MNT_WHITESPACE:
			strTagName = MCD_T("#text");
			break;
		}
		return strTagName;
	}

	if ( m_iPos )
		strTagName = x_GetTagName( m_iPos );
	return strTagName;
}

bool CMarkup::IntoElem()
{
	// If there is no child position and IntoElem is called it will succeed in release 6.3
	// (A subsequent call to FindElem will find the first element)
	// The following short-hand behavior was never part of EDOM and was misleading
	// It would find a child element if there was no current child element position and go into it
	// It is removed in release 6.3, this change is NOT backwards compatible!
	// if ( ! m_iPosChild )
	//	FindChildElem();

	if ( m_iPos && m_nNodeType == MNT_ELEMENT )
	{
		x_SetPos( m_iPos, m_iPosChild, 0 );
		return true;
	}
	return false;
}

bool CMarkup::OutOfElem()
{
	// Go to parent element
	if ( m_iPosParent )
	{
		x_SetPos( m_aPos[m_iPosParent].iElemParent, m_iPosParent, m_iPos );
		return true;
	}
	return false;
}

MCD_STR CMarkup::GetAttribName( int n ) const
{
	// Return nth attribute name of main position
	TokenPos token( m_strDoc, m_nDocFlags );
	if ( m_iPos && m_nNodeType == MNT_ELEMENT )
		token.nNext = m_aPos[m_iPos].nStart + 1;
	else if ( m_nNodeLength && m_nNodeType == MNT_PROCESSING_INSTRUCTION )
		token.nNext = m_nNodeOffset + 2;
	else
		return MCD_T("");
	if ( x_FindAttrib(token,NULL,n) )
		return x_GetToken( token );
	return MCD_T("");
}

bool CMarkup::SavePos( MCD_CSTR szPosName /*=""*/, int nMap /*=0*/ )
{
	// Save current element position in saved position map
	if ( szPosName )
	{
		SavedPosMap* pMap;
		x_GetMap( pMap, nMap );
		SavedPos savedpos;
		if ( szPosName )
			savedpos.strName = szPosName;
		if ( m_iPosChild )
		{
			savedpos.iPos = m_iPosChild;
			savedpos.nSavedPosFlags |= SavedPos::SPM_CHILD;
		}
		else if ( m_iPos )
		{
			savedpos.iPos = m_iPos;
			savedpos.nSavedPosFlags |= SavedPos::SPM_MAIN;
		}
		else
		{
			savedpos.iPos = m_iPosParent;
		}
		savedpos.nSavedPosFlags |= SavedPos::SPM_USED;

		int nSlot = x_Hash( szPosName, pMap->nMapSize);
		SavedPos* pSavedPos = pMap->pTable[nSlot];
		int nOffset = 0;
		if ( ! pSavedPos )
		{
			pSavedPos = new SavedPos[2];
			pSavedPos[1].nSavedPosFlags = SavedPos::SPM_LAST;
			pMap->pTable[nSlot] = pSavedPos;
		}
		else
		{
			while ( pSavedPos[nOffset].nSavedPosFlags & SavedPos::SPM_USED )
			{
				if ( pSavedPos[nOffset].strName == (MCD_PCSZ)szPosName )
					break;
				if ( pSavedPos[nOffset].nSavedPosFlags & SavedPos::SPM_LAST )
				{
					int nNewSize = (nOffset + 6) * 2;
					SavedPos* pNewSavedPos = new SavedPos[nNewSize];
					for ( int nCopy=0; nCopy<=nOffset; ++nCopy )
						pNewSavedPos[nCopy] = pSavedPos[nCopy];
					pNewSavedPos[nOffset].nSavedPosFlags ^= SavedPos::SPM_LAST;
					pNewSavedPos[nNewSize-1].nSavedPosFlags = SavedPos::SPM_LAST;
					delete [] pSavedPos;
					pSavedPos = pNewSavedPos;
					pMap->pTable[nSlot] = pSavedPos;
					++nOffset;
					break;
				}
				++nOffset;
			}
		}
		if ( pSavedPos[nOffset].nSavedPosFlags & SavedPos::SPM_LAST )
			savedpos.nSavedPosFlags |= SavedPos::SPM_LAST;
		pSavedPos[nOffset] = savedpos;

		/*
		// To review hash table balance, uncomment and watch strBalance
		MCD_STR strBalance, strSlot;
		for ( nSlot=0; nSlot < pMap->nMapSize; ++nSlot )
		{
			pSavedPos = pMap->pTable[nSlot];
			int nCount = 0;
			while ( pSavedPos && pSavedPos->nSavedPosFlags & SavedPos::SPM_USED )
			{
				++nCount;
				if ( pSavedPos->nSavedPosFlags & SavedPos::SPM_LAST )
					break;
				++pSavedPos;
			}
			strSlot.Format( MCD_T("%d "), nCount );
			strBalance += strSlot;
		}
		*/
		return true;
	}
	return false;
}

bool CMarkup::RestorePos( MCD_CSTR szPosName /*=""*/, int nMap /*=0*/ )
{
	// Restore element position if found in saved position map
	if ( szPosName )
	{
		SavedPosMap* pMap;
		x_GetMap( pMap, nMap );
		int nSlot = x_Hash( szPosName, pMap->nMapSize );
		SavedPos* pSavedPos = pMap->pTable[nSlot];
		if ( pSavedPos )
		{
			int nOffset = 0;
			while ( pSavedPos[nOffset].nSavedPosFlags & SavedPos::SPM_USED )
			{
				if ( pSavedPos[nOffset].strName == (MCD_PCSZ)szPosName )
				{
					int i = pSavedPos[nOffset].iPos;
					if ( pSavedPos[nOffset].nSavedPosFlags & SavedPos::SPM_CHILD )
						x_SetPos( m_aPos[m_aPos[i].iElemParent].iElemParent, m_aPos[i].iElemParent, i );
					else if ( pSavedPos[nOffset].nSavedPosFlags & SavedPos::SPM_MAIN )
						x_SetPos( m_aPos[i].iElemParent, i, 0 );
					else
						x_SetPos( i, 0, 0 );
					return true;
				}
				if ( pSavedPos[nOffset].nSavedPosFlags & SavedPos::SPM_LAST )
					break;
				++nOffset;
			}
		}
	}
	return false;
}

bool CMarkup::SetMapSize( int nSize, int nMap /*=0*/ )
{
	// Set saved position map hash table size before using it
	// Returns false if map already exists
	// Some prime numbers: 53, 101, 211, 503, 1009, 2003, 10007, 20011, 50021, 100003, 200003, 500009
	SavedPosMap* pNewMap;
	return x_GetMap( pNewMap, nMap, nSize );
}

bool CMarkup::RemoveElem()
{
	// Remove current main position element
	if ( m_iPos && m_nNodeType == MNT_ELEMENT )
	{
		int iPos = x_RemoveElem( m_iPos );
		x_SetPos( m_iPosParent, iPos, 0 );
		return true;
	}
	return false;
}

bool CMarkup::RemoveChildElem()
{
	// Remove current child position element
	if ( m_iPosChild )
	{
		int iPosChild = x_RemoveElem( m_iPosChild );
		x_SetPos( m_iPosParent, m_iPos, iPosChild );
		return true;
	}
	return false;
}


//////////////////////////////////////////////////////////////////////
// Private Methods
//////////////////////////////////////////////////////////////////////

MCD_STR CMarkup::x_GetLastError()
{
	const int nErrorBufferSize = 100;
	MCD_CHAR szError[nErrorBufferSize];
#if defined(MCD_STRERROR) // C error routine
	MCD_PSZNCPY( szError, MCD_STRERROR, nErrorBufferSize - 1 );
#else // no C error routine
	if ( ::FormatMessage(0x1200,0,::GetLastError(),0,szError,nErrorBufferSize,0) < 1 )
		szError[0] = '\0';
#endif // no C error routine
	MCD_STR strError = szError;
	for ( int nChar=0; nChar<MCD_STRLENGTH(strError); ++nChar )
		if ( strError[nChar] == '\r' || strError[nChar] == '\n' )
		{
			strError = MCD_STRMID( strError, 0, nChar ); // no trailing newline
			break;
		}
	return strError;
}

bool CMarkup::x_AllocPosArray( int nNewSize /*=0*/ )
{
	// Resize m_aPos when the document is created or the array is filled
	// The PosArray class is implemented using segments to reduce contiguous memory requirements
	// It reduces reallocations (copying of memory) since this only occurs within one segment
	// The "Grow By" algorithm ensures there are no reallocations after 2 segments
	//
	if ( ! nNewSize )
		nNewSize = m_iPosFree + (m_iPosFree>>1); // Grow By: multiply size by 1.5
	if ( m_aPos.GetSize() < nNewSize )
	{
		// Grow By: new size can be at most one more complete segment
		int nSeg = (m_aPos.GetSize()?m_aPos.GetSize()-1:0) >> m_aPos.PA_SEGBITS;
		int nNewSeg = (nNewSize-1) >> m_aPos.PA_SEGBITS;
		if ( nNewSeg > nSeg + 1 )
		{
			nNewSeg = nSeg + 1;
			nNewSize = (nNewSeg+1) << m_aPos.PA_SEGBITS;
		}

		// Allocate array of segments
		if ( m_aPos.nSegs <= nNewSeg )
		{
			int nNewSegments = 4 + nNewSeg * 2;
			char* pNewSegments = new char[nNewSegments*sizeof(char*)];
			if ( m_aPos.SegsUsed() )
				memcpy( pNewSegments, m_aPos.pSegs, m_aPos.SegsUsed()*sizeof(char*) );
			if ( m_aPos.pSegs )
				delete[] (char*)m_aPos.pSegs;
			m_aPos.pSegs = (ElemPos**)pNewSegments;
			m_aPos.nSegs = nNewSegments;
		}

		// Calculate segment sizes
		int nSegSize = m_aPos.GetSize() - (nSeg << m_aPos.PA_SEGBITS);
		int nNewSegSize = nNewSize - (nNewSeg << m_aPos.PA_SEGBITS);

		// Complete first segment
		int nFullSegSize = 1 << m_aPos.PA_SEGBITS;
		if ( nSeg < nNewSeg && nSegSize < nFullSegSize )
		{
			char* pNewFirstSeg = new char[ nFullSegSize * sizeof(ElemPos) ];
			if ( nSegSize )
			{
				// Reallocate
				memcpy( pNewFirstSeg, m_aPos.pSegs[nSeg], nSegSize * sizeof(ElemPos) );
				delete[] (char*)m_aPos.pSegs[nSeg];
			}
			m_aPos.pSegs[nSeg] = (ElemPos*)pNewFirstSeg;
		}

		// New segment
		char* pNewSeg = new char[ nNewSegSize * sizeof(ElemPos) ];
		if ( nNewSeg == nSeg && nSegSize )
		{
			// Reallocate
			memcpy( pNewSeg, m_aPos.pSegs[nSeg], nSegSize * sizeof(ElemPos) );
			delete[] (char*)m_aPos.pSegs[nSeg];
		}
		m_aPos.pSegs[nNewSeg] = (ElemPos*)pNewSeg;
		m_aPos.nSize = nNewSize;
	}
	return true;
}

bool CMarkup::x_ParseDoc()
{
	// Preserve pre-parse result
	MCD_STR strResult = m_strError;

	// Reset indexes
	ResetPos();
	m_SavedPosMapArray.RemoveAll();

	// Starting size of position array: 1 element per 64 bytes of document
	// Tight fit when parsing small doc, only 0 to 2 reallocs when parsing large doc
	// Start at 8 when creating new document
	m_iPosFree = 1;
	x_AllocPosArray( MCD_STRLENGTH(m_strDoc) / 64 + 8 );
	m_iPosDeleted = 0;

	// Parse document
	m_aPos[0].ClearVirtualParent();
	if ( MCD_STRLENGTH(m_strDoc) )
	{
		TokenPos token( m_strDoc, m_nDocFlags );
		int iPos = x_ParseElem( 0, token );
		m_aPos[0].nLength = MCD_STRLENGTH(m_strDoc);
		if ( iPos > 0 )
		{
			m_aPos[0].iElemChild = iPos;
			if ( m_aPos[iPos].iElemNext )
				m_strError = MCD_T("Root element has sibling");
		}
		else
			m_strError = MCD_T("No root element");
	}
	else
		m_strError = MCD_T("Empty document");

	ResetPos();

	// Combine preserved result with parse error
	if ( ! MCD_STRISEMPTY(strResult) )
	{
		if ( MCD_STRISEMPTY(m_strError) )
			m_strError = strResult;
		else
			m_strError = strResult + MCD_T(", ") + m_strError;
	}

	return IsWellFormed();
};

int CMarkup::x_ParseElem( int iPosParent, TokenPos& token )
{
	// This is either called by x_ParseDoc or x_AddSubDoc or x_SetElemContent
	// Returns index of the first element encountered or zero if no elements
	//
	int iElemRoot = 0;
	int iPos = iPosParent;
	int iVirtualParent = iPosParent;
	int nRootDepth = m_aPos[iPos].Level();
	token.nNext = 0;
	MCD_STRCLEAR(m_strError);

	// Loop through the nodes of the document
	NodeStack aNodes;
	aNodes.Add();
	int nDepth = 0;
	int nMatchDepth;
	int iPosChild;
	int iPosMatch;
	int nTypeFound = 0;
	ElemPos* pElem;
	int iElemFirst, iElemLast;
	while ( 1 )
	{
		nTypeFound = x_ParseNode( token, aNodes.Top() );
		nMatchDepth = 0;
		if ( nTypeFound == MNT_ELEMENT ) // start tag
		{
			iPos = x_GetFreePos();
			if ( ! iElemRoot )
				iElemRoot = iPos;
			pElem = &m_aPos[iPos];
			pElem->iElemParent = iPosParent;
			pElem->iElemNext = 0;
			if ( m_aPos[iPosParent].iElemChild )
			{
				iElemFirst = m_aPos[iPosParent].iElemChild;
				iElemLast = m_aPos[iElemFirst].iElemPrev;
				m_aPos[iElemLast].iElemNext = iPos;
				pElem->iElemPrev = iElemLast;
				m_aPos[iElemFirst].iElemPrev = iPos;
				pElem->nFlags = 0;
			}
			else
			{
				m_aPos[iPosParent].iElemChild = iPos;
				pElem->iElemPrev = iPos;
				pElem->nFlags = MNF_FIRST;
			}
			pElem->SetLevel( nRootDepth + nDepth );
			pElem->iElemChild = 0;
			pElem->nStart = aNodes.Top().nStart;
			pElem->SetStartTagLen( aNodes.Top().nLength );
			if ( aNodes.Top().nNodeFlags & MNF_EMPTY )
			{
				iPos = iPosParent;
				pElem->SetEndTagLen( 0 );
				pElem->nLength = aNodes.Top().nLength;
			}
			else
			{
				iPosParent = iPos;
				++nDepth;
				aNodes.Add();
			}
		}
		else if ( nTypeFound == 0 ) // end tag
		{
			nMatchDepth = nDepth;
			iPosMatch = iPos;
			while ( nMatchDepth && ! token.Match(aNodes.At(nMatchDepth-1).strMeta) )
			{
				/*
				// Auto-switch case sensitivity
				if ( ! (token.nTokenFlags & MDF_IGNORECASE ) )
				{
					token.nTokenFlags |= MDF_IGNORECASE;
					if ( token.Match(aNodes.At(nMatchDepth-1).strMeta) )
						break;
					token.nTokenFlags |= MDF_IGNORECASE;
				}
				*/
				--nMatchDepth;
				iPosMatch = m_aPos[iPosMatch].iElemParent;
			}
			if ( nMatchDepth == 0 )
			{
				// Not matched at all, it is a lone end tag, a non-element node
				m_aPos[iVirtualParent].nFlags |= MNF_ILLFORMED;
				m_aPos[iPos].nFlags |= MNF_ILLDATA;
				if ( MCD_STRISEMPTY(m_strError) )
				{
					MCD_CHAR szError[100];
					MCD_SPRINTF( MCD_SSZ(szError), MCD_T("No start tag for end tag '%.50s' at offset %d"),
						MCD_2PCSZ(x_GetToken(token)), aNodes.Top().nStart );
					m_strError = szError;
				}
			}
			else
			{
				pElem = &m_aPos[iPosMatch];
				pElem->nLength = aNodes.Top().nStart - pElem->nStart + aNodes.Top().nLength;
				pElem->SetEndTagLen( aNodes.Top().nLength );
			}
		}
		else if ( nTypeFound == -1 )
		{
			m_aPos[iVirtualParent].nFlags |= MNF_ILLFORMED;
			m_aPos[iPos].nFlags |= MNF_ILLDATA;
			if ( MCD_STRISEMPTY(m_strError) )
				m_strError = aNodes.Top().strMeta;
		}

		// Matched end tag, or end of document
		if ( nMatchDepth || nTypeFound == -2 )
		{
			if ( nDepth > nMatchDepth )
				m_aPos[iVirtualParent].nFlags |= MNF_ILLFORMED;

			// Process any non-ended elements
			while ( nDepth > nMatchDepth )
			{
				// Element with no end tag
				pElem = &m_aPos[iPos];
				iPosChild = pElem->iElemChild;
				iPosParent = pElem->iElemParent;
				pElem->SetEndTagLen( 0 );
				pElem->nFlags |= MNF_NONENDED;
				pElem->iElemChild = 0;
				pElem->nLength = pElem->StartTagLen();
				if ( pElem->nFlags & MNF_ILLDATA )
				{
					pElem->nFlags ^= MNF_ILLDATA;
					m_aPos[iPosParent].nFlags |= MNF_ILLDATA;
				}
				while ( iPosChild )
				{
					m_aPos[iPosChild].iElemParent = iPosParent;
					m_aPos[iPosChild].iElemPrev = iPos;
					m_aPos[iPos].iElemNext = iPosChild;
					iPos = iPosChild;
					iPosChild = m_aPos[iPosChild].iElemNext;
				}
				iPos = iPosParent;
				aNodes.Remove();
				--nDepth;

				// Error string
				// if end tag did not match, top node is end tag that did not match pElem
				// if end of document, any nodes below top have no end tag
				if ( MCD_STRISEMPTY(m_strError) )
				{
					if ( nTypeFound == 0 )
					{
						MCD_CHAR szError[200];
						MCD_SPRINTF( MCD_SSZ(szError), MCD_T("End tag '%.50s' at offset %d does not match start tag '.50%s' at offset %d"),
							MCD_2PCSZ(x_GetToken(token)), token.nL-1, MCD_2PCSZ(aNodes.Top().strMeta), pElem->nStart );
						m_strError = szError;
					}
					else
					{
						MCD_CHAR szError[100];
						MCD_SPRINTF( MCD_SSZ(szError), MCD_T("Element '%.50s' at offset %d not ended"),
							MCD_2PCSZ(aNodes.Top().strMeta), aNodes.Top().nStart );
						m_strError = szError;
					}
				}
			}
			if ( nTypeFound == -2 )
				break;
			iPosParent = m_aPos[iPos].iElemParent;
			iPos = iPosParent;
			aNodes.Remove();
			--nDepth;
		}
	}
	return iElemRoot;
}

bool CMarkup::x_FindAny( MCD_PCSZ pDoc, int& nChar )
{
	// Starting at nChar, find a non-whitespace char
	// return false if no non-whitespace before end of document, nChar points to end
	// otherwise return true and nChar points to non-whitespace char
	while ( pDoc[nChar] && MCD_PSZCHR(MCD_T(" \t\n\r"),pDoc[nChar]) )
		++nChar;
	return pDoc[nChar] != '\0';
}

bool CMarkup::x_FindName( CMarkup::TokenPos& token )
{
	// Starting at token.nNext, bypass whitespace and find the next name
	// returns true on success, members of token point to token
	// returns false on end of document, members point to end of document
	MCD_PCSZ pDoc = token.pDoc;
	int nChar = token.nNext;

	// By-pass leading whitespace
	if ( ! x_FindAny(pDoc,nChar) )
	{
		// No token was found before end of document
		token.nL = nChar;
		token.nR = nChar - 1;
		token.nNext = nChar;
		return false;
	}

	// Go until special char or whitespace
	token.nL = nChar;
	while ( pDoc[nChar] && ! MCD_PSZCHR(MCD_T(" \t\n\r<>=\\/?!"),pDoc[nChar]) )
		nChar += MCD_CLEN(&pDoc[nChar]);

	// Adjust end position if it is one special char
	if ( nChar == token.nL )
		++nChar; // it is a special char
	token.nR = nChar - 1;

	// nNext points to one past last char of token
	token.nNext = nChar;
	return true;
}

MCD_STR CMarkup::x_GetToken( const CMarkup::TokenPos& token )
{
	// The token contains indexes into the document identifying a small substring
	// Build the substring from those indexes and return it
	if ( token.nL > token.nR )
		return MCD_T("");
	MCD_STR strToken( &token.pDoc[token.nL], token.Length() );
	return strToken;
}

int CMarkup::x_FindElem( int iPosParent, int iPos, MCD_PCSZ pPath ) const
{
	// If pPath is NULL or empty, go to next sibling element
	// Otherwise go to next sibling element with matching path
	//
	if ( iPos )
		iPos = m_aPos[iPos].iElemNext;
	else
		iPos = m_aPos[iPosParent].iElemChild;

	// Finished here if pPath not specified
	if ( pPath == NULL || !pPath[0] )
		return iPos;

	// Search
	TokenPos token( m_strDoc, m_nDocFlags );
	while ( iPos )
	{
		// Compare tag name
		token.nNext = m_aPos[iPos].nStart + 1;
		x_FindName( token ); // Locate tag name
		if ( token.Match(pPath) )
			return iPos;
		iPos = m_aPos[iPos].iElemNext;
	}
	return 0;

}

int CMarkup::x_ParseNode( CMarkup::TokenPos& token, CMarkup::NodePos& node )
{
	// Call this with token.nNext set to the start of the node or tag
	// Upon return token.nNext points to the char after the node or tag
	// 
	// <!--...--> comment
	// <!DOCTYPE ...> dtd
	// <?target ...?> processing instruction
	// <![CDATA[...]]> cdata section
	// <NAME ...> element start tag
	// </NAME ...> element end tag
	//
	// returns the nodetype or
	// 0 for end tag
	// -1 for bad node
	// -2 for end of document
	//
	enum ParseBits
	{
		PD_OPENTAG = 1,
		PD_BANG = 2,
		PD_DASH = 4,
		PD_BRACKET = 8,
		PD_TEXTORWS = 16,
		PD_DOCTYPE = 32,
		PD_INQUOTE_S = 64,
		PD_INQUOTE_D = 128,
		PD_EQUALS = 256,
	};
	int nParseFlags = 0;

	MCD_PCSZ pFindEnd = NULL;
	int nNodeType = -1;
	int nEndLen = 0;
	int nName = 0;
	unsigned int cDminus1 = 0, cDminus2 = 0;
	#define FINDNODETYPE(e,t,n) { pFindEnd=e; nEndLen=(sizeof(e)-1)/sizeof(MCD_CHAR); nNodeType=t; if(n) nName=(int)(pDoc-token.pDoc)+n-1; }
	#define FINDNODEBAD(e) { pFindEnd=MCD_T(">"); nEndLen=1; MCD_CHAR szE[100]; MCD_SPRINTF(MCD_SSZ(szE),MCD_T("Incorrect %s at offset %d"),e,nR); node.strMeta=szE; nNodeType=-1; }

	node.nStart = token.nNext;
	node.nNodeFlags = 0;

	int nR = token.nNext;
	MCD_PCSZ pDoc = &token.pDoc[nR];
	register unsigned int cD = (unsigned int)*pDoc;
	if ( ! cD )
	{
		node.nLength = 0;
		node.nNodeType = 0;
		return -2; // end of document
	}

	while ( 1 )
	{
		cD = (unsigned int)*pDoc;
		if ( ! cD )
		{
			nR = (int)(pDoc - token.pDoc) - 1;
			if ( nNodeType != MNT_WHITESPACE && nNodeType != MNT_TEXT )
			{
				MCD_PCSZ pType = MCD_T("tag");
				if ( (nParseFlags & PD_DOCTYPE) || nNodeType == MNT_DOCUMENT_TYPE )
					pType = MCD_T("Doctype");
				else if ( nNodeType == MNT_ELEMENT )
					pType = MCD_T("Element tag");
				else if ( nNodeType == 0 )
					pType = MCD_T("Element end tag");
				else if ( nNodeType == MNT_CDATA_SECTION )
					pType = MCD_T("CDATA Section");
				else if ( nNodeType == MNT_PROCESSING_INSTRUCTION )
					pType = MCD_T("Processing instruction");
				else if ( nNodeType == MNT_COMMENT )
					pType = MCD_T("Comment");
				nNodeType = -1;
				MCD_CHAR szError[100];
				MCD_SPRINTF( MCD_SSZ(szError), MCD_T("%s at offset %d unterminated"), pType, node.nStart );
				node.strMeta = szError;
			}
			break;
		}

		if ( nName )
		{
			if ( MCD_PSZCHR(MCD_T(" \t\n\r/>"),(MCD_CHAR)cD) )
			{
				int nNameLen = (int)(pDoc - token.pDoc) - nName;
				if ( nNodeType == 0 )
				{
					token.nL = nName;
					token.nR = nName + nNameLen - 1;
				}
				else
				{
					MCD_STRASSIGN(node.strMeta,&token.pDoc[nName],nNameLen);
				}
				nName = 0;
				cDminus2 = 0;
				cDminus1 = 0;
			}
			else
			{
				pDoc += MCD_CLEN( pDoc );
				continue;
			}
		}

		if ( pFindEnd )
		{
			if ( cD == '>' && ! (nParseFlags & (PD_INQUOTE_S|PD_INQUOTE_D)) )
			{
				nR = (int)(pDoc - token.pDoc);
				if ( nEndLen == 1 )
				{
					pFindEnd = NULL;
					if ( nNodeType == MNT_ELEMENT && cDminus1 == '/' )
					{
						if ( (! cDminus2) || MCD_PSZCHR(MCD_T(" \t\n\r\'\""),(MCD_CHAR)cDminus2) )
							node.nNodeFlags |= MNF_EMPTY;
					}
				}
				else if ( nR > nEndLen )
				{
					// Test for end of PI or comment
					MCD_PCSZ pEnd = pDoc - nEndLen + 1;
					MCD_PCSZ pInFindEnd = pFindEnd;
					int nLen = nEndLen;
					while ( --nLen && *pEnd++ == *pInFindEnd++ );
					if ( nLen == 0 )
						pFindEnd = NULL;
				}
				if ( ! pFindEnd && ! (nParseFlags & PD_DOCTYPE) )
					break;
			}
			else if ( cD == '<' && (nNodeType == MNT_TEXT || nNodeType == -1) )
			{
				nR = (int)(pDoc - token.pDoc) - 1;
				break;
			}
			else if ( nNodeType & MNT_ELEMENT )
			{
				if ( (nParseFlags & (PD_INQUOTE_S|PD_INQUOTE_D)) )
				{
					if ( cD == '\"' && (nParseFlags&PD_INQUOTE_D) )
						nParseFlags ^= PD_INQUOTE_D; // off
					else if ( cD == '\'' && (nParseFlags&PD_INQUOTE_S) )
						nParseFlags ^= PD_INQUOTE_S; // off
				}
				else // not in quotes
				{
					// Only set INQUOTE status when preceeded by equal sign
					if ( cD == '\"' && (nParseFlags&PD_EQUALS) )
						nParseFlags ^= PD_INQUOTE_D|PD_EQUALS; // D on, equals off
					else if ( cD == '\'' && (nParseFlags&PD_EQUALS) )
						nParseFlags ^= PD_INQUOTE_S|PD_EQUALS; // S on, equals off
					else if ( cD == '=' && cDminus1 != '=' && ! (nParseFlags&PD_EQUALS) )
						nParseFlags ^= PD_EQUALS; // on
					else if ( (nParseFlags&PD_EQUALS) && ! MCD_PSZCHR(MCD_T(" \t\n\r"),(MCD_CHAR)cD) )
						nParseFlags ^= PD_EQUALS; // off
				}
				cDminus2 = cDminus1;
				cDminus1 = cD;
			}
			else if ( nNodeType & MNT_DOCUMENT_TYPE )
			{
				if ( cD == '\"' && ! (nParseFlags&PD_INQUOTE_S) )
					nParseFlags ^= PD_INQUOTE_D; // toggle
				else if ( cD == '\'' && ! (nParseFlags&PD_INQUOTE_D) )
					nParseFlags ^= PD_INQUOTE_S; // toggle
			}
		}
		else if ( nParseFlags )
		{
			if ( nParseFlags & PD_TEXTORWS )
			{
				if ( cD == '<' )
				{
					nR = (int)(pDoc - token.pDoc) - 1;
					nNodeType = MNT_WHITESPACE;
					break;
				}
				else if ( ! MCD_PSZCHR(MCD_T(" \t\n\r"),(MCD_CHAR)cD) )
				{
					nParseFlags ^= PD_TEXTORWS;
					FINDNODETYPE( MCD_T("<"), MNT_TEXT, 0 )
				}
			}
			else if ( nParseFlags & PD_OPENTAG )
			{
				nParseFlags ^= PD_OPENTAG;
				if ( cD > 0x60 || ( cD > 0x40 && cD < 0x5b ) || cD == 0x5f || cD == 0x3a )
					FINDNODETYPE( MCD_T(">"), MNT_ELEMENT, 1 )
				else if ( cD == '/' )
					FINDNODETYPE( MCD_T(">"), 0, 2 )
				else if ( cD == '!' )
					nParseFlags |= PD_BANG;
				else if ( cD == '?' )
					FINDNODETYPE( MCD_T("?>"), MNT_PROCESSING_INSTRUCTION, 2 )
				else
					FINDNODEBAD( MCD_T("tag name character") )
			}
			else if ( nParseFlags & PD_BANG )
			{
				nParseFlags ^= PD_BANG;
				if ( cD == '-' )
					nParseFlags |= PD_DASH;
				else if ( cD == '[' && !(nParseFlags & PD_DOCTYPE) )
					nParseFlags |= PD_BRACKET;
				else if ( cD == 'D' && !(nParseFlags & PD_DOCTYPE) )
					nParseFlags |= PD_DOCTYPE;
				else if ( MCD_PSZCHR(MCD_T("EAN"),(MCD_CHAR)cD) ) // <!ELEMENT ATTLIST ENTITY NOTATION
					FINDNODETYPE( MCD_T(">"), MNT_DOCUMENT_TYPE, 0 )
				else
					FINDNODEBAD( MCD_T("! tag") )
			}
			else if ( nParseFlags & PD_DASH )
			{
				nParseFlags ^= PD_DASH;
				if ( cD == '-' )
					FINDNODETYPE( MCD_T("-->"), MNT_COMMENT, 0 )
				else
					FINDNODEBAD( MCD_T("comment tag") )
			}
			else if ( nParseFlags & PD_BRACKET )
			{
				nParseFlags ^= PD_BRACKET;
				if ( cD == 'C' )
					FINDNODETYPE( MCD_T("]]>"), MNT_CDATA_SECTION, 0 )
				else
					FINDNODEBAD( MCD_T("tag") )
			}
			else if ( nParseFlags & PD_DOCTYPE )
			{
				if ( cD == '<' )
					nParseFlags |= PD_OPENTAG;
				else if ( cD == '>' )
				{
					nR = (int)(pDoc - token.pDoc);
					nNodeType = MNT_DOCUMENT_TYPE;
					break;
				}
			}
		}
		else if ( cD == '<' )
		{
			nParseFlags |= PD_OPENTAG;
		}
		else
		{
			nNodeType = MNT_WHITESPACE;
			if ( MCD_PSZCHR(MCD_T(" \t\n\r"),(MCD_CHAR)cD) )
				nParseFlags |= PD_TEXTORWS;
			else
				FINDNODETYPE( MCD_T("<"), MNT_TEXT, 0 )
		}
		pDoc += MCD_CLEN( pDoc );
	}
	token.nNext = nR + 1;
	node.nLength = token.nNext - node.nStart;
	node.nNodeType = nNodeType;
	return nNodeType;
}

MCD_STR CMarkup::x_GetPath( int iPos ) const
{
	MCD_STR strPath;
	while ( iPos )
	{
		MCD_STR strTagName = x_GetTagName( iPos );
		int iPosParent = m_aPos[iPos].iElemParent;
		int iPosSib = 0;
		int nCount = 0;
		while ( iPosSib != iPos )
		{
			iPosSib = x_FindElem( iPosParent, iPosSib, MCD_2PCSZ(strTagName) );
			++nCount;
		}
		if ( nCount > 1 )
		{
			MCD_CHAR szPred[25];
			MCD_SPRINTF( MCD_SSZ(szPred), MCD_T("[%d]"), nCount );
			strPath = MCD_T("/") + strTagName + szPred + strPath;
		}
		else
			strPath = MCD_T("/") + strTagName + strPath;
		iPos = iPosParent;
	}
	return strPath;
}

MCD_STR CMarkup::x_GetTagName( int iPos ) const
{
	// Return the tag name at specified element
	TokenPos token( m_strDoc, m_nDocFlags );
	token.nNext = m_aPos[iPos].nStart + 1;
	if ( ! iPos || ! x_FindName( token ) )
		return MCD_T("");

	// Return substring of document
	return x_GetToken( token );
}

bool CMarkup::x_FindAttrib( CMarkup::TokenPos& token, MCD_PCSZ pAttrib, int n/*=0*/ )
{
	// Return true if found, otherwise false and token.nNext is new insertion point
	// If pAttrib is NULL find attrib n and leave token at attrib name
	// If pAttrib is given, find matching attrib and leave token at value
	// support non-well-formed attributes e.g. href=/advanced_search?hl=en, nowrap
	// token also holds start and length of preceeding whitespace to support remove
	//
	int nPreSpaceStart;
	int nPreSpaceLength;
	int nChar;
	MCD_CHAR cFirstChar;
	MCD_PCSZ pDoc = token.pDoc;
	int nAttrib = -1; // starts at tag name
	int nFoundAttribNameR = 0;
	bool bAfterEqual = false;
	while ( 1 )
	{
		// Starting at token.nNext, bypass whitespace and find the next token
		nChar = token.nNext;
		nPreSpaceStart = nChar;
		if ( ! x_FindAny(pDoc,nChar) )
			break;
		nPreSpaceLength = nChar - nPreSpaceStart;

		// Is it an opening quote?
		cFirstChar = pDoc[nChar];
		if ( cFirstChar == '\"' || cFirstChar == '\'' )
		{
			token.nTokenFlags |= MNF_QUOTED;

			// Move past opening quote
			++nChar;
			token.nL = nChar;

			// Look for closing quote
			while ( pDoc[nChar] && pDoc[nChar] != cFirstChar )
				nChar += MCD_CLEN( &pDoc[nChar] );

			// Set right to before closing quote
			token.nR = nChar - 1;

			// Set nChar past closing quote unless at end of document
			if ( pDoc[nChar] )
				++nChar;
		}
		else
		{
			token.nTokenFlags &= ~MNF_QUOTED;

			// Go until special char or whitespace
			token.nL = nChar;
			if ( bAfterEqual )
			{
				while ( pDoc[nChar] && ! MCD_PSZCHR(MCD_T(" \t\n\r>"),pDoc[nChar]) )
					nChar += MCD_CLEN( &pDoc[nChar] );
			}
			else
			{
				while ( pDoc[nChar] && ! MCD_PSZCHR(MCD_T("= \t\n\r>/?"),pDoc[nChar]) )
					nChar += MCD_CLEN( &pDoc[nChar] );
			}

			// Adjust end position if it is one special char
			if ( nChar == token.nL )
				++nChar; // it is a special char
			token.nR = nChar - 1;
		}

		// nNext points to one past last char of token
		token.nNext = nChar;

		if ( ! bAfterEqual && ! (token.nTokenFlags&MNF_QUOTED) )
		{
			// Is it an equal sign?
			MCD_CHAR cChar = pDoc[token.nL];
			if ( cChar == '=' )
			{
				bAfterEqual = true;
				continue;
			}

			// Is it the right angle bracket?
			if ( cChar == '>' || cChar == '/' || cChar == '?' )
			{
				token.nNext = nPreSpaceStart;
				break; // attrib not found
			}

			if ( nFoundAttribNameR )
				break;

			// Attribute name
			if ( nAttrib != -1 )
			{
				if ( ! pAttrib )
				{
					if ( nAttrib == n )
						return true; // found by number
				}
				else if ( token.Match(pAttrib) )
				{
					// Matched attrib name, go forward to value
					nFoundAttribNameR = token.nR;
					token.nPreSpaceStart = nPreSpaceStart;
					token.nPreSpaceLength = nPreSpaceLength;
				}
			}
			++nAttrib;
		}
		else if ( nFoundAttribNameR )
			break;
		bAfterEqual = false;
	}

	if ( nFoundAttribNameR )
	{
		if ( ! bAfterEqual )
		{
			// when attribute has no value the value is the attribute name
			token.nL = token.nPreSpaceStart + token.nPreSpaceLength;
			token.nR = nFoundAttribNameR;
			token.nNext = nFoundAttribNameR + 1;
		}
		return true; // found by name
	}
	return false; // not found
}

MCD_STR CMarkup::x_GetAttrib( int iPos, MCD_PCSZ pAttrib ) const
{
	// Return the value of the attrib
	TokenPos token( m_strDoc, m_nDocFlags );
	if ( iPos && m_nNodeType == MNT_ELEMENT )
		token.nNext = m_aPos[iPos].nStart + 1;
	else if ( iPos == m_iPos && m_nNodeLength && m_nNodeType == MNT_PROCESSING_INSTRUCTION )
		token.nNext = m_nNodeOffset + 2;
	else
		return MCD_T("");

	if ( pAttrib && x_FindAttrib( token, pAttrib ) )
		return UnescapeText( &token.pDoc[token.nL], token.Length() );
	return MCD_T("");
}

bool CMarkup::x_SetAttrib( int iPos, MCD_PCSZ pAttrib, int nValue, int nFlags /*=0*/ )
{
	// Convert integer to string
	MCD_CHAR szVal[25];
	MCD_SPRINTF( MCD_SSZ(szVal), MCD_T("%d"), nValue );
	return x_SetAttrib( iPos, pAttrib, szVal, nFlags );
}

bool CMarkup::x_SetAttrib( int iPos, MCD_PCSZ pAttrib, MCD_PCSZ pValue, int nFlags /*=0*/ )
{
	// Set attribute in iPos element
	TokenPos token( m_strDoc, m_nDocFlags );
	if ( iPos && m_nNodeType == MNT_ELEMENT )
		token.nNext = m_aPos[iPos].nStart + 1;
	else if ( iPos == m_iPos && m_nNodeLength && m_nNodeType == MNT_PROCESSING_INSTRUCTION )
		token.nNext = m_nNodeOffset + 2;
	else
		return false;

	// Create insertion text depending on whether attribute already exists
	// Decision: for empty value leaving attrib="" instead of removing attrib
	int nReplace = 0;
	int nInsertAt;
	MCD_STR strInsert;
	strInsert += x_ATTRIBQUOTE;
	strInsert += EscapeText( pValue, MNF_ESCAPEQUOTES|nFlags );
	strInsert += x_ATTRIBQUOTE;
	if ( x_FindAttrib( token, pAttrib ) )
	{
		// Replace value
		nInsertAt = token.nL - ((token.nTokenFlags&MNF_QUOTED)?1:0);
		nReplace = token.Length() + ((token.nTokenFlags&MNF_QUOTED)?2:0);
	}
	else
	{
		// Insert string name value pair
		MCD_STR strFormat;
		strFormat = MCD_T(" ");
		strFormat += pAttrib;
		strFormat += MCD_T("=");
		strFormat += strInsert;
		strInsert = strFormat;
		nInsertAt = token.nNext;
	}

	x_DocChange( nInsertAt, nReplace, strInsert );
	int nAdjust = MCD_STRLENGTH(strInsert) - nReplace;
	if ( m_nNodeType == MNT_PROCESSING_INSTRUCTION )
	{
		x_AdjustForNode( m_iPosParent, m_iPos, nAdjust );
		m_nNodeLength += nAdjust;
		MARKUP_SETDEBUGSTATE;
		return true;
	}
	m_aPos[iPos].AdjustStartTagLen( nAdjust );
	m_aPos[iPos].nLength += nAdjust;
	x_Adjust( iPos, nAdjust );
	MARKUP_SETDEBUGSTATE;
	return true;
}


bool CMarkup::x_CreateNode( MCD_STR& strNode, int nNodeType, MCD_PCSZ pText )
{
	// Set strNode based on nNodeType and szData
	// Return false if szData would jeopardize well-formed document
	//
	switch ( nNodeType )
	{
	case MNT_PROCESSING_INSTRUCTION:
		strNode = MCD_T("<?");
		strNode += pText;
		strNode += MCD_T("?>");
		break;
	case MNT_COMMENT:
		strNode = MCD_T("<!--");
		strNode += pText;
		strNode += MCD_T("-->");
		break;
	case MNT_ELEMENT:
		strNode = MCD_T("<");
		strNode += pText;
		strNode += MCD_T("/>");
		break;
	case MNT_TEXT:
	case MNT_WHITESPACE:
		strNode = EscapeText( pText );
		break;
	case MNT_DOCUMENT_TYPE:
		strNode = pText;
		break;
	case MNT_LONE_END_TAG:
		return false;
	case MNT_CDATA_SECTION:
		if ( MCD_PSZSTR(pText,MCD_T("]]>")) != NULL )
			return false;
		strNode = MCD_T("<![CDATA[");
		strNode += pText;
		strNode += MCD_T("]]>");
		break;
	}
	return true;
}

MCD_STR CMarkup::x_EncodeCDATASection( MCD_PCSZ szData )
{
	// Split CDATA Sections if there are any end delimiters
	MCD_STR strData = MCD_T("<![CDATA[");
	MCD_PCSZ pszNextStart = szData;
	MCD_PCSZ pszEnd = MCD_PSZSTR( szData, MCD_T("]]>") );
	while ( pszEnd )
	{
		strData += MCD_STR( pszNextStart, (int)(pszEnd - pszNextStart) );
		strData += MCD_T("]]]]><![CDATA[>");
		pszNextStart = pszEnd + 3;
		pszEnd = MCD_PSZSTR( pszNextStart, MCD_T("]]>") );
	}
	strData += pszNextStart;
	strData += MCD_T("]]>");
	return strData;
}

bool CMarkup::x_SetData( int iPos, int nValue )
{
	// Convert integer to string
	MCD_CHAR szVal[25];
	MCD_SPRINTF( MCD_SSZ(szVal), MCD_T("%d"), nValue );
	return x_SetData( iPos, szVal, 0 );
}

bool CMarkup::x_SetData( int iPos, MCD_PCSZ szData, int nFlags )
{
	// Set data at specified position
	// if nFlags==1, set content of element to a CDATA Section
	MCD_STR strInsert;

	if ( iPos == m_iPos && m_nNodeLength )
	{
		// Not an element
		if ( ! x_CreateNode(strInsert, m_nNodeType, szData) )
			return false;
		x_DocChange( m_nNodeOffset, m_nNodeLength, strInsert );
		x_AdjustForNode( m_iPosParent, iPos, MCD_STRLENGTH(strInsert) - m_nNodeLength );
		m_nNodeLength = MCD_STRLENGTH(strInsert);
		MARKUP_SETDEBUGSTATE;
		return true;
	}

	// Set data in iPos element
	if ( ! iPos || m_aPos[iPos].iElemChild )
		return false;

	// Build strInsert from szData based on nFlags
	if ( nFlags & MNF_WITHCDATA )
		strInsert = x_EncodeCDATASection( szData );
	else
		strInsert = EscapeText( szData, nFlags );

	// Insert
	NodePos node( MNF_WITHNOLINES|MNF_REPLACE );
	node.strMeta = strInsert;
	int iPosBefore = 0;
	int nReplace = x_InsertNew( iPos, iPosBefore, node );
	int nAdjust = MCD_STRLENGTH(node.strMeta) - nReplace;
	x_Adjust( iPos, nAdjust );
	m_aPos[iPos].nLength += nAdjust;
	if ( m_aPos[iPos].nFlags & MNF_ILLDATA )
		m_aPos[iPos].nFlags &= ~MNF_ILLDATA;
	MARKUP_SETDEBUGSTATE;
	return true;
}

MCD_STR CMarkup::x_GetData( int iPos ) const
{
	if ( iPos == m_iPos && m_nNodeLength )
	{
		if ( m_nNodeType == MNT_COMMENT )
			return MCD_STRMID( m_strDoc, m_nNodeOffset+4, m_nNodeLength-7 );
		else if ( m_nNodeType == MNT_PROCESSING_INSTRUCTION )
			return MCD_STRMID( m_strDoc, m_nNodeOffset+2, m_nNodeLength-4 );
		else if ( m_nNodeType == MNT_CDATA_SECTION )
			return MCD_STRMID( m_strDoc, m_nNodeOffset+9, m_nNodeLength-12 );
		else if ( m_nNodeType == MNT_TEXT )
			return UnescapeText( &(MCD_2PCSZ(m_strDoc))[m_nNodeOffset], m_nNodeLength );
		else if ( m_nNodeType == MNT_LONE_END_TAG )
			return MCD_STRMID( m_strDoc, m_nNodeOffset+2, m_nNodeLength-3 );
		else
			return MCD_STRMID( m_strDoc, m_nNodeOffset, m_nNodeLength );
	}

	// Return a string representing data between start and end tag
	// Return empty string if there are any children elements
	MCD_STR strData;
	if ( ! m_aPos[iPos].iElemChild && ! m_aPos[iPos].IsEmptyElement() )
	{
		// Quick scan for any tags inside content
		int nContentLen = m_aPos[iPos].ContentLen();
		int nStartContent = m_aPos[iPos].StartContent();
		MCD_PCSZ pszContent = &(MCD_2PCSZ(m_strDoc))[nStartContent];
		MCD_PCSZ pszTag = MCD_PSZCHR( pszContent, '<' );
		if ( pszTag && ((int)(pszTag-pszContent) < nContentLen) )
		{
			// Concatenate all CDATA Sections and text nodes, ignore other nodes
			TokenPos token( m_strDoc, m_nDocFlags );
			token.nNext = nStartContent;
			NodePos node;
			while ( token.nNext < nStartContent + nContentLen )
			{
				x_ParseNode( token, node );
				if ( node.nNodeType == MNT_TEXT )
					strData += UnescapeText( &token.pDoc[node.nStart], node.nLength );
				else if ( node.nNodeType == MNT_CDATA_SECTION )
					strData += MCD_STRMID( m_strDoc, node.nStart+9, node.nLength-12 );
			}
		}
		else // no tags
			strData = UnescapeText( &(MCD_2PCSZ(m_strDoc))[nStartContent], nContentLen );
	}
	return strData;
}

MCD_STR CMarkup::x_GetElemContent( int iPos ) const
{
	if ( iPos && m_aPos[iPos].ContentLen() )
		return MCD_STRMID( m_strDoc, m_aPos[iPos].StartContent(), m_aPos[iPos].ContentLen() );
	return MCD_T("");
}

bool CMarkup::x_SetElemContent( MCD_PCSZ szContent )
{
	// Set data in iPos element only
	if ( ! m_iPos )
		return false;

	if ( m_nNodeLength )
		return false; // not an element

	// Unlink all children
	int iPos = m_iPos;
	int iPosChild = m_aPos[iPos].iElemChild;
	bool bHadChild = (iPosChild != 0);
	while ( iPosChild )
		iPosChild = x_ReleaseSubDoc( iPosChild );
	if ( bHadChild )
		x_CheckSavedPos();

	// Parse content
	bool bWellFormed = true;
	TokenPos token( szContent, m_nDocFlags );
	int iPosVirtual = x_GetFreePos();
	m_aPos[iPosVirtual].ClearVirtualParent();
	m_aPos[iPosVirtual].SetLevel( m_aPos[iPos].Level() + 1 );
	iPosChild = x_ParseElem( iPosVirtual, token );
	if ( m_aPos[iPosVirtual].nFlags & MNF_ILLFORMED )
		bWellFormed = false;
	m_aPos[iPos].nFlags = (m_aPos[iPos].nFlags & ~MNF_ILLDATA) | (m_aPos[iPosVirtual].nFlags & MNF_ILLDATA);

	// Prepare insert and adjust offsets
	NodePos node( MNF_WITHNOLINES|MNF_REPLACE );
	node.strMeta = szContent;
	int iPosBefore = 0;
	int nReplace = x_InsertNew( iPos, iPosBefore, node );
	
	// Adjust and link in the inserted elements
	x_Adjust( iPosChild, node.nStart );
	m_aPos[iPosChild].nStart += node.nStart;
	m_aPos[iPos].iElemChild = iPosChild;
	while ( iPosChild )
	{
		m_aPos[iPosChild].iElemParent = iPos;
		iPosChild = m_aPos[iPosChild].iElemNext;
	}
	x_ReleasePos( iPosVirtual );

	int nAdjust = MCD_STRLENGTH(node.strMeta) - nReplace;
	x_Adjust( iPos, nAdjust, true );
	m_aPos[iPos].nLength += nAdjust;

	x_SetPos( m_iPosParent, m_iPos, 0 );
	return bWellFormed;
}

void CMarkup::x_DocChange( int nLeft, int nReplace, const MCD_STR& strInsert )
{
	// Insert strInsert int m_strDoc at nLeft replacing nReplace chars
	// When creating a document, reduce reallocs by reserving string space
	// If realloc needed, allow for 1.5 times the new length
	//
	int nDocLength = MCD_STRLENGTH(m_strDoc);
	int nInsLength = MCD_STRLENGTH(strInsert);
	int nNewLength = nInsLength + nDocLength - nReplace;
	int nAllocLen = MCD_STRCAPACITY(m_strDoc);
#if defined(MCD_STRINSERTREPLACE) // STL, replace method
	if ( nNewLength > nAllocLen )
		MCD_BLDRESERVE( m_strDoc, (nNewLength + nNewLength/2 + 128) );
	MCD_STRINSERTREPLACE( m_strDoc, nLeft, nReplace, strInsert );
#else // MFC, no replace method
	int nBufferLen = nNewLength;
	if ( nNewLength > nAllocLen )
		nBufferLen += nBufferLen/2 + 128;
	MCD_CHAR* pDoc = MCD_GETBUFFER( m_strDoc, nBufferLen );
	if ( nInsLength != nReplace && nLeft+nReplace < nDocLength )
		memmove( &pDoc[nLeft+nInsLength], &pDoc[nLeft+nReplace], (nDocLength-nLeft-nReplace)*sizeof(MCD_CHAR) );
	memcpy( &pDoc[nLeft], strInsert, nInsLength*sizeof(MCD_CHAR) );
	MCD_RELEASEBUFFER( m_strDoc, pDoc, nNewLength );
#endif // MFC, no replace method

}

void CMarkup::x_Adjust( int iPos, int nShift, bool bAfterPos /*=false*/ )
{
	// Loop through affected elements and adjust indexes
	// Algorithm:
	// 1. update children unless bAfterPos
	//    (if no children or bAfterPos is true, length of iPos not affected)
	// 2. update starts of next siblings and their children
	// 3. go up until there is a next sibling of a parent and update starts
	// 4. step 2
	int iPosTop = m_aPos[iPos].iElemParent;
	bool bPosFirst = bAfterPos; // mark as first to skip its children

	// Stop when we've reached the virtual parent (which has no tags)
	while ( m_aPos[iPos].StartTagLen() )
	{
		// Were we at containing parent of affected position?
		bool bPosTop = false;
		if ( iPos == iPosTop )
		{
			// Move iPosTop up one towards root
			iPosTop = m_aPos[iPos].iElemParent;
			bPosTop = true;
		}

		// Traverse to the next update position
		if ( ! bPosTop && ! bPosFirst && m_aPos[iPos].iElemChild )
		{
			// Depth first
			iPos = m_aPos[iPos].iElemChild;
		}
		else if ( m_aPos[iPos].iElemNext )
		{
			iPos = m_aPos[iPos].iElemNext;
		}
		else
		{
			// Look for next sibling of a parent of iPos
			// When going back up, parents have already been done except iPosTop
			while ( 1 )
			{
				iPos = m_aPos[iPos].iElemParent;
				if ( iPos == iPosTop )
					break;
				if ( m_aPos[iPos].iElemNext )
				{
					iPos = m_aPos[iPos].iElemNext;
					break;
				}
			}
		}
		bPosFirst = false;

		// Shift indexes at iPos
		if ( iPos != iPosTop )
			m_aPos[iPos].nStart += nShift;
		else
			m_aPos[iPos].nLength += nShift;
	}
}

int CMarkup::x_InsertNew( int iPosParent, int& iPosRel, CMarkup::NodePos& node )
{
	// Parent empty tag or tags with no content?
	bool bEmptyParentTag = iPosParent && m_aPos[iPosParent].IsEmptyElement();
	bool bNoContentParentTags = iPosParent && ! m_aPos[iPosParent].ContentLen();
	if ( node.nLength )
	{
		// Located at a non-element node
		if ( ! (node.nNodeFlags & MNF_INSERT) )
			node.nStart += node.nLength;
	}
	else if ( iPosRel )
	{
		// Located at an element
		node.nStart = m_aPos[iPosRel].nStart;
		if ( ! (node.nNodeFlags & MNF_INSERT) ) // follow iPosRel
			node.nStart += m_aPos[iPosRel].nLength;
	}
	else if ( bEmptyParentTag )
	{
		// Parent has no separate end tag, so split empty element
		if ( m_aPos[iPosParent].nFlags & MNF_NONENDED )
			node.nStart = m_aPos[iPosParent].StartContent();
		else
			node.nStart = m_aPos[iPosParent].StartContent() - 1;
	}
	else
	{
		if ( node.nNodeFlags & (MNF_INSERT|MNF_REPLACE) )
			node.nStart = m_aPos[iPosParent].StartContent();
		else // before end tag
			node.nStart = m_aPos[iPosParent].StartAfter() - m_aPos[iPosParent].EndTagLen();
	}

	// Go up to start of next node, unless its splitting an empty element
	if ( ! (node.nNodeFlags&(MNF_WITHNOLINES|MNF_REPLACE)) && ! bEmptyParentTag )
	{
		MCD_PCSZ pDoc = MCD_2PCSZ(m_strDoc);
		int nChar = node.nStart;
		if ( ! x_FindAny(pDoc,nChar) || pDoc[nChar] == '<' )
			node.nStart = nChar;
	}

	// Is insert relative to element position? (i.e. not other kind of node)
	if ( ! node.nLength )
	{
		// Modify iPosRel to reflect position before
		if ( iPosRel )
		{
			if ( node.nNodeFlags & MNF_INSERT )
			{
				if ( ! (m_aPos[iPosRel].nFlags & MNF_FIRST) )
					iPosRel = m_aPos[iPosRel].iElemPrev;
				else
					iPosRel = 0;
			}
		}
		else if ( ! (node.nNodeFlags & MNF_INSERT) )
		{
			// If parent has a child, add after last child
			if ( m_aPos[iPosParent].iElemChild )
				iPosRel = m_aPos[m_aPos[iPosParent].iElemChild].iElemPrev;
		}
	}

	// Get node length (used only by x_AddNode)
	node.nLength = MCD_STRLENGTH(node.strMeta);

	// Prepare end of lines
	if ( (! (node.nNodeFlags & MNF_WITHNOLINES)) && (bEmptyParentTag || bNoContentParentTags) )
		node.nStart += x_EOLLEN;
	if ( ! (node.nNodeFlags & MNF_WITHNOLINES) )
		node.strMeta += x_EOL;

	// Calculate insert offset and replace length
	int nReplace = 0;
	int nInsertAt = node.nStart;
	if ( bEmptyParentTag )
	{
		MCD_STR strTagName = x_GetTagName( iPosParent );
		MCD_STR strFormat;
		if ( node.nNodeFlags & MNF_WITHNOLINES )
			strFormat = MCD_T(">");
		else
			strFormat = MCD_T(">") x_EOL;
		strFormat += node.strMeta;
		strFormat += MCD_T("</");
		strFormat += strTagName;
		node.strMeta = strFormat;
		if ( m_aPos[iPosParent].nFlags & MNF_NONENDED )
		{
			nInsertAt = m_aPos[iPosParent].StartAfter() - 1;
			nReplace = 0;
			m_aPos[iPosParent].nFlags ^= MNF_NONENDED;
		}
		else
		{
			nInsertAt = m_aPos[iPosParent].StartAfter() - 2;
			nReplace = 1;
			m_aPos[iPosParent].AdjustStartTagLen( -1 );
		}
		m_aPos[iPosParent].SetEndTagLen( 3 + MCD_STRLENGTH(strTagName) );
	}
	else
	{
		if ( node.nNodeFlags & MNF_REPLACE )
		{
			nInsertAt = m_aPos[iPosParent].StartContent();
			nReplace = m_aPos[iPosParent].ContentLen();
		}
		else if ( bNoContentParentTags )
		{
			node.strMeta = x_EOL + node.strMeta;
			nInsertAt = m_aPos[iPosParent].StartContent();
		}
	}
	x_DocChange( nInsertAt, nReplace, node.strMeta );
	return nReplace;
}

bool CMarkup::x_AddElem( MCD_PCSZ pName, int nValue, int nFlags )
{
	// Convert integer to string
	MCD_CHAR szVal[25];
	MCD_SPRINTF( MCD_SSZ(szVal), MCD_T("%d"), nValue );
	return x_AddElem( pName, szVal, nFlags );
}

bool CMarkup::x_AddElem( MCD_PCSZ pName, MCD_PCSZ pValue, int nFlags )
{
	if ( nFlags & MNF_CHILD )
	{
		// Adding a child element under main position
		if ( ! m_iPos )
			return false;
	}

	// Locate where to add element relative to current node
	NodePos node( nFlags );
	int iPosParent, iPosBefore;
	if ( nFlags & MNF_CHILD )
	{
		iPosParent = m_iPos;
		iPosBefore = m_iPosChild;
	}
	else
	{
		iPosParent = m_iPosParent;
		iPosBefore = m_iPos;
		node.nStart = m_nNodeOffset;
		node.nLength = m_nNodeLength;
	}

	// Cannot have data in non-ended element
	if ( (nFlags&MNF_WITHNOEND) && pValue && pValue[0] )
		return false;

	// Allocate ElemPos structure for this element
	int iPos = x_GetFreePos();

	// Create string for insert
	// If no pValue is specified, an empty element is created
	// i.e. either <NAME>value</NAME> or <NAME/>
	//
	ElemPos* pElem = &m_aPos[iPos];
	int nLenName = MCD_PSZLEN(pName);
	if ( ! pValue || ! pValue[0] )
	{
		// <NAME/> empty element
		node.strMeta = MCD_T("<");
		node.strMeta += pName;
		if ( nFlags & MNF_WITHNOEND )
		{
			node.strMeta += MCD_T(">");
			pElem->SetStartTagLen( nLenName + 2 );
			pElem->nLength = nLenName + 2;
		}
		else
		{
			if ( nFlags & MNF_WITHXHTMLSPACE )
			{
				node.strMeta += MCD_T(" />");
				pElem->SetStartTagLen( nLenName + 4 );
				pElem->nLength = nLenName + 4;
			}
			else
			{
				node.strMeta += MCD_T("/>");
				pElem->SetStartTagLen( nLenName + 3 );
				pElem->nLength = nLenName + 3;
			}
		}
		pElem->SetEndTagLen( 0 );
	}
	else
	{
		// <NAME>value</NAME>
		MCD_STR strValue;
		if ( nFlags & MNF_WITHCDATA )
			strValue = x_EncodeCDATASection( pValue );
		else
		{
#ifdef		ESCAPE
		strValue = EscapeText( pValue, nFlags );
#else
		strValue = pValue;
#endif
		}
		int nLenValue = MCD_STRLENGTH(strValue);
		node.strMeta = MCD_T("<");
		node.strMeta += pName;
		node.strMeta += MCD_T(">");
		node.strMeta += strValue;
		node.strMeta += MCD_T("</");
		node.strMeta += pName;
		node.strMeta += MCD_T(">");
		pElem->SetEndTagLen( nLenName + 3 );
		pElem->nLength = nLenName * 2 + nLenValue + 5;
		pElem->SetStartTagLen( nLenName + 2 );
	}

	// Insert
	int nReplace = x_InsertNew( iPosParent, iPosBefore, node );

	pElem->nStart = node.nStart;
	pElem->iElemChild = 0;
	if ( nFlags & MNF_WITHNOEND )
		pElem->nFlags = MNF_NONENDED;
	else
		pElem->nFlags = 0;
	x_LinkElem( iPosParent, iPosBefore, iPos );

	x_Adjust( iPos, MCD_STRLENGTH(node.strMeta) - nReplace );

	if ( nFlags & MNF_CHILD )
		x_SetPos( m_iPosParent, iPosParent, iPos );
	else
		x_SetPos( iPosParent, iPos, 0 );
	return true;
}

MCD_STR CMarkup::x_GetSubDoc( int iPos ) const
{
	if ( iPos )
	{
		int nStart = m_aPos[iPos].nStart;
		int nNext = nStart + m_aPos[iPos].nLength;
		MCD_PCSZ pDoc = MCD_2PCSZ(m_strDoc);
		int nChar = nNext;
		if ( ! x_FindAny(pDoc,nChar) || pDoc[nChar] == '<' )
			nNext = nChar;
		return MCD_STRMID( m_strDoc, nStart, nNext - nStart );
	}
	return MCD_T("");
}

bool CMarkup::x_AddSubDoc( MCD_PCSZ pSubDoc, int nFlags )
{
	// Add subdocument, parse, and modify positions of affected elements
	//
	NodePos node( nFlags );
	int iPosParent, iPosBefore;
	if ( nFlags & MNF_CHILD )
	{
		// Add a subdocument under main position, before or after child
		if ( ! m_iPos )
			return false;
		iPosParent = m_iPos;
		iPosBefore = m_iPosChild;
	}
	else
	{
		// Add a subdocument under parent position, before or after main
		iPosParent = m_iPosParent;
		iPosBefore = m_iPos;
		node.nStart = m_nNodeOffset;
		node.nLength = m_nNodeLength;
	}

	// Parse subdocument
	bool bWellFormed = true;
	TokenPos token( pSubDoc, m_nDocFlags );
	int iPosVirtual = x_GetFreePos();
	m_aPos[iPosVirtual].ClearVirtualParent();
	m_aPos[iPosVirtual].SetLevel( m_aPos[iPosParent].Level() + 1 );
	int iPos = x_ParseElem( iPosVirtual, token );
	if ( (!iPos) || m_aPos[iPosVirtual].nFlags & MNF_ILLFORMED )
		bWellFormed = false;
	if ( m_aPos[iPosVirtual].nFlags & MNF_ILLDATA )
		m_aPos[iPosParent].nFlags |= MNF_ILLDATA;

	// Extract subdocument without leading/trailing nodes
	int nExtractStart = 0;
	int iPosLast = m_aPos[iPos].iElemPrev;
	if ( bWellFormed )
	{
		nExtractStart = m_aPos[iPos].nStart;
		int nExtractLength = m_aPos[iPos].nLength;
		if ( iPos != iPosLast )
		{
			nExtractLength = m_aPos[iPosLast].nStart - nExtractStart + m_aPos[iPosLast].nLength;
			bWellFormed = false; // treat as subdoc here, but return not well-formed
		}
		MCD_STRASSIGN(node.strMeta,&pSubDoc[nExtractStart],nExtractLength);
	}
	else
	{
		node.strMeta = pSubDoc;
		node.nNodeFlags |= MNF_WITHNOLINES;
	}

	// Insert
	int nReplace = x_InsertNew( iPosParent, iPosBefore, node );

	// Adjust and link in the inserted elements
	// iPosVirtual will stop it from affecting rest of document
	int nAdjust = node.nStart - nExtractStart;
	if ( iPos && nAdjust )
	{
		x_Adjust( iPos, nAdjust );
		m_aPos[iPos].nStart += nAdjust;
	}
	int iPosChild = iPos;
	while ( iPosChild )
	{
		int iPosNext = m_aPos[iPosChild].iElemNext;
		x_LinkElem( iPosParent, iPosBefore, iPosChild );
		iPosBefore = iPosChild;
		iPosChild = iPosNext;
	}
	x_ReleasePos( iPosVirtual );

	// Now adjust remainder of document
	x_Adjust( iPosLast, MCD_STRLENGTH(node.strMeta) - nReplace, true );

	// Set position to top element of subdocument
	if ( nFlags & MNF_CHILD )
		x_SetPos( m_iPosParent, iPosParent, iPos );
	else // Main
		x_SetPos( m_iPosParent, iPos, 0 );
	return bWellFormed;
}

int CMarkup::x_RemoveElem( int iPos )
{
	// Remove element and all contained elements
	// Return new position
	//
	if ( ! iPos )
		return 0;

	// Determine whether any whitespace up to next tag
	int nAfterEnd = m_aPos[iPos].StartAfter();
	MCD_PCSZ pDoc = MCD_2PCSZ(m_strDoc);
	int nChar = nAfterEnd;
	if ( ! x_FindAny(pDoc,nChar) || pDoc[nChar] == '<' )
		nAfterEnd = nChar;

	// Remove from document, adjust affected indexes, and unlink
	int nLen = nAfterEnd - m_aPos[iPos].nStart;
	x_DocChange( m_aPos[iPos].nStart, nLen, MCD_STR() );
	x_Adjust( iPos, - nLen, true );
	int iPosPrev = x_UnlinkElem( iPos );
	x_CheckSavedPos();
	return iPosPrev;
}

void CMarkup::x_LinkElem( int iPosParent, int iPosBefore, int iPos )
{
	// Link in element, and initialize nFlags, and iElem indexes
	ElemPos* pElem = &m_aPos[iPos];
	pElem->iElemParent = iPosParent;
	if ( iPosBefore )
	{
		// Link in after iPosBefore
		pElem->nFlags &= ~MNF_FIRST;
		pElem->iElemNext = m_aPos[iPosBefore].iElemNext;
		if ( pElem->iElemNext )
			m_aPos[pElem->iElemNext].iElemPrev = iPos;
		else
			m_aPos[m_aPos[iPosParent].iElemChild].iElemPrev = iPos;
		m_aPos[iPosBefore].iElemNext = iPos;
		pElem->iElemPrev = iPosBefore;
	}
	else
	{
		// Link in as first child
		pElem->nFlags |= MNF_FIRST;
		if ( m_aPos[iPosParent].iElemChild )
		{
			pElem->iElemNext = m_aPos[iPosParent].iElemChild;
			pElem->iElemPrev = m_aPos[pElem->iElemNext].iElemPrev;
			m_aPos[pElem->iElemNext].iElemPrev = iPos;
			m_aPos[pElem->iElemNext].nFlags ^= MNF_FIRST;
		}
		else
		{
			pElem->iElemNext = 0;
			pElem->iElemPrev = iPos;
		}
		m_aPos[iPosParent].iElemChild = iPos;
	}
	if ( iPosParent )
		pElem->SetLevel( m_aPos[iPosParent].Level() + 1 );
}

int CMarkup::x_UnlinkElem( int iPos )
{
	// Fix links to remove element and mark as deleted
	// return previous position or zero if none
	ElemPos* pElem = &m_aPos[iPos];

	// Find previous sibling and bypass removed element
	int iPosPrev = 0;
	if ( pElem->nFlags & MNF_FIRST )
	{
		if ( pElem->iElemNext ) // set next as first child
		{
			m_aPos[pElem->iElemParent].iElemChild = pElem->iElemNext;
			m_aPos[pElem->iElemNext].iElemPrev = pElem->iElemPrev;
			m_aPos[pElem->iElemNext].nFlags |= MNF_FIRST;
		}
		else // no children remaining
			m_aPos[pElem->iElemParent].iElemChild = 0;
	}
	else
	{
		iPosPrev = pElem->iElemPrev;
		m_aPos[iPosPrev].iElemNext = pElem->iElemNext;
		if ( pElem->iElemNext )
			m_aPos[pElem->iElemNext].iElemPrev = iPosPrev;
		else
			m_aPos[m_aPos[pElem->iElemParent].iElemChild].iElemPrev = iPosPrev;
	}
	x_ReleaseSubDoc( iPos );
	return iPosPrev;
}

int CMarkup::x_ReleasePos( int iPos )
{
	int iPosNext = m_aPos[iPos].iElemNext;
	m_aPos[iPos].iElemNext = m_iPosDeleted;
	m_aPos[iPos].nFlags = MNF_DELETED;
	m_iPosDeleted = iPos;
	return iPosNext;
}

int CMarkup::x_ReleaseSubDoc( int iPos )
{
	// Mark position structures as deleted by depth first traversal
	// Tricky because iElemNext used in traversal is overwritten for linked list of deleted
	// Return value is what iElemNext was before being overwritten
	//
	int iPosNext = 0, iPosTop = iPos;
	while ( 1 )
	{
		if ( m_aPos[iPos].iElemChild )
			iPos = m_aPos[iPos].iElemChild;
		else
		{
			while ( 1 )
			{
				iPosNext = x_ReleasePos( iPos );
				if ( iPos == iPosTop )
					return iPosNext;
				if ( iPosNext )
					break;
				iPos = m_aPos[iPos].iElemParent;
			}
			iPos = iPosNext;
		}
	}
	return iPosNext;
}

bool CMarkup::x_GetMap( SavedPosMap*& pMap, int nMap, int nMapSize /*=7*/ )
{
	// Find or create map, returns true if map(s) created
	SavedPosMap** ppMaps = m_SavedPosMapArray.pMaps;
	int nMapIndex = 0;
	if ( ppMaps )
	{
		// Length of array is unknown, so loop through maps
		while ( nMapIndex <= nMap )
		{
			pMap = ppMaps[nMapIndex];
			if ( ! pMap )
				break;
			if ( nMapIndex == nMap )
				return false; // not created
			++nMapIndex;
		}
		nMapIndex = 0;
	}

	// Create map(s)
	// If you access map 1 before map 0 created, then 2 maps will be created
	m_SavedPosMapArray.pMaps = new SavedPosMap*[nMap+2];
	if ( ppMaps )
	{
		while ( ppMaps[nMapIndex] )
		{
			m_SavedPosMapArray.pMaps[nMapIndex] = ppMaps[nMapIndex];
			++nMapIndex;
		}
		delete[] ppMaps;
	}
	ppMaps = m_SavedPosMapArray.pMaps;
	while ( nMapIndex <= nMap )
	{
		ppMaps[nMapIndex] = new SavedPosMap( nMapSize );
		++nMapIndex;
	}
	ppMaps[nMapIndex] = NULL;
	pMap = ppMaps[nMap];
	return true; // map(s) created
}

void CMarkup::x_CheckSavedPos()
{
	// Remove any saved positions now pointing to deleted elements
	// Must be done as part of element removal before position reassigned
	if ( m_SavedPosMapArray.pMaps )
	{
		int nMap = 0;
		while ( m_SavedPosMapArray.pMaps[nMap] )
		{
			SavedPosMap* pMap = m_SavedPosMapArray.pMaps[nMap];
			for ( int nSlot = 0; nSlot < pMap->nMapSize; ++nSlot )
			{
				SavedPos* pSavedPos = pMap->pTable[nSlot];
				if ( pSavedPos )
				{
					int nOffset = 0;
					int nSavedPosCount = 0;
					while ( 1 )
					{
						if ( pSavedPos[nOffset].nSavedPosFlags & SavedPos::SPM_USED )
						{
							int iPos = pSavedPos[nOffset].iPos;
							if ( ! (m_aPos[iPos].nFlags & MNF_DELETED) )
							{
								if ( nSavedPosCount < nOffset )
								{
									pSavedPos[nSavedPosCount] = pSavedPos[nOffset];
									pSavedPos[nSavedPosCount].nSavedPosFlags &= ~SavedPos::SPM_LAST;
								}
								++nSavedPosCount;
							}
						}
						if ( pSavedPos[nOffset].nSavedPosFlags & SavedPos::SPM_LAST )
						{
							while ( nSavedPosCount <= nOffset )
								pSavedPos[nSavedPosCount++].nSavedPosFlags &= ~SavedPos::SPM_USED;
							break;
						}
						++nOffset;
					}
				}
			}
			++nMap;
		}
	}
}

void CMarkup::x_AdjustForNode( int iPosParent, int iPos, int nShift )
{
	// Adjust affected indexes
	bool bAfterPos = true;
	if ( ! iPos )
	{
		// Change happened before or at first element under iPosParent
		// If there are any children of iPosParent, adjust from there
		// otherwise start at parent and adjust from there
		iPos = m_aPos[iPosParent].iElemChild;
		if ( iPos )
		{
			m_aPos[iPos].nStart += nShift;
			bAfterPos = false;
		}
		else
		{
			iPos = iPosParent;
			m_aPos[iPos].nLength += nShift;
		}
	}
	x_Adjust( iPos, nShift, bAfterPos );
}

bool CMarkup::x_AddNode( int nNodeType, MCD_PCSZ pText, int nNodeFlags )
{
	// Only comments, DTDs, and processing instructions are followed by CRLF
	// Other nodes are usually concerned with mixed content, so no CRLF
	if ( ! (nNodeType & (MNT_PROCESSING_INSTRUCTION|MNT_COMMENT|MNT_DOCUMENT_TYPE)) )
		nNodeFlags |= MNF_WITHNOLINES;

	// Add node of nNodeType after current node position
	NodePos node( nNodeFlags );
	if ( ! x_CreateNode(node.strMeta, nNodeType, pText) )
		return false;

	// Locate where to add node relative to current node
	int iPosBefore = m_iPos;
	int iPosParent = m_iPosParent;
	node.nStart = m_nNodeOffset;
	node.nLength = m_nNodeLength;
	node.nNodeType = nNodeType;

	int nReplace = x_InsertNew( iPosParent, iPosBefore, node );

	// If its a new element, create an ElemPos
	int iPos = iPosBefore;
	if ( nNodeType == MNT_ELEMENT )
	{
		// Set indexes
		iPos = x_GetFreePos();
		ElemPos* pElem = &m_aPos[iPos];
		pElem->nStart = node.nStart;
		pElem->SetStartTagLen( node.nLength );
		pElem->SetEndTagLen( 0 );
		pElem->nLength = node.nLength;
		node.nStart = 0;
		node.nLength = 0;
		pElem->iElemChild = 0;
		pElem->nFlags = 0;
		x_LinkElem( iPosParent, iPosBefore, iPos );
	}

	// Need to adjust element positions after iPos
	x_AdjustForNode( iPosParent, iPos, MCD_STRLENGTH(node.strMeta) - nReplace );

	// Set current position
	m_iPos = iPos;
	m_iPosChild = 0;
	m_nNodeOffset = node.nStart;
	m_nNodeLength = node.nLength;
	m_nNodeType = nNodeType;
	MARKUP_SETDEBUGSTATE;
	return true;
}

void CMarkup::x_RemoveNode( int iPosParent, int& iPos, int& nNodeType, int& nNodeOffset, int& nNodeLength )
{
	// Remove node and return new position
	//
	int iPosPrev = iPos;

	// Removing an element?
	if ( nNodeType == MNT_ELEMENT )
	{
		nNodeOffset = m_aPos[iPos].nStart;
		nNodeLength = m_aPos[iPos].nLength;
		iPosPrev = x_UnlinkElem( iPos );
		x_CheckSavedPos();
	}

	// Find previous node type, offset and length
	int nPrevOffset = 0;
	if ( iPosPrev )
		nPrevOffset = m_aPos[iPosPrev].StartAfter();
	else if ( iPosParent )
		nPrevOffset = m_aPos[iPosParent].StartContent();
	TokenPos token( m_strDoc, m_nDocFlags );
	NodePos node;
	token.nNext = nPrevOffset;
	int nPrevType = 0;
	while ( token.nNext < nNodeOffset )
	{
		nPrevOffset = token.nNext;
		nPrevType = x_ParseNode( token, node );
	}
	int nPrevLength = nNodeOffset - nPrevOffset;
	if ( ! nPrevLength )
	{
		// Previous node is iPosPrev element
		nPrevOffset = 0;
		if ( iPosPrev )
			nPrevType = MNT_ELEMENT;
	}

	// Remove node from document
 	x_DocChange( nNodeOffset, nNodeLength, MCD_STR() );
	x_AdjustForNode( iPosParent, iPosPrev, - nNodeLength );

	// Was removed node a lone end tag?
	if ( nNodeType == MNT_LONE_END_TAG )
	{
		// See if we can unset parent MNF_ILLDATA flag
		token.nNext = m_aPos[iPosParent].StartContent();
		int nEndOfContent = token.nNext + m_aPos[iPosParent].ContentLen();
		int iPosChild = m_aPos[iPosParent].iElemChild;
		while ( token.nNext < nEndOfContent )
		{
			if ( x_ParseNode(token,node) <= 0 )
				break;
			if ( node.nNodeType == MNT_ELEMENT )
			{
				token.nNext = m_aPos[iPosChild].StartAfter();
				iPosChild = m_aPos[iPosChild].iElemNext;
			}
		}
		if ( token.nNext == nEndOfContent )
			m_aPos[iPosParent].nFlags &= ~MNF_ILLDATA;
	}

	nNodeType = nPrevType;
	nNodeOffset = nPrevOffset;
	nNodeLength = nPrevLength;
	iPos = iPosPrev;
}
