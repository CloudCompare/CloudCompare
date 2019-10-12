////PrjLog.hpp
/********************************************************************
	PrjLog
	created:	2013/10/01
	author:		LX 
	purpose:	This file is for PrjLog function
*********************************************************************/
#if !defined PrjLog_hpp__LX_2013_10_1
#define PrjLog_hpp__LX_2013_10_1

#include <stdio.h> 
#include <stdlib.h> 
#include <time.h>
#include <memory.h>
#include <sys/stat.h>
#include <windows.h>
//#include "typedef.h"

#define VERF_LINE(s) if ( s[strlen(s)-1]=='\n' ) { if(s[strlen(s)-2]=='\r') s[strlen(s)-2]=0; else s[strlen(s)-1]=0;}
#define VERF_SLASH(s) if ( s[strlen(s)-1]=='/'||s[strlen(s)-1]=='\\' ) s[strlen(s)-1]=0

#ifndef _Dos2Unix
#define _Dos2Unix
inline void Dos2Unix(char *strCmd){ int i,len = strlen(strCmd); for ( i=0;i<len;i++ ){ if ( strCmd[i]=='\\' ) strCmd[i]='/'; } }
#endif
#ifndef _Unix2Dos
#define _Unix2Dos
inline void Unix2Dos(char *strCmd){ int i,len = strlen(strCmd); for ( i=0;i<len;i++ ){ if ( strCmd[i]=='/' ) strCmd[i]='\\'; } }
#endif
inline char* strrpath(const char* s)
{
	char *pSplit1 = (char*)strrchr( s,'\\' ); 
	char *pSplit2 = (char*)strrchr( s,'/' );
	
	return pSplit1>pSplit2?pSplit1:pSplit2;
}

#ifdef WIN32

#include <Winsock.h>
#pragma comment(lib,"Ws2_32.lib") 
#pragma message("Automatically linking with Ws2_32.lib") 
inline size_t readlink( const char* pPar,char *buf,size_t bufSz ){ return (size_t)GetModuleFileName( NULL,buf,bufSz ); }
inline void _gethostname( char *n,unsigned long nz){ GetComputerName(n,&nz); }

#ifndef _IsExist
#define _IsExist
inline BOOL IsExist(LPCSTR lpstrPathName){
    WIN32_FIND_DATA fd; HANDLE hFind=INVALID_HANDLE_VALUE;
    hFind = ::FindFirstFile(lpstrPathName,&fd);
    if ( hFind==INVALID_HANDLE_VALUE ) return FALSE;
    ::FindClose(hFind); return TRUE;
}
#endif

#ifndef _CreateDir
#define _CreateDir
inline static BOOL CreateDir(LPCTSTR szPath){
    WIN32_FIND_DATA fd; HANDLE hFind = ::FindFirstFile(szPath,&fd);
    if ( hFind!=INVALID_HANDLE_VALUE ){ ::FindClose(hFind); ::CreateDirectory(szPath,NULL); return TRUE; }
    char strPath[512]; strcpy( strPath,szPath );
	char *pSplit1 = strrchr( strPath,'\\' ); 
	char *pSplit2 = strrchr( strPath,'/' );
	char *pSplit = pSplit1>pSplit2?pSplit1:pSplit2;
	if ( !pSplit ) return TRUE; else *pSplit = 0; 
    if ( !CreateDir(strPath) ) return FALSE;
    return ::CreateDirectory(szPath,NULL);
}
#endif

#ifndef _RemoveDir
#define _RemoveDir
inline BOOL RemoveDir(LPCSTR szPath){
	char strPath[512]; strcpy( strPath,szPath ); 
	if ( strPath[strlen(strPath)-1]=='/'||strPath[strlen(strPath)-1]=='\\' ) strPath[strlen(strPath)-1]=0;
	WIN32_FIND_DATA fd;  strcat( strPath,"\\*.*" );
	HANDLE hFind = ::FindFirstFile(strPath,&fd);
    if ( hFind!=INVALID_HANDLE_VALUE ){ 
		while( FindNextFile(hFind, &fd) ){
			if ( (fd.dwFileAttributes&FILE_ATTRIBUTE_DIRECTORY)==FILE_ATTRIBUTE_DIRECTORY )
				RemoveDir( strPath );
			else{
				strcpy( strrchr(strPath,'\\')+1,fd.cFileName );
				::DeleteFile( strPath );				
			}
		}
		::FindClose(hFind);		
	}
    return ::RemoveDirectory(szPath);
}
#endif

/* COM headers (requires shell32.lib, ole32.lib, uuid.lib) */
#include <objbase.h>
#include <shlobj.h>

inline BOOL   CreateLink(LPCSTR szPath,LPCSTR szLink)  					 
{   
    
// 	HRESULT   hres   ;    
// 	IShellLink   *   psl   ;       
// 	IPersistFile*   ppf   ;      
// 	TCHAR   wsz[   MAX_PATH]   ;   
//     
// 	//创建一个IShellLink实例       
// 	hres   =   CoCreateInstance(   CLSID_ShellLink,   NULL,   		
// 		CLSCTX_INPROC_SERVER,   IID_IShellLink, (void   **)&psl)   ;   
//     
// 	if(   FAILED(   hres))   
// 		
// 		return   FALSE   ;   
//     
// 	//设置目标应用程序   
//     
// 	psl   ->   SetPath(   szPath)   ;   
//         
// 	//从IShellLink获取其IPersistFile接口   
//     
// 	//用于保存快捷方式的数据文件   (*.lnk)   
//     
// 	hres   =   psl   ->   QueryInterface(   IID_IPersistFile,   
// 		
// 		(void**)&ppf)   ;   
//     
// 	if(   FAILED(   hres))   
// 		
// 		return   FALSE   ;   
//     
// 	//   确保数据文件名为ANSI格式   
//     
// 	MultiByteToWideChar(   CP_ACP,   0,   szLink,   -1,     
// 		
// 		wsz,   MAX_PATH)   ;   
//     
// 	//调用IPersistFile::Save   
//     
// 	//保存快捷方式的数据文件   (*.lnk)   
//     
// 	hres   =   ppf   ->   Save(   wsz,   STGM_READWRITE)   ;   
//     
// 	//释放IPersistFile和IShellLink接口   
//     
// 	ppf   ->   Release(   )   ;   
//     
// 	psl   ->   Release(   )   ;   
    
	return   TRUE;   
    
}
inline void RemoveLink(LPCSTR szLinkName)
{
	
}

#else
#ifndef	__USE_LARGEFILE64
#define __USE_LARGEFILE64
#endif
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdarg.h>
#include <netdb.h> 
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define _gethostname	gethostname	

#ifndef _strlwr
#define _strlwr
inline char* strlwr( char *str ){
	char *orig = str; char d = 'a'-'A';
	for( ;*str!='\0';str++ ){ if ( *str>='A' && *str<='Z' ) *str = *str+d; }
	return orig;
}
#endif

#ifndef _IsExist
#define _IsExist
inline BOOL IsExist(LPCSTR lpstrPathName){return ( (access(lpstrPathName,0)==0)?TRUE:FALSE ); }
#endif

#ifndef _CreateDir
#define _CreateDir
inline BOOL CreateDir(LPCSTR szPath){
	char strPath[512],*pSplit; 
	if ( IsExist(szPath) ) return TRUE;
	strcpy( strPath,szPath ); pSplit = strrchr( strPath,'\\' );
	if ( !pSplit ) pSplit = strrchr( strPath,'/' );
	
	if ( !pSplit ) return TRUE; else *pSplit = 0; 
    if ( !CreateDir(strPath) ) return FALSE;
    return (mkdir( szPath,S_IRWXU )==0)?TRUE:FALSE;
}
#endif

#ifndef _RemoveDir
#define _RemoveDir
inline BOOL RemoveDir(LPCSTR szPath){ char strCmd[1024]; sprintf( strCmd,"rm -f -r \"%s\"",szPath ); system( strCmd ); }
#endif

#ifndef _CopyFile
#define _CopyFile
inline BOOL CopyFile( LPCSTR lpExistingFileName, LPCSTR lpNewFileName, BOOL bFailIfExists)
{ if( !strcmp(lpExistingFileName,lpNewFileName) ) return TRUE; if(IsExist(lpNewFileName)) { if(bFailIfExists) return FALSE; else remove(lpNewFileName);	} char strCmd[1024]; sprintf( strCmd,"cp -f \"%s\" \"%s\"",lpExistingFileName,lpNewFileName ); system( strCmd ); return TRUE;}
#endif

inline BOOL CreateLink(LPCSTR szTarget,LPCSTR szLinkName)
{
	char strCmd[1024]; 
	sprintf(strCmd,"ln -s \"%s\" \"%s\" ",szTarget,szLinkName);
	if ( system(strCmd) == 0 ){
		return FALSE;
	}
	return TRUE;
}

inline void RemoveLink(LPCSTR szLinkName)
{
	char strCmd[1024]; sprintf( strCmd,"unlink \"%s\"",szLinkName ); system( strCmd );
}
#endif // end of #define WIN32

inline BOOL IsDir(LPCSTR szPath)
{
	char strPath[512];	strcpy(strPath,szPath);		VERF_SLASH(strPath);
	struct stat buf;
	int cc;
    cc=stat(strPath,&buf);	//lstat
    if(!cc && (buf.st_mode & S_IFDIR)) return TRUE;
    return FALSE;
}

#ifndef S_IFLNK
#define S_IFLNK 0120000
#endif

inline bool IsLnk(LPCSTR szPath)
{
	char strPath[512];	strcpy(strPath,szPath);		VERF_SLASH(strPath);
	struct stat buf;
	int cc;
    cc=stat(strPath,&buf);	//lstat
    if(!cc && (buf.st_mode & S_IFLNK)) return true;
    return false;
}
inline int filecmp_mtime(LPCSTR f1,LPCSTR f2)
{
	if(!f1 || !f2 || !strcmp(f1,f2) ) return 0;
	struct stat fs1,fs2;
	int c1,c2;

	c1 = stat(f1,&fs1);	c2 = stat(f2,&fs2);
	if (c1){
		if(c2) return 0;
		return -1;
	}
	if (c2){
		return 1;
	}

	if ( fs1.st_mtime > fs2.st_mtime ){
		return 1;
	}
	if ( fs1.st_mtime < fs2.st_mtime ){
		return -1;
	}
	return 0;
}

#define		CONFIG_FILE	"FlowControl.ini"

#define     PRJ_ERR     "/flc.ERR"
#define     ATMCH_ERR   "/AT/ATMCH/ATMCH.ERR"
#define     ATADJ_ERR   "/AT/ATADJ/ATADJ.ERR"
#define     DEMMCH_ERR  "/DEM/DEMMCH/DEMMCH.ERR"
#define     DOMEPI_ERR  "/DOM/EPI/EPI.ERR"
#define     DOMREC_ERR  "/DOM/REC/REC.ERR"
#define     DOMFUS_ERR  "/DOM/FUS/FUS.ERR"
#define     DOMMZX_ERR  "/DOM/MZX/MZX.ERR"

#define     PRJ_LOG     "/flc.LOG"
#define     ATMCH_LOG   "/AT/ATMCH/ATMCH.LOG"
#define     ATADJ_LOG   "/AT/ATADJ/ATADJ.LOG"
#define     DEMMCH_LOG  "/DEM/DEMMCH/DEMMCH.LOG"
#define     DOMEPI_LOG  "/DOM/EPI/EPI.LOG"
#define     DOMREC_LOG  "/DOM/REC/REC.LOG"
#define     DOMFUS_LOG  "/DOM/FUS/FUS.LOG"
#define     DOMMZX_LOG  "/DOM/MZX/MZX.LOG"


#define ERR_NONE			0x0000
#define ERR_UNKNOW			0xFFFF
#define ERR_INITVAR			0xFFFE


// State of execute
#define ERR_EXE_FLAG		0x0010
#define ERR_EXE_NOTFIND		0x0011
#define ERR_EXE_NOTRUN		0x0012
#define ERR_EXE_NOTARGV		0x0013
#define	ERR_EXE_NOTPAR		ERR_EXE_NOTARGV
#define ERR_EXE_PARERR		0x0015
#define ERR_EXE_RUNING  	0x0016
#define ERR_EXE_KILL		0x0017


// State of project open
#define ERR_PRJ_FLAG		0x0020
#define ERR_PRJ_NOTFIND		0x0021
#define ERR_PRJ_OPEN		0x0022
#define ERR_PRJ_FORMAT		0x0023
#define ERR_PRJ_NOTDIR		0x0024

// State of dictionary
#define ERR_DIR_FLAG		0x0030
#define ERR_DIR_CREATE		0x0031
#define ERR_DIR_DELETE		0x0032
#define ERR_DIR_NOTFIND		0x0033

// State of file
#define ERR_FILE_TYPE		0x0047
#define ERR_FILE_FLAG		0x0040
#define ERR_FILE_OPEN		0x0041
#define ERR_FILE_OPEN4R		0x0042
#define ERR_FILE_OPEN4W		0x0043
#define ERR_FILE_OPEN4C		0x0044
#define ERR_FILE_READ		0x0045
#define ERR_FILE_WRITE		0x0046

#define ERR_BUFFER_MALLOC	0x0050

// 流程控制 错误信息码 开始,具体内容请自己添加
#define ERR_LCKZ_FLAG		0x1000

// 空三匹配 错误信息码 开始,具体内容请自己添加
#define ERR_ATMCH_FLAG		0x3000

// 空三平差 错误信息码 开始,具体内容请自己添加
#define ERR_ATADJ_FLAG	    0x5000


// DEM生产 错误信息码 开始,具体内容请自己添加
#define ERR_DEM_FLAG		0x7000

// DOM生产 错误信息码 开始,具体内容请自己添加
#define ERR_DOM_FLAG		0x8000

// 核线立体 错误信息码 开始,具体内容请自己添加
#define ERR_EPI_FLAG		0x9000
#define ERR_EPI_ORBERR		0x9001
#define ERR_EPI_MODERR		0x9002
#define ERR_EPI_MAKERR		0x9003
#define ERR_EPI_TSKPRJ		0x9004
#define ERR_EPI_SRCIMG		0x9005
#define ERR_EPI_EPIIMG		0x9006

void PrintMsg( LPCSTR lpstrMsg );
void PrintLog( const char *pPN,int code,const char *pMsg );


inline void PrintMsg( LPCSTR lpstrMsg )
{
	printf( "%s\n",lpstrMsg );	
}



#ifndef FLOW_PRJ_MIP

inline void PrintLog( const char *pPN,int code,const char *pMsg ){
	FILE *fTxt = fopen( pPN,"a+" );
	if ( fTxt ){
		time_t lt = time(0); 
		char execname[256]="";  readlink( "/proc/self/exe",execname,256 );
		char hostname[256]="";  _gethostname(hostname, sizeof(hostname)); 
		struct hostent *addr = gethostbyname(hostname);

		fprintf( fTxt,"Code= 0x%.4X\n",code );
		fprintf( fTxt,"MsgInf= %s\n",pMsg );
        fprintf( fTxt,"Machine= %s(%s)\n",hostname,addr?inet_ntoa(*(struct in_addr *)addr->h_addr_list[0]):"." );
		fprintf( fTxt,"DateTime= %s",asctime(localtime(&lt)) );
		fprintf( fTxt,"ExecFile= %s\n",execname );
		fclose( fTxt );
	}
}

#endif


static char gs_strLog[512];
static char gs_strSta[512];
static char gs_strErr[512];

inline  void ErrPrint(int errCode,const char *fmt, ... );
inline  bool OpenErr( LPCSTR lpstrPathName ){ ErrPrint( ERR_INITVAR,gs_strErr ); return strcpy( gs_strErr,lpstrPathName )!=NULL; }
inline  void ErrPrint(int errCode,const char *fmt, ... ){
	static char *gs_PN = NULL; if ( errCode==ERR_INITVAR ) gs_PN=(char*)fmt;
	if ( errCode!=ERR_INITVAR && gs_PN && strlen(gs_PN)>3 ){
		char strMsg[2048]; va_list ap; va_start( ap, fmt ); vsprintf( strMsg,fmt,ap ); va_end(ap);
		PrintLog( gs_PN,errCode,strMsg ); PrintMsg( strMsg );
	}
}

inline  void StaPrint(int staCode,const char *fmt, ... );
inline  bool OpenSta( LPCSTR lpstrPathName ){ StaPrint( ERR_INITVAR,gs_strSta );  return strcpy( gs_strSta,lpstrPathName )!=NULL; }
inline  void StaPrint(int staCode,const char *fmt, ... ){
	static char *gs_PN = NULL; if ( staCode==ERR_INITVAR ) gs_PN=(char*)fmt;
	if ( staCode!=ERR_INITVAR && gs_PN && strlen(gs_PN)>3 ){
		char strMsg[2048]; va_list ap; va_start( ap, fmt ); vsprintf( strMsg,fmt,ap ); va_end(ap);
		PrintLog( gs_PN,staCode,strMsg ); PrintMsg( strMsg );
	}
}

inline  void LogPrint(int logCode,const char *fmt, ... );
inline  bool OpenLog( LPCSTR lpstrPathName ){ LogPrint( ERR_INITVAR,gs_strLog ); return strcpy( gs_strLog,lpstrPathName )!=NULL; }
inline  void LogPrint(int logCode,const char *fmt, ... ){
	static char *gs_PN = NULL; if ( logCode==ERR_INITVAR ) gs_PN=(char*)fmt;
	if ( logCode!=ERR_INITVAR ){
		char strMsg[2048]; va_list ap; va_start( ap, fmt ); vsprintf( strMsg,fmt,ap ); va_end(ap);
		if (gs_PN && strlen(gs_PN)>3) PrintLog( gs_PN,logCode,strMsg );
		PrintMsg( strMsg );
	}
}

inline  void GlobPrint( const char *fmt, ... ){
	static char strMsg[2048]; va_list ap; va_start( ap, fmt ); vsprintf( strMsg,fmt,ap ); va_end(ap);
	char log[256];  readlink( "/proc/self/exe",log,256 );
	char *pS = strrchr( log,'.' ); if (!pS) pS = log+strlen(log); strcpy( pS,".LOG" );
	PrintLog( log,ERR_EXE_FLAG,strMsg ); PrintMsg( strMsg );
}


#endif // PrjLog_hpp__LX_2013_10_1