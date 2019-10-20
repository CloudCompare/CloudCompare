/*
@File	: StFileOperator.hpp
@Brief	: Functions for file operation
@Author	: Xinyi Liu
@Date	: May 05th, 2016
*/
#ifndef __ST_FILE_OPERATOR_HPP__
#define __ST_FILE_OPERATOR_HPP__

#ifdef WIN32
#include <windows.h>
#else
typedef 	unsigned int	HANDLE;
typedef 	int				BOOL;
typedef 	const char*     LPCSTR;
typedef		char*			LPSTR;
typedef 	unsigned char	BYTE;
typedef 	unsigned int	RGBQUAD;
typedef 	unsigned short	WORD;
typedef 	unsigned int	DWORD;
typedef 	unsigned int	UINT;
typedef 	unsigned long	LONGLONG;
#endif

#include <io.h>
#include <vector>
#include <string>
#include <tchar.h>
#include <time.h>
#include <chrono>
#include <random>
#include <algorithm>
#include <direct.h>

#ifdef _UNICODE
#define IO_STR std::wstring
#else
#define IO_STR std::string
#endif // _UNICODE

inline void Dos2Unix(TCHAR *strCmd){ size_t len = lstrlen(strCmd); for (size_t i = 0; i < len; i++){ if (strCmd[i] == '\\') strCmd[i] = '/'; } }

inline void Unix2Dos(TCHAR *strCmd){ size_t len = lstrlen(strCmd); for (size_t i = 0; i < len; i++){ if (strCmd[i] == '/') strCmd[i] = '\\'; } }

inline void GetUnixDir(TCHAR *strCmd){ Dos2Unix(strCmd); if (strCmd[lstrlen(strCmd) - 1] != '/') strcat(strCmd, "/"); }

inline bool IsExist(LPCTSTR lpstrPathName){
	WIN32_FIND_DATA fd; HANDLE hFind = INVALID_HANDLE_VALUE;
	hFind = ::FindFirstFile(lpstrPathName, &fd);
	if (hFind == INVALID_HANDLE_VALUE) return FALSE;
	::FindClose(hFind); return TRUE;
}

inline static bool CreateDir(LPCTSTR szPath){
	WIN32_FIND_DATA fd; HANDLE hFind = ::FindFirstFile(szPath, &fd);
	if (hFind != INVALID_HANDLE_VALUE){ ::FindClose(hFind); ::CreateDirectory(szPath, NULL); return TRUE; }
	TCHAR strPath[512]; _tcscpy_s(strPath, szPath);
	TCHAR *pSplit1 = _tcsrchr(strPath, '\\');
	TCHAR *pSplit2 = _tcsrchr(strPath, '/');
	TCHAR *pSplit = pSplit1 > pSplit2 ? pSplit1 : pSplit2;
	if (!pSplit) return TRUE; else *pSplit = 0;
	if (!CreateDir(strPath)) return FALSE;
	return ::CreateDirectory(szPath, NULL);
}

inline TCHAR* GetExtName(LPCTSTR strPath)
{
	TCHAR str[512]; _tcscpy_s(str, strPath);
	TCHAR *ptr; ptr = _tcsrchr(str, '.');
	if (ptr) return ptr;
	return 0;
}
inline IO_STR ExcludeExt(IO_STR file_path)
{
	TCHAR str[512]; _tcscpy_s(str, file_path.c_str());
	TCHAR *ptr = _tcsrchr(str, '.');
	if (ptr) _tcscpy(ptr, "");
	return str;
}

inline char *GetFileDirectory(const char* filename) {
	char *ret = NULL;
	char dir[1024];
	char *cur;
	if (filename == NULL) return(NULL);

#if defined(WIN32) && !defined(__CYGWIN__)  
#   define IS_SEP(ch) ((ch=='/')||(ch=='\\'))  
#else  
#   define IS_SEP(ch) (ch=='/')  
#endif  
	strncpy(dir, filename, 1023);
	dir[1023] = 0;
	cur = &dir[strlen(dir)];
	while (cur > dir)
	{
		if (IS_SEP(*cur)) break;

		cur--;
	}
	if (IS_SEP(*cur))
	{
		if (cur == dir)
		{
			//1.根目录  
			dir[1] = 0;
		}
		else
		{
			*cur = 0;
		}
		ret = strdup(dir);
	}
	else
	{
		//1.如果是相对路径,获取当前目录  
		//io.h  
		if (getcwd(dir, 1024) != NULL)
		{
			dir[1023] = 0;
			ret = strdup(dir);
		}
	}
	strcat(ret, "\\");
	return ret;
#undef IS_SEP  
}

//#include <ShObjIdl.h>
//#include <ShlGuid.h>
//inline bool GetShortcutFile(LPCTSTR shortcutpath, TCHAR* objectpath){
//	HRESULT hres;
//	IShellLink *psl;
//	IPersistFile *ppf;
//	WIN32_FIND_DATA fd;
//	CoInitialize(NULL);
//	hres = CoCreateInstance(CLSID_ShellLink, NULL, CLSCTX_INPROC_SERVER, IID_IShellLink, (void**)&psl);
//	if (!SUCCEEDED(hres)) return false;
//
//	hres = psl->QueryInterface(IID_IPersistFile, (void**)&ppf);
//	if (SUCCEEDED(hres))
//	{
//		wchar_t wsz[MAX_PATH];
//#ifdef _UNICODE
//		_tcscpy_s(wsz, shortcutpath);
//#else
//		MultiByteToWideChar(CP_ACP, 0, shortcutpath, -1, wsz, MAX_PATH);
//#endif // _UNICODE			
//		hres = ppf->Load(wsz, STGM_READ);
//		if (SUCCEEDED(hres))
//			hres = psl->GetPath(objectpath, MAX_PATH, &fd, NULL);
//		ppf->Release();
//	}
//	psl->Release();  return SUCCEEDED(hres);
//}

inline void FindPathInDir(IO_STR directory_path, std::vector<IO_STR>& filepaths)
{
	if (!IsExist(directory_path.c_str())){ return; }
	struct _finddata_t file_info;
	IO_STR strfind = directory_path + "\\*";
	intptr_t handle = _findfirst(strfind.c_str(), &file_info);
	if (-1 == handle) return;

	do{
		if (file_info.attrib & _A_SUBDIR){
			if ((strcmp(file_info.name, ".") != 0) && (strcmp(file_info.name, "..") != 0)){
				IO_STR newPath = directory_path + "\\" + file_info.name;
				FindPathInDir(newPath, filepaths);
			}
		}
		else
			filepaths.push_back(directory_path + "\\" + file_info.name);
	} while (_findnext(handle, &file_info) == 0);
	_findclose(handle);
}

//! get time string of type "year-month-day:hour-minute-second" from broken-down time
inline IO_STR GetTimestr(tm *time)
{
	if (mktime(time) == -1){
		return IO_STR(_T(""));
	}
	TCHAR strTime[128];
	_tcsftime(strTime, 128, _T("%Y-%m-%d:%H-%M-%S"), time);
	return IO_STR(strTime);
}
//! get time string of type "year-month-day:hour-minute-second" from time_t
inline IO_STR GetTimestr(time_t time)
{
	return GetTimestr(localtime(&time));
}
//! Get Broken-down time form string of type "year-month-day:hour-minute-second"
inline tm GetTimeBrd(IO_STR strtime)
{
	tm time;
	_stscanf_s(strtime.c_str(), _T("%d-%d-%d:%d-%d-%d"),
		&time.tm_year, &time.tm_mon, &time.tm_mday,
		&time.tm_hour, &time.tm_min, &time.tm_sec);
	time.tm_year -= 1900;
	time.tm_mon -= 1;
	return time;
}
inline tm GetLocalTime()
{
	time_t _time; time(&_time);
	return *localtime(&_time);
}

//! print log to file error_type 0 = print "process", 1 = print "warning", 2 = print "error", 3 = no type print
inline bool PrintLog(LPCTSTR file_path, LPCTSTR log_info, int error_type = 0, bool bPrintTime = true, bool bAddto = true){
	FILE* fp;
	if (bAddto)	_tfopen_s(&fp, file_path, _T("at"));
	else		_tfopen_s(&fp, file_path, _T("w"));
	if (!fp) return false;
	IO_STR str_time, str_type;
	if (bPrintTime){ str_time = GetTimestr(&GetLocalTime()) + " - "; }
	if (error_type == 0){ str_type = _T("process : "); }
	else if (error_type == 1){ str_type = _T("warning : "); }
	else if (error_type == 2){ str_type = _T("error : "); }

	_ftprintf_s(fp, _T("%s%s%s\n"), str_time.c_str(), str_type.c_str(), log_info);
	fclose(fp);
	return true;
}

template <class RandomAccessIterator>
inline void Shuffle(RandomAccessIterator first, RandomAccessIterator end){
	auto seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::shuffle(first, end, std::default_random_engine(seed));
}

inline void GenerateRandomRare(std::vector<size_t>& index, size_t data_size, size_t rare_number)
{
	if (rare_number < 0) return;
	for (size_t i = 0; i < data_size; ++i){ index.push_back(i); }
	Shuffle(index.begin(), index.end());
	index.resize(rare_number);
	index.shrink_to_fit();
	sort(begin(index), end(index), [](size_t _l, size_t _r){return _l < _r; });
}

template<class read_type>
inline bool rarefydata(std::vector<read_type>& data, int number)
{
	if (number < 0 || number >= data.size()){
		return true;
	}
	if (number == 0){
		data.clear();
		return true;
	}
	std::vector<size_t> sIndex;
	GenerateRandomRare(sIndex, data.size(), number);

	auto& it_index = sIndex.begin();

	std::vector<read_type> data_out;
	for (size_t i = 0; i < data.size(); ++i){
		if (it_index == sIndex.end()) continue;
		if (i != *it_index) continue;
		it_index++;

		data_out.push_back(data[i]);
	}
	data.clear();
	data.assign(data_out.begin(), data_out.end());
	data.shrink_to_fit();

	return true;
}

inline bool readlines(IO_STR path, std::vector<IO_STR> & lines)
{
	FILE *fp = fopen(path.c_str(), "r");
	if (!fp) {
		return false;
	}
	char lbuf[2048];
	while (fgets(lbuf, 2048, fp)) {
		lines.push_back(lbuf);
	}
	fclose(fp);
	return true;
}

#endif // !__ST_FILE_OPERATOR_HPP__