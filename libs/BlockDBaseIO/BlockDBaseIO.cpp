/*
File	:		BlockDBaseIO.cpp
Brief	:		The cpp file of BlockDBaseIO

Author	:		Xinyi Liu
Date	:		2019/10/20
E-mail	:		liuxy0319@outlook.com
*/

#include "BlockDBaseIO.h"

#include "LasPrjIO.h"
#include "libpq-fe.h"
#include "StFileOperator.hpp"

#include <cfloat>
#include <windows.h>

#include <io.h>
#include <set>

BLKDB_NAMESPACE_BEGIN

#define prj_file_tag "BLK_PRJ"
#define prj_file_ver "1.0"
#define META_SPLIT ";"

static std::string g_error_info;

inline std::vector<std::string> _splitString(char* str, const char* seps)
{
	std::vector<std::string> sub_strs;
	char* token = strtok(str, seps);
	while (token) {
		std::string sub(token);
		sub = sub.substr(0, sub.find_last_of("\t\r\n"));
		sub_strs.push_back(sub);
		token = strtok(NULL, seps);
	}
	return sub_strs;
}

inline void Dos2Unix(char *strCmd) { size_t len = strlen(strCmd); for (size_t i = 0; i < len; i++) { if (strCmd[i] == '\\') strCmd[i] = '/'; } }

bool blkSceneInfo::setStrSceneInfo(std::string str)
{
	char temp[1024]; strcpy(temp, str.c_str());
	std::vector<std::string> seps = _splitString(temp, ";");
	if (seps.size() < 7) {
		return false;
	}
	strcpy(sceneID, seps.front().c_str());
	for (size_t i = 0; i < 6; i++) {
		sscanf(seps[i + 1].c_str(), "%lf", &bound[i]);
	}
	return true;
}

std::string blkSceneInfo::getStrSceneInfo(std::string * key) const
{
	if (key) { *key = "SceneInfo"; }
	std::string par(sceneID); par += ";";

	for (size_t i = 0; i < 6; i++) {
		par += std::to_string(bound[i]) + ";";
	}
	return par;
}

void blkDataInfo::pushToMeta(strMetaMap & meta_info) const
{
	meta_info.insert(std::make_pair("Name", sName));
	meta_info.insert(std::make_pair("ID", sID));
	meta_info.insert(std::make_pair("Path", sPath));
	char strid[32]; sprintf(strid, "%d", nGroupID);
	meta_info.insert(std::make_pair("GroupID", strid));
}

bool blkDataInfo::pullFromMeta(const strMetaMap & meta_info)
{
	std::string value;
	bool result = true;
	result &= _getMetaValue(meta_info, std::string("Name"), value); strcpy(sName, value.c_str());
	result &= _getMetaValue(meta_info, std::string("ID"), value); strcpy(sID, value.c_str());
	result &= _getMetaValue(meta_info, std::string("Path"), value); strcpy(sPath, value.c_str());
	result &= _getMetaValue(meta_info, std::string("GroupID"), value); sscanf(value.c_str(), "%d", &nGroupID);
	return result;
}

bool blkPtCldInfo::isValid()
{
	if (!blkDataInfo::isValid()) {
		return false;
	}
	return true;
}

void blkPtCldInfo::pushToMeta(strMetaMap & meta_info) const
{
	blkDataInfo::pushToMeta(meta_info);
	std::string key, value;
	value = getStrLevel(&key);		meta_info.insert(std::make_pair(key, value));
	value = getStrSceneInfo(&key);	meta_info.insert(std::make_pair(key, value));
}

bool blkPtCldInfo::pullFromMeta(const strMetaMap & meta_info)
{
	bool result = true;
	result &= blkDataInfo::pullFromMeta(meta_info);

	std::string key, value;
	getStrLevel(&key);		result &= _getMetaValue(meta_info, key, value); result &= setStrLevel(value);
	getStrSceneInfo(&key);	result &= _getMetaValue(meta_info, key, value); result &= setStrSceneInfo(value);

	return result;
}

bool blkPtCldInfo::setStrLevel(std::string str)
{
	for (size_t i = 0; i < PCLEVEL_END; i++) {
		if (strcmpi(str.c_str(), g_strPtCldLevelName[i]) == 0) {
			level = BLOCK_PtCldLevel(i);
			return true;
		}
	}
	return false;
}

std::string blkPtCldInfo::getStrLevel(std::string * key) const
{
	if (key) { *key = "Level"; }
	return std::string(g_strPtCldLevelName[level]);
}

bool blkPtCldInfo::setStrSceneInfo(std::string str)
{
	return scene_info.setStrSceneInfo(str);
}

std::string blkPtCldInfo::getStrSceneInfo(std::string * key) const
{
	return scene_info.getStrSceneInfo(key);
}

bool blkCameraInfo::isValid()
{
	if(!blkDataInfo::isValid())
		return false;
	return true;
}

void blkCameraInfo::pushToMeta(strMetaMap & meta_info) const
{
	blkDataInfo::pushToMeta(meta_info);
	std::string key, value;
	value = getStrPixSize(&key);	meta_info.insert(std::make_pair(key, value));
	value = getStrFrameSize(&key);	meta_info.insert(std::make_pair(key, value));
	value = getStrFxy(&key);		meta_info.insert(std::make_pair(key, value));
	value = getStrDistPar(&key);	meta_info.insert(std::make_pair(key, value));
}

bool blkCameraInfo::pullFromMeta(const strMetaMap & meta_info)
{
	bool result = true;
	result &= blkDataInfo::pullFromMeta(meta_info);

	std::string key, value;
	getStrPixSize(&key);	result &= _getMetaValue(meta_info, key, value); result &= setStrPixSize(value);
	getStrFrameSize(&key);	result &= _getMetaValue(meta_info, key, value); result &= setStrFrameSize(value);
	getStrFxy(&key);		result &= _getMetaValue(meta_info, key, value); result &= setStrFxy(value);
	getStrDistPar(&key);	result &= _getMetaValue(meta_info, key, value); result &= setStrDistPar(value);

	return result;
}

bool blkCameraInfo::setStrPixSize(std::string str)
{
	if (str.empty()) { return false; }
	sscanf(str.c_str(), "%lf", &pixelSize);
	return true;
}

std::string blkCameraInfo::getStrPixSize(std::string * key) const
{
	if (key) { *key = "PixelSize"; }
	return std::to_string(pixelSize);
}

bool blkCameraInfo::setStrFrameSize(std::string str)
{
	char temp[1024]; strcpy(temp, str.c_str());
	std::vector<std::string> seps = _splitString(temp, ";");
	if (seps.size() < 2) {
		return false;
	}
	sscanf(seps[0].c_str(), "%d", &width);
	sscanf(seps[1].c_str(), "%d", &height);
	return true;
}

std::string blkCameraInfo::getStrFrameSize(std::string * key) const
{
	if (key) { *key = "FrameSize"; }
	std::string par;
	par += std::to_string(width) + ";";
	par += std::to_string(height) + ";";
	return par;
}

bool blkCameraInfo::setStrFxy(std::string str)
{
	char temp[1024]; strcpy(temp, str.c_str());
	std::vector<std::string> seps = _splitString(temp, ";");
	if (seps.size() < 3) {
		return false;
	}
	sscanf(seps[0].c_str(), "%lf", &f);
	sscanf(seps[1].c_str(), "%lf", &x0);
	sscanf(seps[2].c_str(), "%lf", &y0);

	return true;
}

std::string blkCameraInfo::getStrFxy(std::string * key) const
{
	if (key) { *key = "FocalAndPxPy"; }
	std::string par;
	par += std::to_string(f) + ";";
	par += std::to_string(x0) + ";";
	par += std::to_string(y0) + ";";
	return par;
}

bool blkCameraInfo::setStrDistPar(std::string str)
{
	char temp[1024]; strcpy(temp, str.c_str());
	std::vector<std::string> seps = _splitString(temp, ";");
	if (seps.size() < 8) {
		return false;
	}
	for (size_t i = 0; i < 8; i++) {
		sscanf(seps[i].c_str(), "%lf", &distortionPar[i]);
	}
	return true;
}

std::string blkCameraInfo::getStrDistPar(std::string * key) const
{
	if (key) { *key = "DistortPar"; }
	std::string par;
	for (size_t i = 0; i < 8; i++) {
		par += std::to_string(distortionPar[i]) + ";";
	}
	return par;
}

bool blkImageInfo::isValid()
{
	if (!blkDataInfo::isValid()) {
		return false;
	}
	return _finite(posXs) && _finite(posYs) && _finite(posZs);
}

void blkImageInfo::pushToMeta(strMetaMap & meta_info) const
{
	blkDataInfo::pushToMeta(meta_info);
	std::string key, value; 
	value = getStrLevel(&key);		meta_info.insert(std::make_pair(key, value));
	value = getStrGPS(&key);		meta_info.insert(std::make_pair(key, value));
	value = getStrPOS(&key);		meta_info.insert(std::make_pair(key, value));
	value = getStrCamera(&key);		meta_info.insert(std::make_pair(key, value));
	value = getStrExtraTag(&key);	meta_info.insert(std::make_pair(key, value));
}

bool blkImageInfo::pullFromMeta(const strMetaMap & meta_info)
{
	bool result = true;
	result &= blkDataInfo::pullFromMeta(meta_info);

	std::string key, value;
	getStrLevel(&key); result &= _getMetaValue(meta_info, key, value); result &= setStrLevel(value);
	getStrGPS(&key); result &= _getMetaValue(meta_info, key, value); result &= setStrGPS(value);
	getStrPOS(&key); result &= _getMetaValue(meta_info, key, value); result &= setStrPOS(value);
	getStrCamera(&key); result &= _getMetaValue(meta_info, key, value); result &= setStrCamera(value);
	getStrExtraTag(&key); result &= _getMetaValue(meta_info, key, value); result &= setStrExtraTag(value);

	return result;
}

bool blkImageInfo::setStrLevel(std::string str)
{
	for (size_t i = 0; i < IMGLEVEL_END; i++) {
		if (strcmpi(str.c_str(), g_strImageLevelName[i]) == 0) {
			level = BLOCK_ImgLevel(i);
			return true;
		}
	}
	return false;
}

std::string blkImageInfo::getStrLevel(std::string* key) const
{
	if (key) { *key = "Level"; }
	return g_strImageLevelName[level];
}

bool blkImageInfo::setStrGPS(std::string str)
{
	char temp[1024]; strcpy(temp, str.c_str());
	std::vector<std::string> seps = _splitString(temp, ";");
	if (seps.size() < 3) {
		return false;
	}
	sscanf(seps[0].c_str(), "%lf", &gpsLat);
	sscanf(seps[1].c_str(), "%lf", &gpsLon);
	sscanf(seps[2].c_str(), "%lf", &gpsHeight);
	return true;
}

std::string blkImageInfo::getStrGPS(std::string * key) const
{
	if (key) { *key = "GPSInfo"; }
	char gps_info[1024]; sprintf(gps_info, "%lf;%lf;%lf;", gpsLat, gpsLon, gpsHeight);
	return std::string(gps_info);
}

bool blkImageInfo::setStrPOS(std::string str)
{
	char temp[1024]; strcpy(temp, str.c_str());
	std::vector<std::string> seps = _splitString(temp, ";");
	if (seps.size() < 6) {
		return false;
	}
	sscanf(seps[0].c_str(), "%lf", &posXs);
	sscanf(seps[1].c_str(), "%lf", &posYs);
	sscanf(seps[2].c_str(), "%lf", &posZs);
	sscanf(seps[3].c_str(), "%lf", &posPhi);
	sscanf(seps[4].c_str(), "%lf", &posOmega);
	sscanf(seps[5].c_str(), "%lf", &posKappa);
	if (seps.size() >= 7) {
		sscanf(seps[6].c_str(), "%lf", &gps_time);
	}
	return true;
}

std::string blkImageInfo::getStrPOS(std::string * key) const
{
	if (key) { *key = "POSInfo"; }
	char info[1024];
	sprintf(info, "%lf;%lf;%lf;%lf;%lf;%lf;%lf", posXs, posYs, posZs, posPhi, posOmega, posKappa, gps_time);
	return std::string(info);
}

bool blkImageInfo::setStrCamera(std::string str)
{
	if (str.empty()) { return false; }
	cameraName = str;
	return true;
}

std::string blkImageInfo::getStrCamera(std::string * key) const
{
	if (key) { *key = "CamName"; }
	return cameraName;
}

bool blkImageInfo::setStrExtraTag(std::string str)
{
	char temp[1024]; strcpy(temp, str.c_str());
	std::vector<std::string> seps = _splitString(temp, ";");
	
	if (seps.size() >= 1) { sscanf(seps[0].c_str(), "%d", &stripID); }
	if (seps.size() >= 2) { sscanf(seps[1].c_str(), "%d", &attrib); }
	if (seps.size() >= 3) { sscanf(seps[2].c_str(), "%d", &bFlag); }

	return true;
}

std::string blkImageInfo::getStrExtraTag(std::string * key) const
{
	if (key) { *key = "ExtraTag"; }
	char info[1024];
	sprintf(info, "%d;%d;%d;", stripID, attrib, bFlag);
	return std::string(info);
}

BlockDBaseIO::BlockDBaseIO(const char * path)
{
	setPath(path);
}

BlockDBaseIO::~BlockDBaseIO()
{
}

bool BlockDBaseIO::loadProject()
{
	char lpstrXmlPN[_MAX_PATH]; strcpy(lpstrXmlPN, m_project_path);

	if (_access(lpstrXmlPN, 0) == -1) {
		m_error_info = "cannot read project file: " + std::string(lpstrXmlPN);
		return false;
	}

	char strValue[1024];
	
	// load file tag
	::GetPrivateProfileString("FILE_TAG", "Tag", "", strValue, 1024, lpstrXmlPN);
	if (strcmpi(strValue, prj_file_tag) != 0) {
		m_error_info = "wrong file tag, cannot load project file: " + std::string(lpstrXmlPN);
		return false;
	}
	::GetPrivateProfileString("FILE_TAG", "Ver", "", strValue, 1024, lpstrXmlPN);

	// load file header
	::GetPrivateProfileString("PRJ_HEADER", "PrjName", "", m_projHdr.sName, 1024, lpstrXmlPN);
	::GetPrivateProfileString("PRJ_HEADER", "PrjDirPath", "", m_projHdr.sDirPath, 1024, lpstrXmlPN);

	::GetPrivateProfileString("PRJ_HEADER", "ProjectID", "", strValue, 1024, lpstrXmlPN); sscanf(strValue, "%d", &m_projHdr.projectID);
	::GetPrivateProfileString("PRJ_HEADER", "GroupID", "", strValue, 1024, lpstrXmlPN); sscanf(strValue, "%d", &m_projHdr.groupID);

	::GetPrivateProfileString("DATA_FILE_INFO", "LASList", "", m_projHdr.lasListPath, _MAX_PATH, lpstrXmlPN);
	::GetPrivateProfileString("DATA_FILE_INFO", "IMGList", "", m_projHdr.imgListPath, _MAX_PATH, lpstrXmlPN);
	::GetPrivateProfileString("DATA_FILE_INFO", "CAMList", "", m_projHdr.camParPath, _MAX_PATH, lpstrXmlPN);

	::GetPrivateProfileString("DATA_FILE_INFO", "GCDProd", "Data/miscs/GCDProd.ini", m_projHdr.m_strProdGCDPN, _MAX_PATH, lpstrXmlPN);
	::GetPrivateProfileString("DATA_FILE_INFO", "GCDGcp", "Data/miscs/GCDgCP.ini", m_projHdr.m_strGcpGCDPN, _MAX_PATH, lpstrXmlPN);
	::GetPrivateProfileString("DATA_FILE_INFO", "GCD7Par", "Data/miscs/GCD7par.ini", m_projHdr.m_str7parPN, _MAX_PATH, lpstrXmlPN);

	//! load files
	if (!loadPoints(m_ptClds, m_projHdr.lasListPath)) {
		m_error_info = "failed to load point clouds from: " + std::string(m_projHdr.lasListPath);
		return false;
	}
	if (!loadImages(m_images, m_cameras, m_projHdr.imgListPath, m_projHdr.camParPath)) {
		m_error_info = "failed to load images from: " + std::string(m_projHdr.imgListPath);
		return false;
	}

	//! load meta
	try {
		char strkeys[2048];
		::GetPrivateProfileString("META_INFO", "METAKEY", "", strkeys, 2048, lpstrXmlPN);
		auto keys = _splitString(strkeys, META_SPLIT);
		for (auto & key : keys) {
			::GetPrivateProfileString("META_INFO", key.c_str(), "", strkeys, 2048, lpstrXmlPN);
			addMetaValue(key, strkeys);
		}
	}
	catch (const std::exception&) {
		throw std::runtime_error("not enough memory?");
		return false;
	}
	return true;
}

bool BlockDBaseIO::saveProject(bool save_regisprj)
{
	char lpstrXmlPN[_MAX_PATH]; strcpy(lpstrXmlPN, m_project_path);
	
	GetUnixDir(m_projHdr.sDirPath);
	CreateDir(m_projHdr.sDirPath);
	FILE* fp = fopen(lpstrXmlPN, "w");
	if (!fp) {
		m_error_info = "cannot save to: " + std::string(lpstrXmlPN);
		return false;
	}
	fclose(fp);
	// deduce the dir
	if (strlen(m_projHdr.lasListPath) == 0) sprintf(m_projHdr.lasListPath, "%s%s", m_projHdr.sDirPath, "Data/points/LasFile.ini");
	CreateDir(GetFileDirectory(m_projHdr.lasListPath));
	if (strlen(m_projHdr.imgListPath) == 0) sprintf(m_projHdr.imgListPath, "%s%s", m_projHdr.sDirPath, "Data/images/ImgFile.ini");
	CreateDir(GetFileDirectory(m_projHdr.imgListPath));
	if (strlen(m_projHdr.camParPath) == 0) sprintf(m_projHdr.camParPath, "%s%s", m_projHdr.sDirPath, "Data/images/CamFile.ini");
	CreateDir(GetFileDirectory(m_projHdr.camParPath));
	
	if (strlen(m_projHdr.m_strProdGCDPN) == 0) sprintf(m_projHdr.m_strProdGCDPN, "%s%s", m_projHdr.sDirPath, "Data/miscs/GCDproduct.ini");
	CreateDir(GetFileDirectory(m_projHdr.m_strProdGCDPN));
	if (strlen(m_projHdr.m_strGcpGCDPN) == 0) sprintf(m_projHdr.m_strGcpGCDPN, "%s%s", m_projHdr.sDirPath, "Data/miscs/GCDGcp.ini");
	CreateDir(GetFileDirectory(m_projHdr.m_strGcpGCDPN));
	if (strlen(m_projHdr.m_str7parPN) == 0) sprintf(m_projHdr.m_str7parPN, "%s%s", m_projHdr.sDirPath, "Data/miscs/GCD7par.ini");
	CreateDir(GetFileDirectory(m_projHdr.m_str7parPN));	

	//! save files
	if (!savePoints(m_ptClds, m_projHdr.lasListPath)) {
		m_error_info = g_error_info + "\nSave point clouds failed";
		return false;
	}
	if (!saveImages(m_images, m_cameras, m_projHdr.imgListPath, m_projHdr.camParPath)) {
		m_error_info = g_error_info + "\nSave images failed";
		return false;
	}

	char strValue[1024];
	// write file tag
	::WritePrivateProfileString("FILE_TAG", "Tag", prj_file_tag, lpstrXmlPN);
	::WritePrivateProfileString("FILE_TAG", "Ver", prj_file_ver, lpstrXmlPN);
	// write prj header
	::WritePrivateProfileString("PRJ_HEADER", "PrjName", m_projHdr.sName, lpstrXmlPN);
	::WritePrivateProfileString("PRJ_HEADER", "PrjDirPath", m_projHdr.sDirPath, lpstrXmlPN); 
	sprintf(strValue, "%d", m_projHdr.projectID);
	::WritePrivateProfileString("PRJ_HEADER", "ProjectID", strValue, lpstrXmlPN);
	sprintf(strValue, "%d", m_projHdr.groupID);
	::WritePrivateProfileString("PRJ_HEADER", "GroupID", strValue, lpstrXmlPN);
	// write data file info
	::WritePrivateProfileString("DATA_FILE_INFO", "LASList", m_projHdr.lasListPath,  lpstrXmlPN);
	::WritePrivateProfileString("DATA_FILE_INFO", "IMGList", m_projHdr.imgListPath,  lpstrXmlPN);
	::WritePrivateProfileString("DATA_FILE_INFO", "CAMList", m_projHdr.camParPath, lpstrXmlPN);

	::WritePrivateProfileString("DATA_FILE_INFO", "GCDProd", m_projHdr.m_strProdGCDPN, lpstrXmlPN);
	::WritePrivateProfileString("DATA_FILE_INFO", "GCDGcp", m_projHdr.m_strGcpGCDPN, lpstrXmlPN);
	::WritePrivateProfileString("DATA_FILE_INFO", "GCD7Par", m_projHdr.m_str7parPN, lpstrXmlPN);

	if (save_regisprj) {
		char regis_path[_MAX_PATH];
		std::string value;
		if (!getMetaValue(REGISPRJ_PATH_KEY, value)) {
			sprintf(regis_path, "%s%s%s%s%s%s%s", m_projHdr.sDirPath, IF_PROCESS_DIR, "/", g_strTaskDirName[TASK_REGIS], "/", m_projHdr.sName, ".xml");
			addMetaValue(REGISPRJ_PATH_KEY, regis_path);
		}
		else {
			sprintf(regis_path, value.c_str());
		}

		if (!saveRegistrationProject(regis_path)) {
			m_error_info = "cannot save registration project to: " + std::string(regis_path);
			//return false;
		}
	}

	// write meta info
	::WritePrivateProfileString("META_INFO", "METAKEY", "", lpstrXmlPN);
	std::set<std::string> keys;
	for (auto & meta : m_meta_info) {
		::WritePrivateProfileString("META_INFO", meta.first.c_str(), meta.second.c_str(), lpstrXmlPN);
		keys.insert(meta.first);
	}
	std::string all_keys; 
	for (auto key : keys) {
		all_keys += key + ";";
	}
	::WritePrivateProfileString("META_INFO", "METAKEY", all_keys.c_str(), lpstrXmlPN);

	return true;
}

bool BlockDBaseIO::saveRegistrationProject(const char * path)
{
	CreateDir(GetFileDirectory(path));
	LasPrjIO regis_xml;
	regis_xml.m_PrjHdr.ProjectID = m_projHdr.projectID;
	regis_xml.m_PrjHdr.BlockID = m_projHdr.groupID;
	std::string meta_value;
	regis_xml.m_PrjHdr.m_fDemGSD = getMetaValue("DEMGSD", meta_value) ? atof(meta_value.c_str()) : 0;
	
	strcpy(regis_xml.m_PrjHdr.PrjPath, path);
	strcpy(regis_xml.m_PrjHdr.RegisPrjPath, path);
	strcpy(strrchr(regis_xml.m_PrjHdr.RegisPrjPath, '.'), ".regisprj");

	char dir[_MAX_PATH]; strcpy(dir, GetFileDirectory(path)); GetUnixDir(dir);
	
	sprintf(regis_xml.m_PrjHdr.CamParFilePath, "%s%s", dir, "Data/CamParFile.txt");		CreateDir(GetFileDirectory(regis_xml.m_PrjHdr.CamParFilePath));
	sprintf(regis_xml.m_PrjHdr.ImgListPath, "%s%s", dir, "Data/ImgListFile.txt");		CreateDir(GetFileDirectory(regis_xml.m_PrjHdr.ImgListPath));
	sprintf(regis_xml.m_PrjHdr.PosFilePath, "%s%s", dir, "Data/PosFile.txt");			CreateDir(GetFileDirectory(regis_xml.m_PrjHdr.PosFilePath));
	sprintf(regis_xml.m_PrjHdr.LasListPath, "%s%s", dir, "Data/LasFile.txt");			CreateDir(GetFileDirectory(regis_xml.m_PrjHdr.LasListPath));

	for (auto & info : m_ptClds) {
		LasStrip las_strip;
		sscanf(info.sID, "%d", &las_strip.nIdx);
		las_strip.BlockID = info.nGroupID;
		strcpy(las_strip.szPath, info.sPath); Unix2Dos(las_strip.szPath);
		strcpy(las_strip.szName, las_strip.szPath);
		strcpy(las_strip.szName, strrchr(las_strip.szName, '\\') + 1);

		regis_xml.m_LasInfo.push_back(las_strip);
	}

	for (size_t i = 0; i < m_cameras.size(); i++) {
		auto info = m_cameras[i];
		CameraData cam_data;
		cam_data.CameraID = i;	//!!!!
		strcpy(cam_data.CameraName, info.sName);
		cam_data.pixelSize = info.pixelSize;
		cam_data.width = info.width;
		cam_data.height = info.height;
		cam_data.f = info.f;
		cam_data.x0 = info.x0;
		cam_data.y0 = info.y0;
		for (size_t di = 0; di < 8; di++) {
			cam_data.distortionPar[di] = info.distortionPar[di];
		}
		for (size_t di = 0; di < 6; di++) {
			cam_data.CameraBias[di] = info.cameraBias[di];
		}

		regis_xml.m_CameraInfo.push_back(cam_data);
	}
	
	for (auto & info : m_images) {
		ImgData img_data;

		img_data.CameraID = -1;
		for (auto & cam : regis_xml.m_CameraInfo) {
			if (strcmpi(cam.CameraName, info.cameraName.c_str()) == 0) {
				img_data.CameraID = cam.CameraID;
				break;
			}
		}
		if (img_data.CameraID == -1) {
			continue;
		}
				
		strcpy(img_data.szImgPath, info.sPath); Unix2Dos(img_data.szImgPath);

		strcpy(img_data.szImgName, info.sPath); Unix2Dos(img_data.szImgName);
		strcpy(img_data.szImgName, strrchr(img_data.szImgName, '\\') + 1);

		sscanf(info.sID, "%d", &img_data.ImageID);
		img_data.BlockID = info.nGroupID;
		img_data.Lat = info.gpsLat;
		img_data.Lon = info.gpsLon;
		img_data.Height = info.gpsHeight;
		img_data.gps_time = info.gps_time;
		
		img_data.Xs = info.posXs;
		img_data.Ys = info.posYs;
		img_data.Zs = info.posZs;
		img_data.Phi = info.posPhi;
		img_data.Omega = info.posOmega;
		img_data.Kappa = info.posKappa;

		img_data.StripID = info.stripID;
		img_data.bFlag = info.bFlag;
		img_data.Attrib = info.attrib;

		
		int StripID, Attrib, CameraID, bFlag;
		
		regis_xml.m_ImgInfo.push_back(img_data);
	}
	
	if (!regis_xml.GeneratePrjFiles()) {
		m_error_info = regis_xml.getErrorMsg();
		return false;
	}

	return true;
}

void BlockDBaseIO::clear()
{
	m_ptClds.clear();
	m_images.clear();
	m_cameras.clear();
	m_meta_info.clear();

}

void BlockDBaseIO::setPath(const char * path)
{
	strcpy(m_project_path, path);
	char drive[_MAX_DRIVE], dir[_MAX_DIR], name[_MAX_FNAME], ext[_MAX_EXT];
	_splitpath(m_project_path, drive, dir, name, ext);
	sprintf(m_projHdr.sDirPath, "%s%s", drive, dir);
	sprintf(m_projHdr.sName, "%s", name);
}

bool BlockDBaseIO::getMetaValue(std::string key, std::string & value)
{
	return _getMetaValue(m_meta_info, key, value);
}

bool BlockDBaseIO::addMetaValue(std::string key, std::string value)
{
	m_meta_info.insert(std::make_pair(key, value));
	return true;
}

blkDataInfo* BlockDBaseIO::addData(blkDataInfo * info)
{
	if (!info) return nullptr;

	switch (info->dataType())
	{
	case Blk_PtCld:
		m_ptClds.push_back(*static_cast<blkPtCldInfo*>(info));
		return &m_ptClds.back();
	case Blk_Image:
		m_images.push_back(*static_cast<blkImageInfo*>(info));
		return &m_images.back();
	case Blk_Camera:
		m_cameras.push_back(*static_cast<blkCameraInfo*>(info));
		return &m_cameras.back();
	case Blk_Miscs:
	default:
		break;
	}
	return nullptr;
}

bool loadMetaListFile(const char* path, char* prefix, std::vector<strMetaMap>& meta_info)
{
	if (_access(path, 0) == -1) {
		g_error_info = "cannot load file: " + std::string(path);
		return false;
	}

	//! file header
	size_t count = ::GetPrivateProfileInt("FILE_HEADER", "Count", 0, path);
	::GetPrivateProfileString("FILE_HEADER", "PREFIX", "", prefix, 1024, path);

	//! start read item by item
	char strValue[1024];
	::GetPrivateProfileString("FILE_HEADER", "METAKEY", "", strValue, 1024, path);
	std::vector<std::string> keys = _splitString(strValue, ";");

	for (size_t i = 0; i < count; i++) {
		char appName[1024];
		sprintf(appName, "%s%d", prefix, i);

		strMetaMap info;
		memset(strValue, 0, 1024);
		for (auto & key : keys) {
			::GetPrivateProfileString(appName, key.c_str(), "", strValue, 1024, path);
			info.insert(std::make_pair(key, strValue));
		}
		meta_info.push_back(info);
	}

	return true;
}

bool saveMetaListFile(const char* path, const char* prefix, const std::vector<strMetaMap> & meta_info, bool overwrite)
{
	CreateDir(GetFileDirectory(path));
	if (overwrite) {
		FILE* fp = fopen(path, "w");
		if (!fp) { g_error_info = "cannot save to: " + std::string(path); return false; }
		fclose(fp);
	}
	else if (_access(GetFileDirectory(path), 0) == -1) {
		g_error_info = "wrong directory" + std::string(path); return false;
	}

	char strValue[1024];
	sprintf(strValue, "%d", meta_info.size());
	::WritePrivateProfileString("FILE_HEADER", "Count", strValue, path);
	::WritePrivateProfileString("FILE_HEADER", "PREFIX", prefix, path);

	//! start save item by item
	std::set<std::string> meta_keys;

	for (size_t i = 0; i < meta_info.size(); ++i) {
		char appName[1024];
		sprintf(appName, "%s%d", prefix, i);
		for (auto & info : meta_info[i]) {
			::WritePrivateProfileString(appName, info.first.c_str(), info.second.c_str(), path);
			meta_keys.insert(info.first);
		}
	}
	//! finally save the meta keys
	std::string meta_key;
	for (auto key : meta_keys) {
		meta_key += key + ";";
	}
	::WritePrivateProfileString("FILE_HEADER", "METAKEY", meta_key.c_str(), path);

	return true;
}

bool loadImages(std::vector<blkImageInfo>& images, std::vector<blkCameraInfo>& cameras,
	const char* img_list, const char* cam_list)
{
	char prefix[1024];
	std::vector<strMetaMap> meta_info_img;
	if (!loadMetaListFile(img_list, prefix, meta_info_img)) {
		return false;
	}

	for (auto & meta_info : meta_info_img) {
		blkImageInfo info;
		if (info.pullFromMeta(meta_info) || info.isValid())
			images.push_back(info);
	}

	std::vector<strMetaMap> meta_info_cam;
	if (!loadMetaListFile(cam_list, prefix, meta_info_cam)) {
		return false;
	}

	for (auto & meta_info : meta_info_cam) {
		blkCameraInfo info;
		if (info.pullFromMeta(meta_info) || info.isValid()) {
			cameras.push_back(info);
		}
	}

	return true;
}

bool saveImages(const std::vector<blkImageInfo>& images, const std::vector<blkCameraInfo>& cameras,
	const char* img_list, const char* cam_list)
{
	{
		FILE* fp = fopen(img_list, "w");
		if (!fp) {
			g_error_info = "failed to save images to: " + std::string(img_list);
			return false;
		}
		fclose(fp);
		fp = fopen(cam_list, "w");
		if (!fp) {
			g_error_info = "failed to save cameras to: " + std::string(cam_list);
			return false;
		}
		fclose(fp);
	}

	//! write image list
	std::vector<strMetaMap> meta_info_img;
	for (auto info : images) {
		strMetaMap meta_info;
		info.pushToMeta(meta_info);
		meta_info_img.push_back(meta_info);
	}

	//! camera
	std::vector<strMetaMap> meta_info_cam;
	for (auto info : cameras) {
		strMetaMap meta_info;
		info.pushToMeta(meta_info);
		meta_info_cam.push_back(meta_info);
	}

	return saveMetaListFile(img_list, "IMG_", meta_info_img, true)
		&& saveMetaListFile(cam_list, "CAM_", meta_info_cam, true);
}

bool loadPoints(std::vector<blkPtCldInfo>& ptClds, const char* path)
{
	char prefix[1024];
	std::vector<strMetaMap> meta_info;
	if (!loadMetaListFile(path, prefix, meta_info)) {
		return false;
	}
	for (auto & meta_info : meta_info) {
		blkPtCldInfo info;
		if (info.pullFromMeta(meta_info) || info.isValid()) {
			ptClds.push_back(info);
		}
	}

	return true;
}

bool savePoints(const std::vector<blkPtCldInfo>& ptClds, const char* path)
{	
	std::vector<strMetaMap> meta_info;
	for (auto & pc : ptClds) {
		strMetaMap info;
		pc.pushToMeta(info);
		meta_info.push_back(info);
	}
	return saveMetaListFile(path, "LAS_", meta_info, true);
}

bool getImageGPSInfo(const char* path, double& Lat, double& Lon, double& Height)
{
	LasPrjIO las_prj;
	return (las_prj.ReadImgGpsInfo(path, Lat, Lon, Height) == 1);
}

bool loadPosFile(const char* path, std::vector<blkImageInfo>& images_pos)
{
	FILE *fpos;
	fpos = fopen(path, "r");
	if (!fpos) { return false; }

	int nImgNum(-1);
	char szBuf[1024];
	char char_temp[1024];
	//	CString str_temp;

	fgets(szBuf, 1024, fpos);
	sscanf(szBuf, "%d", &nImgNum);
	if (nImgNum < 0) {
		return false;
	}
	for (size_t i = 0; i < nImgNum; i++)
	{
		blkImageInfo info;
		fgets(szBuf, 1024, fpos);
		sscanf(szBuf, "%s %lf %lf %lf %lf %lf %lf %lf",
			info.sName, &info.gps_time,
			&info.posXs, &info.posYs, &info.posZs,
			&info.posPhi, &info.posOmega, &info.posKappa);
		images_pos.push_back(info);
	}
	fclose(fpos);

	return true;
}

bool savePosFile(const char* path, const std::vector<blkImageInfo>& images_pos)
{
	FILE *fp;
	fp = fopen(path, "w");
	if (!fp)
	{
		return false;
	}
	fprintf(fp, "%d\n", images_pos.size());
	for (int i = 0; i < images_pos.size(); i++)
	{
		fprintf(fp, "%s %.10lf %.5lf %.5lf %.5lf %.10lf %.10lf %.10lf\n", images_pos[i].sName, images_pos[i].gps_time,
			images_pos[i].posXs, images_pos[i].posYs, images_pos[i].posZs, images_pos[i].posPhi,
			images_pos[i].posOmega, images_pos[i].posKappa);
	}

	fclose(fp);
	return true;
}

bool connectPgsDB(const char* connInfo)
{
	PGconn* Conn = PQconnectdb(connInfo);
	if (PQstatus(Conn) == CONNECTION_OK) {


	}
	return true;
}

BLKDB_NAMESPACE_END
