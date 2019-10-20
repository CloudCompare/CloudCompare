#include "LasPrjIO.h"
#include"Markup.h"
#include"Exif.h"
#include "omp.h" 
#include <io.h>
#include <windows.h>
using namespace std;

double pi = 3.14159265358979;

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

LasPrjIO::LasPrjIO()
{
	m_CameraInfo.reserve(m_MaxCamCount);
	m_ImgInfo.reserve(m_MaxImgCount);
	m_LasInfo.reserve(m_MaxLasCount);
}

LasPrjIO::~LasPrjIO()
{
}

bool LasPrjIO::WritePrjFile(PrjHdr sPrjHdr)
{
	//char strSynPath[MAX_PATHNAME];  strcpy(strSynPath, lpstrPath);
	//char strImpPath[MAX_PATHNAME];	if (IsExist(m_strImpFilePath)) strcpy(strImpPath, m_strImpFilePath);	else{ GetImpFilePath(lpstrPath, strImpPath); Dos2Unix(strImpPath); }
	//char strExpPath[MAX_PATHNAME];	if (IsExist(m_strExpFilePath)) strcpy(strExpPath, m_strExpFilePath);	else{ GetExpFilePath(lpstrPath, strExpPath); Dos2Unix(strExpPath); }
	//char strTskPath[MAX_PATHNAME];	if (IsExist(m_strTskFilePath)) strcpy(strTskPath, m_strTskFilePath);	else{ GetTskFilePath(lpstrPath, strTskPath); Dos2Unix(strTskPath); }

	//TRACE("输出xml文件！\n");
	
	CMarkup xml;
	char str_temp[64];
	sprintf(str_temp, "%.5lf", sPrjHdr.m_fDemGSD);
	xml.SetDoc("<?xml version=\"1.0\" encoding=\"GB2312\" standalone=\"yes\"?>\r\n");
	xml.AddElem("TargetDetectFile");
	xml.IntoElem();
	{
		xml.AddElem("ProjectHeader");
		xml.IntoElem();
		xml.AddElem("PrjPath", sPrjHdr.PrjPath);
		xml.AddElem("PrjID", sPrjHdr.ProjectID);
		xml.AddElem("BlockID", sPrjHdr.BlockID);
		xml.AddElem("RegisPrjPath", sPrjHdr.RegisPrjPath);
		xml.AddElem("DemGSD", str_temp);
		xml.OutOfElem();
		xml.AddElem("FileBody");
		xml.IntoElem();
		{
			xml.AddElem("Target");
			xml.IntoElem();
			{
				xml.AddElem("targetID", "0220");
				xml.AddElem("LasDataInfo");
				xml.IntoElem();
				xml.AddElem("LasDataListFPath", sPrjHdr.LasListPath);
				xml.OutOfElem();
				xml.AddElem("ImgDataInfo");
				xml.IntoElem();
				{
					xml.AddElem("ImgDataListFPath", sPrjHdr.ImgListPath);
					xml.AddElem("ImgDataCmrFPath", sPrjHdr.CamParFilePath);
					xml.AddElem("POSDataFPath", sPrjHdr.PosFilePath);
				}				
				xml.OutOfElem();
			}
			xml.OutOfElem();
		}
		xml.OutOfElem();		
	}
	xml.OutOfElem();
	
	if (!xml.Save(sPrjHdr.PrjPath)) { printf("工程xml文件生成失败！\n"); return false; }

	return true;
}

int LasPrjIO::WriteLasList(char* lasListPath, LasInfo sLasInfo)
{
	FILE *fp;
	fp = fopen(lasListPath, "w");
	if (!fp)
	{
		m_error_msg = "点云列表文件生成失败!";
		return 0;
	}
	fprintf(fp, "%d\n", sLasInfo.size());
	//fprintf(fp, "%s\n", sPrjHdr.LasDir);
	for (int i = 0; i < sLasInfo.size(); i++)
		fprintf(fp, "%d   %s  %f %d\n", sLasInfo[i].nIdx, sLasInfo[i].szPath, sLasInfo[i].dzoffset,sLasInfo[i].BlockID);

	fclose(fp);
	return 1;
}

int LasPrjIO::WriteImgList(char* imgListPath, ImgInfo sImgInfo)
{
	int nMinID = 999999;
	int nMaxID = -1;
	for (int i = 0; i < sImgInfo.size(); i++)
	{
		if (sImgInfo[i].ImageID < nMinID)
			nMinID = sImgInfo[i].ImageID;
		if (sImgInfo[i].ImageID > nMaxID)
			nMaxID = sImgInfo[i].ImageID;
	}
	int nflag = -1;
	if (nMinID == 0 && nMaxID == (sImgInfo.size() - 1))
		nflag = 0;
	else if (nMinID == 1 && nMaxID == sImgInfo.size())
		nflag = 1;
	else
		nflag = -1;
	
	if (nflag == 0)
	{
		ImgInfo vT;
		vT.resize(sImgInfo.size());
		for (int i = 0; i < sImgInfo.size(); i++)
		{
			vT[sImgInfo[i].ImageID] = sImgInfo[i];
		}

		FILE *fp;
		fp = fopen(imgListPath, "w");
		if (!fp)
		{
			m_error_msg = "影像列表文件生成失败!";
			return 0;
		}
		fprintf(fp, "%d\n", vT.size());
		//fprintf(fp, "%s\n", sPrjHdr.ImgDir);
		for (int i = 0; i < vT.size(); i++)
		{
			/*fprintf(fp,"%d ", sImgInfo[i].ImageID);
			fprintf(fp, " ");*/
			fprintf(fp, "%6d %s %5d %5d\n", vT[i].ImageID, vT[i].szImgPath, vT[i].CameraID, vT[i].BlockID);
		}
		//fprintf(fp, "%d  %s  %d\n", sImgInfo[i].ImageID, sImgInfo[i].szImgPath, sImgInfo[i].BlockID);

		fclose(fp);
	}
	else if (nflag == 1)
	{
		ImgInfo vT;
		vT.resize(sImgInfo.size());
		for (int i = 0; i < sImgInfo.size(); i++)
		{
			vT[sImgInfo[i].ImageID - 1] = sImgInfo[i];
		}
		
		FILE *fp;
		fp = fopen(imgListPath, "w");
		if (!fp)
		{
			m_error_msg = "影像列表文件生成失败!";
			return 0;
		}
		fprintf(fp, "%d\n", vT.size());
		//fprintf(fp, "%s\n", sPrjHdr.ImgDir);
		for (int i = 0; i < vT.size(); i++)
		{
			/*fprintf(fp,"%d ", sImgInfo[i].ImageID);
			fprintf(fp, " ");*/
			fprintf(fp, "%6d %s %5d %5d\n", vT[i].ImageID, vT[i].szImgPath, vT[i].CameraID, vT[i].BlockID);
		}
		//fprintf(fp, "%d  %s  %d\n", sImgInfo[i].ImageID, sImgInfo[i].szImgPath, sImgInfo[i].BlockID);

		fclose(fp);
	}
	else
	{
		FILE *fp;
		fp = fopen(imgListPath, "w");
		if (!fp)
		{
			m_error_msg = "影像列表文件生成失败!";
			return 0;
		}
		fprintf(fp, "%d\n", sImgInfo.size());
		//fprintf(fp, "%s\n", sPrjHdr.ImgDir);
		for (int i = 0; i < sImgInfo.size(); i++)
		{
			/*fprintf(fp,"%d ", sImgInfo[i].ImageID);
			fprintf(fp, " ");*/
			fprintf(fp, "%6d %s %5d %5d\n", sImgInfo[i].ImageID, sImgInfo[i].szImgPath, sImgInfo[i].CameraID, sImgInfo[i].BlockID);
		}
		//fprintf(fp, "%d  %s  %d\n", sImgInfo[i].ImageID, sImgInfo[i].szImgPath, sImgInfo[i].BlockID);

		fclose(fp);
	}
	
	
	
	return 1;
}

int LasPrjIO::WriteCamFile(char* camPath, CameraInfo sCamInfo)
{
	FILE *fp;
	fp = fopen(camPath, "w");
	if (!fp)
	{
		m_error_msg = "相机文件生成失败!";
		return 0;
	}
	//fprintf(fp, "%d\r\n", sCamInfo.CameraList.size());
	for (int i = 0; i < sCamInfo.size(); i++)
	{
		fprintf(fp, "%s\n", sCamInfo[i].CameraName);
		fprintf(fp, "%.7lf\n", sCamInfo[i].pixelSize);
		fprintf(fp, "%d\n", sCamInfo[i].width);
		fprintf(fp, "%d\n", sCamInfo[i].height);
		fprintf(fp, "%.6lf\n", sCamInfo[i].f);
		fprintf(fp, "%lf  %lf\n", sCamInfo[i].x0,sCamInfo[i].y0);
		for (int j = 0; j < 8; j++)
			fprintf(fp, "%lf ", sCamInfo[i].distortionPar[j]);
		fprintf(fp, "%s", "\n");
		for (int j = 0; j < 6; j++)
		{
			fprintf(fp, "%lf ", sCamInfo[i].CameraBias[j]);
			if (j == 2||j==5)
				fprintf(fp, "\n");
		}
	}
		
	fclose(fp);
	return 1;
}

int LasPrjIO::WritePosFile(char* posPath, ImgInfo sImgInfo)
{
	FILE *fp;
	fp = fopen(posPath, "w");
	if (!fp)
	{
		m_error_msg = "Pos文件生成失败!";
		return 0;
	}
	fprintf(fp, "%d\n", sImgInfo.size());
	for (int i = 0; i < sImgInfo.size(); i++)
	{
		fprintf(fp, "%s %.10lf %.5lf %.5lf %.5lf %.10lf %.10lf %.10lf\n", sImgInfo[i].szImgName, sImgInfo[i].gps_time,
			sImgInfo[i].Xs, sImgInfo[i].Ys, sImgInfo[i].Zs, sImgInfo[i].Phi,
			sImgInfo[i].Omega, sImgInfo[i].Kappa);
	}
	
	fclose(fp);
	return 1;
}

bool LasPrjIO::GeneratePrjFiles()
{
	char LasListPath[_MAX_PATH];
	char ImgListPath[_MAX_PATH];
	char CamFilePath[_MAX_PATH];
	char PosFilePath[_MAX_PATH];
	strcpy(LasListPath,m_PrjHdr.LasListPath);
	strcpy(ImgListPath, m_PrjHdr.ImgListPath);
	strcpy(PosFilePath, m_PrjHdr.PosFilePath);
	strcpy(CamFilePath, m_PrjHdr.CamParFilePath);

	//生成xml工程文件
	if (!WritePrjFile(m_PrjHdr))
	{
		m_error_msg = "工程创建失败!";
		return false;
	}

	//生成各类输入数据文件
	if (WriteLasList(LasListPath, m_LasInfo)*WriteImgList(ImgListPath, m_ImgInfo)
		*WriteCamFile(CamFilePath, m_CameraInfo)*WritePosFile(PosFilePath, m_ImgInfo) == 0)
	{
		m_error_msg = "工程创建失败!";
		return false;
	}

	return true;
}

void LasPrjIO::trim(char *strIn, char *strOut) {

	int i, j;
	i = 0;
	j = strlen(strIn) - 1;
	while (strIn[i] == ' ')
		++i;
	while (strIn[j] == ' ')
		--j;
	strncpy(strOut, strIn + i, j - i + 1);
	strOut[j - i + 1] = '\0';
}

int LasPrjIO::ReadLasList(char* sLasLstPath, LasInfo& sLasInfo,PrjHdr& sPrjHdr)
{	
	LasInfo lasInfo;
	if (ReadLasList(sLasLstPath, lasInfo) == 0)
		return 0;

	for (auto & las_info : lasInfo) {
		if (las_info.BlockID == sPrjHdr.BlockID) {
			sLasInfo.push_back(las_info);
		}
	}

	return 1;
}

int LasPrjIO::ReadLasList(char* sLasLstPath, LasInfo& sLasInfo)
{
	FILE *flas;
	flas = fopen(sLasLstPath, "r");
	if (!flas)
	{
		m_error_msg = std::string("无法打开: ") + sLasLstPath;
		return 0;
	}
	int nLasNum;
	char szBuf[1024];
	fgets(szBuf, 1024, flas);
	sscanf(szBuf, "%d", &nLasNum);

	char LasName[_MAX_PATH];
	char Lasdrive[_MAX_DRIVE];
	char Lasdir[_MAX_DIR];
	char Lasext[_MAX_EXT];
	char Lasfname[_MAX_FNAME];
	char szbuf[1024];
// 	char szbuf2[1024];
// 	char temp_left[1024];
// 	char temp_middle[1024];
// 	char temp_right[1024];
// 	char char_temp[1024];
// 	CString str_temp;

	for (int i = 0; i < nLasNum; i++)
	{
		LasStrip lstemp;
		fgets(szBuf, 1024, flas);
		sscanf(szBuf, "%d%s%f%d\n", &lstemp.nIdx, lstemp.szPath, &lstemp.dzoffset, &lstemp.BlockID);
		
// 		fgets(szbuf2, 1024, flas);
// 		trim(szbuf2, szbuf);  //去除路径首尾空格
// 		str_temp = szbuf;
// 		str_temp.TrimRight();  //消除右侧空格
// 		strcpy(temp_right, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		lstemp.BlockID = atoi(temp_right);// las blockID
// 
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 		str_temp = char_temp;
// 		str_temp.TrimRight();
// 		strcpy(temp_right, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		lstemp.dzoffset = atof(temp_right); // dzoffset
// 
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));  //dzoffset左侧
// 		str_temp = char_temp;
// 		str_temp.TrimRight();
// 
// 		strcpy(temp_left, str_temp.Left(str_temp.Find(' ')));
// 		strcpy(temp_middle, str_temp.Right(str_temp.GetLength() - 1 - str_temp.Find(' ')));
// 		lstemp.nIdx = atoi(temp_left);
// 		//lstemp.BlockID = atoi(temp_right);
// 		trim(temp_middle, lstemp.szPath);  //去除路径首尾空格

		//fscanf(flas, "%d%s%d", &lstemp.nIdx, lstemp.szPath, &lstemp.BlockID);
		_splitpath(lstemp.szPath, Lasdrive, Lasdir, Lasfname, Lasext);
		sprintf(lstemp.szName, "%s%s", Lasfname, Lasext);

		sLasInfo.push_back(lstemp);
	}
	fclose(flas);

	return 1;
}

int LasPrjIO::ReadImgList(char* sImgLstPath, ImgInfo& sImgInfo,PrjHdr& sPrjHdr)
{
	ImgInfo imgInfo;
	if (ReadImgList(sImgLstPath, imgInfo) == 0)
		return 0;

	for (auto & img_info : imgInfo) {
		if (img_info.BlockID == sPrjHdr.BlockID) {
			sImgInfo.push_back(img_info);
		}
	}

	return 1;
}

int LasPrjIO::ReadImgList(char* sImgLstPath, ImgInfo& sImgInfo)
{
	FILE *fimg;
	fimg = fopen(sImgLstPath, "r");
	if (!fimg)
	{
		m_error_msg = std::string("无法打开: ") + sImgLstPath;
		return 0;
	}
	int nImgNum;	
	//fscanf(fimg, "%d", &nImgNum);

//	char ImgName[_MAX_PATH];
	char Imgdrive[_MAX_DRIVE];
	char Imgdir[_MAX_DIR];
	char Imgext[_MAX_EXT];
	char Imgfname[_MAX_FNAME];
	char szbuf[1024];
// 	char szbuf2[1024];
// 	char temp_left[1024];
// 	char temp_middle[1024];
// 	char temp_right[1024];
// 	char char_temp[1024];
// 	CString str_temp;

	fgets(szbuf, 1024, fimg);
	sscanf(szbuf, "%d", &nImgNum);

	for (int i = 0; i < nImgNum; i++)
	{
		ImgData img_temp;
		//fscanf(fimg, "%d%s%d", &img_temp.ImageID, img_temp.szImgPath, &img_temp.BlockID);

		fgets(szbuf, 1024, fimg);

		sscanf(szbuf, "%d%s%d%d", &img_temp.ImageID, img_temp.szImgPath, &img_temp.CameraID, &img_temp.BlockID);

// 		trim(szbuf2, szbuf);
// 		str_temp = szbuf;
// 		str_temp.TrimRight();  //消除右侧空格
// 		strcpy(temp_right, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		img_temp.BlockID = atoi(temp_right); //blockid
// 
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 		str_temp = char_temp; 
// 		str_temp.TrimRight();  //消除右侧空格
// 		strcpy(char_temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		img_temp.CameraID = atof(char_temp); //camid
// 		
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 		str_temp = char_temp;
// 		str_temp.TrimRight();  //消除右侧空格
// 		strcpy(temp_left, str_temp.Left(str_temp.Find(' ')));
// 		strcpy(temp_middle, str_temp.Right(str_temp.GetLength() - 1 - str_temp.Find(' ')));
// 		img_temp.ImageID = atoi(temp_left);
// 		trim(temp_middle, img_temp.szImgPath);  //去除路径首尾空格
// 
		_splitpath(img_temp.szImgPath, Imgdrive, Imgdir, Imgfname, Imgext);
 		sprintf(img_temp.szImgName, "%s%s", Imgfname, Imgext);

		sImgInfo.push_back(img_temp);
	}
	fclose(fimg);

	return 1;
}

int LasPrjIO::ReadPosFile(char* sPosPath, ImgInfo& sImgInfo)
{
	//strcpy(sPrjHdr.PosFilePath,sPosPath);
	FILE *fpos;
	fpos = fopen(sPosPath, "r");
	if (!fpos)
	{
		m_error_msg = std::string("无法打开: ") + sPosPath;
		return 0;
	}
	int nImgNum;
	char szBuf[1024];
	char szBuf2[1024];
	char char_temp[1024];
//	CString str_temp;

	fgets(szBuf, 1024, fpos); 
	sscanf(szBuf, "%d", &nImgNum);
	
	for (int i = 0; i < nImgNum; i++)
	{
		ImgData img_temp;
		fgets(szBuf2, 1024, fpos);
		sscanf(szBuf2, "%s %lf %lf %lf %lf %lf %lf %lf",
			img_temp.szImgName, &img_temp.gps_time,
			&img_temp.Xs, &img_temp.Ys, &img_temp.Zs, &img_temp.Phi,
			&img_temp.Omega, &img_temp.Kappa);
		
// 		trim(szBuf2, szBuf);
// 		str_temp = szBuf; //kappa
// 		str_temp.TrimRight();  //消除右侧空格
// 		strcpy(char_temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' '))); 
// 		img_temp.Kappa = atof(char_temp);
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 		str_temp = char_temp;  //omega
// 		str_temp.TrimRight(" ");
// 		strcpy(char_temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		img_temp.Omega = atof(char_temp);
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 		str_temp = char_temp;  //Phi
// 		str_temp.TrimRight(" ");
// 		strcpy(char_temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		img_temp.Phi = atof(char_temp);
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 		str_temp = char_temp;  //Z
// 		str_temp.TrimRight(" ");
// 		strcpy(char_temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		img_temp.Zs = atof(char_temp);
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 		str_temp = char_temp;  //Y
// 		str_temp.TrimRight(" ");
// 		strcpy(char_temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		img_temp.Ys = atof(char_temp);
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 		str_temp = char_temp;  //X
// 		str_temp.TrimRight(" ");
// 		strcpy(char_temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		img_temp.Xs = atof(char_temp);
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 		str_temp = char_temp;  //gpstime
// 		str_temp.TrimRight(" ");
// 		strcpy(char_temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		img_temp.gps_time = atof(char_temp);
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 		trim(char_temp, img_temp.szImgName);    //name

		//将外方位元素存入同名影像中
		for (int j = 0; j < sImgInfo.size();j++)
			if (strcmp(sImgInfo[j].szImgName, img_temp.szImgName)==0)
			{
				sImgInfo[j].gps_time = img_temp.gps_time;
				sImgInfo[j].Xs = img_temp.Xs;
				sImgInfo[j].Ys = img_temp.Ys;
				sImgInfo[j].Zs = img_temp.Zs;
				sImgInfo[j].Phi = img_temp.Phi;
				sImgInfo[j].Omega = img_temp.Omega;
				sImgInfo[j].Kappa = img_temp.Kappa;
				break;
			}
	}

	/*if(sImgInfo.size()== nImgNum)
		for (int i = 0; i<nImgNum; i++)
			fscanf(fpos, "%s %lf %lf %lf %lf %lf %lf %lf",
				sImgInfo[i].szImgName, &sImgInfo[i].gps_time,
				&sImgInfo[i].Xs, &sImgInfo[i].Ys, &sImgInfo[i].Zs, &sImgInfo[i].Phi,
				&sImgInfo[i].Omega, &sImgInfo[i].Kappa);
	else if (sImgInfo.size() == 0)
	{
		for (int i = 0; i < nImgNum; i++)
		{
			ImgData imgdt_temp;
			fscanf(fpos, "%s %lf %lf %lf %lf %lf %lf %lf",
				imgdt_temp.szImgName, &imgdt_temp.gps_time,
				&imgdt_temp.Xs, &imgdt_temp.Ys, &imgdt_temp.Zs, &imgdt_temp.Phi,
				&imgdt_temp.Omega, &imgdt_temp.Kappa);
			sImgInfo.push_back(imgdt_temp);
		}		
	}*/
	
	fclose(fpos);
	return 1;
}
int LasPrjIO::ReadCamFile(char* camPath, CameraInfo& sCamInfo)
{
	sCamInfo.clear();
	CameraData tempCamera;
	FILE*fp;
	fopen_s(&fp, camPath, "rt");
	if (!fp)
	{
		return 0;
	}
	char tempChar[256];
	int myLength = 9;
	int tempLength = 0;
	while (!feof(fp))
	{

		tempLength++;
		if (tempLength%myLength == 1)
		{
			fscanf_s(fp, "%s", tempCamera.CameraName, 256);
		}
		if (tempLength%myLength == 2)
		{
			fscanf_s(fp, "%lf", &tempCamera.pixelSize);
		}
		if (tempLength%myLength == 3)
		{

			fscanf_s(fp, "%d", &tempCamera.width);
		}
		if (tempLength%myLength == 4)
		{

			fscanf_s(fp, "%d", &tempCamera.height);
		}
		if (tempLength%myLength == 5)
		{
			fscanf_s(fp, "%lf", &tempCamera.f);
		}
		if (tempLength%myLength == 6)
		{
			fscanf_s(fp, "%lf %lf", &tempCamera.x0, &tempCamera.y0);
		}
		if (tempLength%myLength == 7)
		{
			fscanf_s(fp, "%lf %lf %lf %lf %lf %lf %lf %lf", &tempCamera.distortionPar[0], &tempCamera.distortionPar[1], &tempCamera.distortionPar[2], &tempCamera.distortionPar[3], &tempCamera.distortionPar[4], &tempCamera.distortionPar[5], &tempCamera.distortionPar[6], &tempCamera.distortionPar[7]);
		}
		if (tempLength%myLength == 8)
		{
			fscanf_s(fp, "%lf %lf %lf", &tempCamera.CameraBias[0], &tempCamera.CameraBias[1], &tempCamera.CameraBias[2]);
		}
		if (tempLength%myLength == 0)
		{
			tempCamera.CameraID = sCamInfo.size();
			fscanf_s(fp, "%lf %lf %lf", &tempCamera.CameraBias[3], &tempCamera.CameraBias[4], &tempCamera.CameraBias[5]);
			sCamInfo.push_back(tempCamera);
		}
	}
	fclose(fp);
	return 1;
}

int LasPrjIO::ReadPrjFile(char* lpXMLPath, PrjHdr& sPrjHdr)
{
	//TRACE("解析xml文件！\n");
	CMarkup xml; std::string strTagName, strData;
	if (!xml.Load(lpXMLPath))
	{
		m_error_msg = "xml文件打开错误!";
		return false;
	}

	xml.ResetMainPos();
	xml.FindElem(); //TargetDetectFile
	xml.IntoElem();
	xml.FindElem();//FileHeader
	xml.IntoElem();
	{
		if (xml.FindElem("PrjPath"))
		{
			strTagName = xml.GetTagName();
			strData = xml.GetData();
			//TRACE("\n---tagName:%s,Data:%s--\n", strTagName, strData);
			strcpy(sPrjHdr.PrjPath, strData.c_str());
		}
		else
		{
			m_error_msg = "不识别的xml文件!";
			return false;
		}

		if (xml.FindElem("PrjID"))
		{
			strTagName = xml.GetTagName();
			strData = xml.GetData();
			//TRACE("\n---tagName:%s,Data:%s--\n", strTagName, strData);
			sPrjHdr.ProjectID = atoi(strData.c_str());
		}
		else
		{
			m_error_msg = "不识别的xml文件!";
			return false;
		}

		if (xml.FindElem("BlockID"))
		{
			strTagName = xml.GetTagName();
			strData = xml.GetData();
			//TRACE("\n---tagName:%s,Data:%s--\n", strTagName, strData);
			sPrjHdr.BlockID = atoi(strData.c_str());
		}
		else
		{
			m_error_msg = "不识别的xml文件!";
			return false;
		}
		//配准工程路径
		if (xml.FindElem("RegisPrjPath"))
		{
			strTagName = xml.GetTagName();
			strData = xml.GetData();
			//TRACE("\n---tagName:%s,Data:%s--\n", strTagName, strData);
			strcpy(sPrjHdr.RegisPrjPath, strData.c_str());
		}
		else
		{
			m_error_msg = "不识别的xml文件!";
			return false;
		}

		if (xml.FindElem("DemGSD"))
		{
			strTagName = xml.GetTagName();
			strData = xml.GetData();
			//TRACE("\n---tagName:%s,Data:%s--\n", strTagName, strData);
			sPrjHdr.m_fDemGSD = atof(strData.c_str());
		}
		else
		{
			m_error_msg = "不识别的xml文件!";
			return false;
		}
	}	
	xml.OutOfElem();

	xml.FindElem();//FileBody
	xml.IntoElem();
	xml.FindElem();//Target
	xml.IntoElem();

	if (xml.FindElem("targetID"))
	{
		strTagName = xml.GetTagName();
		strData = xml.GetData();
		//TRACE("\n---tagName:%s,Data:%s--\n", strTagName, strData);
		if (strData != "0220")
		{
			m_error_msg = "不识别的xml文件!";
			return false;
		}
	}
	else
	{
		m_error_msg = "不识别的xml文件!";
		return false;
	}
	xml.FindElem();
	xml.IntoElem();

	//char path_temp[_MAX_PATH];
	if (xml.FindElem("LasDataListFPath"))
	{
		strTagName = xml.GetTagName();
		strData = xml.GetData();
		//TRACE("\n---tagName:%s,Data:%s--\n", strTagName, strData);
		strcpy(sPrjHdr.LasListPath, strData.c_str());
	}
	else
	{
		m_error_msg = "不识别的xml文件!";
		return false;
	}
	xml.OutOfElem();

	xml.FindElem();
	xml.IntoElem();

	if (xml.FindElem("ImgDataListFPath"))
	{
		strTagName = xml.GetTagName();
		strData = xml.GetData();
		//TRACE("\n---tagName:%s,Data:%s--\n", strTagName, strData);
		strcpy(sPrjHdr.ImgListPath, strData.c_str());
		//ImgListPath = path_temp;
	}
	else
	{
		m_error_msg = "不识别的xml文件!";
		return false;
	}

	if (xml.FindElem("ImgDataCmrFPath"))
	{
		strTagName = xml.GetTagName();
		strData = xml.GetData();
		//TRACE("\n---tagName:%s,Data:%s--\n", strTagName, strData);
		strcpy(sPrjHdr.CamParFilePath, strData.c_str());
		//CamFilePath = path_temp;
	}
	else
	{
		m_error_msg = "不识别的xml文件!";
		return false;
	}
	//
	if (xml.FindElem("POSDataFPath"))
	{
		strTagName = xml.GetTagName();
		strData = xml.GetData();
		//TRACE("\n---tagName:%s,Data:%s--\n", strTagName, strData);
		strcpy(sPrjHdr.PosFilePath, strData.c_str());
		//PosFilePath = path_temp;
	}
	else
	{
		m_error_msg = "不识别的xml文件!";
		return false;
	}
	return true;
}

int LasPrjIO::LoadPrjFiles(char* lpXMLPath)
{
	if (!ReadPrjFile(lpXMLPath,m_PrjHdr))
	{
		m_error_msg = "工程XML文件读取失败!";
		return 0;
	}

	if (strcmp(m_PrjHdr.ImgListPath, "") == 0 || strcmp(m_PrjHdr.ImgListPath, "NULL") == 0)
	{
		if (ReadLasList(m_PrjHdr.LasListPath, m_LasInfo, m_PrjHdr)== 0)
		{
			m_error_msg = "工程数据载入失败!";
			return 0;
		}
	}
	else
	{
		if (ReadImgList(m_PrjHdr.ImgListPath, m_ImgInfo, m_PrjHdr) == 0)
		{
			m_error_msg = "工程数据载入失败!";
			return 0;
		}
		if (ReadLasList(m_PrjHdr.LasListPath, m_LasInfo, m_PrjHdr)
			*ReadCamFile(m_PrjHdr.CamParFilePath, m_CameraInfo)
			*ReadPosFile(m_PrjHdr.PosFilePath, m_ImgInfo) == 0)
		{
			m_error_msg = "工程数据载入失败!";
			return 0;
		}
	}
	
	return 1;
}

int LasPrjIO::LoadPrjFiles2(char* lpXMLPath)
{
	if (!ReadPrjFile(lpXMLPath, m_PrjHdr))
	{
		m_error_msg = "工程XML文件读取失败!";
		return 0;
	}

	if (strcmp(m_PrjHdr.ImgListPath, "") == 0 || strcmp(m_PrjHdr.ImgListPath, "NULL") == 0)
	{
		if (ReadLasList(m_PrjHdr.LasListPath, m_LasInfo)== 0)
		{
			m_error_msg = "工程数据载入失败!";
			return 0;
		}
	}
	else
	{
		if (ReadImgList(m_PrjHdr.ImgListPath, m_ImgInfo) == 0)
		{
			m_error_msg = "工程数据载入失败!";
			return 0;
		}
		if (ReadLasList(m_PrjHdr.LasListPath, m_LasInfo)
			*ReadCamFile(m_PrjHdr.CamParFilePath, m_CameraInfo)
			*ReadPosFile(m_PrjHdr.PosFilePath, m_ImgInfo) == 0)
		{
			m_error_msg = "工程数据载入失败!";
			return 0;
		}
	}
	
	return 1;
}

//读取DPGrid工程相关文件
int LasPrjIO::ReadDPrjInfo(char* lpDppFPath)
{
	char szDpCamFPath[_MAX_PATH];
	char szDpImgLocFPath[_MAX_PATH];
	char szDpVpsFPath[_MAX_PATH];
	char szDrive[_MAX_DRIVE];
	char szDir[_MAX_DIR];
	char szFname[_MAX_FNAME];
	char szExt[_MAX_EXT];
	_splitpath(lpDppFPath,szDrive,szDir,szFname,szExt);
	sprintf(szDpVpsFPath, "%s%s%s.vps", szDrive,szDir,szFname);

	char szImgDir[_MAX_PATH];
	sprintf(szImgDir, "%s%sImages\\",szDrive,szDir);
	sprintf(szDpCamFPath, "%sCamera.cam", szImgDir);
	sprintf(szDpImgLocFPath, "%sImgFile.loc", szImgDir);

	//读取dpp文件
	if (!ReadDppFile(lpDppFPath, m_ImgInfo))
		return 0;
	//读取vps文件
	if (!ReadDPVpsFile(szDpVpsFPath, m_ImgInfo))
		return 0;
	//读取相机文件
	if (!ReadDPCamFile(szDpCamFPath, m_CameraInfo))
		return 0;
	//读取影像Location文件
	if (!ReadDPImgLocFile(szDpImgLocFPath, m_ImgInfo))
		return 0;
	//读取影像Dpi文件
	char szDpiFPath[_MAX_PATH];
	char szDpiFName[_MAX_PATH];
	for (int i = 0; i < m_ImgInfo.size(); i++)
	{
		sprintf(szDpiFName, "%s.dpi", m_ImgInfo[i].szImgName);
		sprintf(szDpiFPath, "%s%s", szImgDir, szDpiFName);
		if (!ReadDPDpiFile(szDpiFPath, m_ImgInfo[i], m_CameraInfo))
			return 0;
	}
	
	return 1;
}

int LasPrjIO::ReadDppFile(char* lpDppFPath,ImgInfo& vImg)
{
	FILE *fDpp;
	fDpp = fopen(lpDppFPath, "r");
	if (fDpp == NULL)
	{
		m_error_msg = "Can not open dpp file!";
		return 0;
	}
	int ImgCount = 0;
	
	char strLine[1024];
	::GetPrivateProfileString("IMAGE_INF", "imagesCount", 0, strLine, 1024, lpDppFPath);
	sscanf(strLine, "%d", &ImgCount);
	if (ImgCount == 0) {
		return 1;
	}
	else if (ImgCount < 0) return 0;

	std::string str_temp;
	//for (int i = 0; i < 23; i++)
	for (int i = 0; i < 30; i++)
	{
		fgets(strLine, 1024, fDpp);
		if (strcmp(strLine, "[IMAGE_INF]\n") == 0)
			break;
	}

	fgets(strLine, 1024, fDpp);
	fgets(strLine, 1024, fDpp);

	char strImgName[256];
	char strImgID[256];
	//CString strName;
	for (int i = 0; i < ImgCount; i++)
	{
		ImgData Img_temp;
		fgets(strLine, 1024, fDpp);
		std::vector<std::string> subSeps = _splitString(strLine, "=");
		if (subSeps.size() < 2) {
			continue;
		}
		strcpy(strImgID, subSeps[0].c_str());
		sscanf(subSeps[1].c_str(), "%s", Img_temp.szImgName);
		
		subSeps = _splitString(strImgID, "_");
		if (subSeps.size() < 2) {
			continue;
		}
		sscanf(subSeps.back().c_str(), "%d", &Img_temp.ImageID);
		Img_temp.ImageID++; //dpp文件影像id从0编号 Adjustment从1编号


// 		char szTemp[1024];
// 		int nFlag = 1;
// 		for (int j = 0; j < 3; j++)
// 		{
// 			str_temp = strLine;
// 			str_temp.TrimRight();  //消除右侧空格
// 
// 			if (j == 0)
// 			{
// 				//int a = str_temp.GetLength() - 1;
// 				string stemp;
// 				stemp = str_temp[str_temp.GetLength() - 1];
// 				//sprintf(szTemp, "%s", str_temp[str_temp.GetLength() - 1]);
// 				if (stemp == "=")
// 				{
// 					nFlag = -1;
// 					break;
// 				}
// 			}
// 
// 			strcpy(strLine, str_temp.Left(/*str_temp.GetLength() - 1 - */str_temp.ReverseFind('\t')));
// 		}
// 
// 		if (nFlag == -1)
// 			continue;
// 
// 		str_temp = strLine;
// 		str_temp.TrimRight();
// 		strcpy(strImgID, str_temp.Left(str_temp.Find('=')));
// 		strcpy(strImgName, str_temp.Right(str_temp.GetLength() - 1 - str_temp.Find('=')));
// 
// 		strName = strImgName;
// 		strName.TrimRight();
// 		strcpy(Img_temp.szImgName, strName); //影像名
// 
// 
// 		str_temp = strImgID;
// 		strcpy(strLine, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('_')));
// 
// 		Img_temp.ImageID = atoi(strLine);  //影像ID
// 		Img_temp.ImageID++;  //dpp文件影像id从0编号 Adjustment从1编号

		vImg.push_back(Img_temp);
	}
	fclose(fDpp);

	return 1;
}

int LasPrjIO::ReadDPCamFile(char* lpDPCamFPath,CameraInfo& vCam)
{
	char strTemp[1024];
	int CamCount;
	::GetPrivateProfileString("CAM_PAR", "camCount", "-1", strTemp, 1024, lpDPCamFPath);
	sscanf(strTemp, "%d", &CamCount);
	if (CamCount < 0) {
		m_error_msg = "无法读取DPGrid工程相机文件!";
		return 0;
	}
	for (int i = 0; i < CamCount; ++i) {
		char app_name[32];
		sprintf(app_name, "CAM_%d", i);
		CameraData CamTemp;
		CamTemp.CameraID = i;
		::GetPrivateProfileString(app_name, "CameraName", "", CamTemp.CameraName, 1024, lpDPCamFPath);
		
		::GetPrivateProfileString(app_name, "FocalLen_mm", "0", strTemp, 1024, lpDPCamFPath);	CamTemp.f = atof(strTemp);
		::GetPrivateProfileString(app_name, "PixelSize_mm", "0", strTemp, 1024, lpDPCamFPath);	CamTemp.pixelSize = atof(strTemp);
		::GetPrivateProfileString(app_name, "CenterX0_mm", "0", strTemp, 1024, lpDPCamFPath);	CamTemp.x0 = atof(strTemp);
		::GetPrivateProfileString(app_name, "CenterY0_mm", "0", strTemp, 1024, lpDPCamFPath);	CamTemp.y0 = atof(strTemp);
		::GetPrivateProfileString(app_name, "FrameWid_pxl", "0", strTemp, 1024, lpDPCamFPath);	CamTemp.width = atoi(strTemp);
		::GetPrivateProfileString(app_name, "FrameHei_pxl", "0", strTemp, 1024, lpDPCamFPath);	CamTemp.height = atoi(strTemp);
		::GetPrivateProfileString(app_name, "CameraType", "0", strTemp, 1024, lpDPCamFPath);	
		::GetPrivateProfileString(app_name, "CameraOritent", "0", strTemp, 1024, lpDPCamFPath);	
		::GetPrivateProfileString(app_name, "DistUnit", "0", strTemp, 1024, lpDPCamFPath);
		::GetPrivateProfileString(app_name, "DistIter", "0", strTemp, 1024, lpDPCamFPath);
		::GetPrivateProfileString(app_name, "DistPar_K1", "0", strTemp, 1024, lpDPCamFPath);	CamTemp.distortionPar[1] = atof(strTemp);
		::GetPrivateProfileString(app_name, "DistPar_K2", "0", strTemp, 1024, lpDPCamFPath);	CamTemp.distortionPar[2] = atof(strTemp);
		::GetPrivateProfileString(app_name, "DistPar_K3", "0", strTemp, 1024, lpDPCamFPath);	CamTemp.distortionPar[3] = atof(strTemp);
		::GetPrivateProfileString(app_name, "DistPar_P1", "0", strTemp, 1024, lpDPCamFPath);	CamTemp.distortionPar[4] = atof(strTemp);
		::GetPrivateProfileString(app_name, "DistPar_P2", "0", strTemp, 1024, lpDPCamFPath);	CamTemp.distortionPar[5] = atof(strTemp);
		::GetPrivateProfileString(app_name, "DistPar_K1r", "0", strTemp, 1024, lpDPCamFPath);
		::GetPrivateProfileString(app_name, "DistPar_K2r", "0", strTemp, 1024, lpDPCamFPath);
		::GetPrivateProfileString(app_name, "DistPar_K3r", "0", strTemp, 1024, lpDPCamFPath);
		::GetPrivateProfileString(app_name, "DistPar_P1r", "0", strTemp, 1024, lpDPCamFPath);
		::GetPrivateProfileString(app_name, "DistPar_P2r", "0", strTemp, 1024, lpDPCamFPath);
		::GetPrivateProfileString(app_name, "FidCount", "0", strTemp, 1024, lpDPCamFPath);

		vCam.push_back(CamTemp);
	}

// 	FILE *fp;
// 	fp = fopen(lpDPCamFPath, "r");
// 	if (!fp)
// 	{
// 		m_error_msg = "无法读取DPGrid工程相机文件!";
// 		return 0;
// 	}
// 	char szBuf[1024];
// 	CString str_temp;
// 	char temp[1024];
// 	int CamCount = 0;
// 	fgets(szBuf, 1024, fp); //省略第一行
// 	fgets(szBuf, 1024, fp);
// 	str_temp = szBuf;
// 	str_temp.TrimRight();  //消除右侧空格
// 	strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 	CamCount = atoi(temp);
// 	int fidcount = 0;
// 	int tempI;
// 	double tempF;
// 	for (int i = 0; i < CamCount; i++)
// 	{
// 		CameraData CamTemp;
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		str_temp = str_temp.Left(str_temp.GetLength() - 1 - str_temp.ReverseFind(']'));
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('_')));
// 		CamTemp.CameraID = atoi(temp);  //相机ID
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		sprintf(CamTemp.CameraName, "%s", temp); //相机名
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		CamTemp.f = atof(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		CamTemp.pixelSize = atof(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		CamTemp.x0 = atof(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		CamTemp.y0 = atof(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		CamTemp.width = atoi(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		CamTemp.height = atoi(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		tempI = atoi(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		tempI = atoi(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		tempI = atoi(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		tempI = atoi(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		CamTemp.distortionPar[1] = atof(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		CamTemp.distortionPar[2] = atof(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		CamTemp.distortionPar[3] = atof(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		CamTemp.distortionPar[4] = atof(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		CamTemp.distortionPar[5] = atof(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		tempF = atof(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		tempF = atof(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		tempF = atof(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		tempF = atof(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		tempF = atof(temp);
// 
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();
// 		strcpy(temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		fidcount = atoi(temp);
// 		/*for (int j = 0; j < fidcount; j++)
// 			fgets(szBuf, 1024, fp);*/
// 
// 		vCam.push_back(CamTemp);
// 	}
// 	fclose(fp);

	return TRUE;
}

int LasPrjIO::ReadDPImgLocFile(char* lpDPImgLocFPath,ImgInfo& vImg)
{
	if (_access(lpDPImgLocFPath, 0) == -1) {
		m_error_msg = "无法读取DPGrid工程影像路径文件!";
		return 0;
	}
	for (auto & img : vImg) {
		char strTemp[512];
		::GetPrivateProfileString("IMGFILE_LOC", img.szImgName, img.szImgPath, strTemp, 512, lpDPImgLocFPath);
		strcpy(img.szImgPath, strTemp);
	}

// 	FILE *fp;
// 	fp = fopen(lpDPImgLocFPath, "r");
// 	if (!fp)
// 	{
// 		m_error_msg = "无法读取DPGrid工程影像路径文件!";
// 		return 0;
// 	}
// 	char szBuf[1024];
// 
// 	fseek(fp, 0L, SEEK_END);
// 	long end = ftell(fp);
// 	fseek(fp, 0L, SEEK_SET);
// 	long start = ftell(fp);
// 	int ImgCount = 0;
// 	CString str_temp;
// 
// 	fgets(szBuf, 1024, fp);
// 	while (end != start)
// 	{
// 		ImgData Imgtemp;
// 		fgets(szBuf, 1024, fp);
// 		str_temp = szBuf;
// 		str_temp.TrimRight();  //消除右侧空格
// 		strcpy(Imgtemp.szImgPath, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		strcpy(Imgtemp.szImgName, str_temp.Left(str_temp.ReverseFind('=')));
// 		
// 		for (int i = 0; i < vImg.size(); i++)
// 		{
// 			if (strcmp(Imgtemp.szImgName, vImg[i].szImgName) == 0)
// 			{
// 				sprintf(vImg[i].szImgPath, "%s", Imgtemp.szImgPath);
// 				break;
// 			}
// 				
// 		}
// 
// 		start = ftell(fp);
// 	}
// 	fclose(fp);

	return 1;
}

int LasPrjIO::ReadDPVpsFile(char* lpDPVpsFPath,ImgInfo& vImg)
{
	if (_access(lpDPVpsFPath, 0) == -1) {
		m_error_msg = "无法读取DPGrid工程Pos文件!";
		return 0;
	}
	for (auto & img : vImg)	{
		char strTemp[1024]; memset(strTemp, 0, 1024);
		::GetPrivateProfileString("IMGFILE_LOC", img.szImgName, "", strTemp, 512, lpDPVpsFPath);
		if (strlen(strTemp) > 0) {
			sscanf(strTemp, "%lf%lf%lf%lf%lf%lf", &img.Xs, &img.Ys, &img.Zs, &img.Phi, &img.Omega, &img.Kappa);
		}
	}

// 	FILE *fpos;
// 	fpos = fopen(lpDPVpsFPath, "r");
// 	if (!fpos)
// 	{
// 		m_error_msg = "无法读取DPGrid工程Pos文件!";
// 		return 0;
// 	}
// 	char szBuf[1024];
// 
// 	fseek(fpos, 0L, SEEK_END);
// 	long end = ftell(fpos);
// 	fseek(fpos, 0L, SEEK_SET);
// 	long start = ftell(fpos);
// 	//int ImgCount = 0;
// 	//CString str_temp;
// 
// 	int nImgNum;
// 	//char szBuf[1024];
// 	char szBuf2[1024];
// 	char char_temp[1024];
// 	CString str_temp;
// 
// 	fgets(szBuf, 1024, fpos);
// 	while (end != start)
// 	{
// 		ImgData img_temp;
// 		fgets(szBuf2, 1024, fpos);
// 		trim(szBuf2, szBuf);
// 		str_temp = szBuf; //temp
// 		str_temp.TrimRight();  //消除右侧空格
// 		strcpy(char_temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		//img_temp.Kappa = atof(char_temp);
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 
// 		str_temp = char_temp;  //temp
// 		str_temp.TrimRight(" ");
// 		strcpy(char_temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		//img_temp.Omega = atof(char_temp);
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 
// 		str_temp = char_temp;  //temp
// 		str_temp.TrimRight(" ");
// 		strcpy(char_temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		//img_temp.Omega = atof(char_temp);
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 
// 		str_temp = char_temp;  //kappa
// 		str_temp.TrimRight(" ");
// 		strcpy(char_temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		img_temp.Kappa = atof(char_temp);
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 
// 		str_temp = char_temp;  //omega
// 		str_temp.TrimRight(" ");
// 		strcpy(char_temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		img_temp.Omega = atof(char_temp);
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 
// 		str_temp = char_temp;  //Phi
// 		str_temp.TrimRight(" ");
// 		strcpy(char_temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		img_temp.Phi = atof(char_temp);
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 
// 		str_temp = char_temp;  //Z
// 		str_temp.TrimRight(" ");
// 		strcpy(char_temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		img_temp.Zs = atof(char_temp);
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 
// 		str_temp = char_temp;  //Y
// 		str_temp.TrimRight(" ");
// 		strcpy(char_temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind(' ')));
// 		img_temp.Ys = atof(char_temp);
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind(' ')));
// 
// 		str_temp = char_temp;  //X
// 		str_temp.TrimRight(" ");
// 		strcpy(char_temp, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 		img_temp.Xs = atof(char_temp);
// 		strcpy(char_temp, str_temp.Left(str_temp.ReverseFind('=')));
// 
// 		trim(char_temp, img_temp.szImgName);    //name
// 
// 		//将外方位元素存入同名影像中
// 		for (int j = 0; j < vImg.size(); j++)
// 			if (strcmp(vImg[j].szImgName, img_temp.szImgName) == 0)
// 			{
// 				//vImg[j].gps_time = img_temp.gps_time;
// 				vImg[j].Xs = img_temp.Xs;
// 				vImg[j].Ys = img_temp.Ys;
// 				vImg[j].Zs = img_temp.Zs;
// 				vImg[j].Phi = img_temp.Phi;
// 				vImg[j].Omega = img_temp.Omega;
// 				vImg[j].Kappa = img_temp.Kappa;
// 				break;
// 			}
// 
// 		start = ftell(fpos);
// 	}
// 	fclose(fpos);

	return 1;
}

int LasPrjIO::ReadDPDpiFile(char* lpDPDpiFPath, ImgData& Img,CameraInfo& vCam)
{
	if (_access(lpDPDpiFPath, 0) == -1) {
		m_error_msg = "无法读取DPGrid工程Pos文件!";
		return 0;
	}
	char szCamName[1024]; memset(szCamName, 0, 1024);
	::GetPrivateProfileString("CMR_PAR", "CameraName", "", szCamName, 512, lpDPDpiFPath);
	for (int i = 0; i < vCam.size(); i++)
	{
		if (strcmp(szCamName, vCam[i].CameraName) == 0)
		{
			Img.CameraID = vCam[i].CameraID;
			break;
		}
	}
	
// 	FILE *fDpi;
// 	fDpi = fopen(lpDPDpiFPath, "r");
// 	if (fDpi == NULL)
// 	{
// 		m_error_msg = "无法读取DPGrid工程影像dpi文件!";
// 		return 0;
// 	}
// 	char szCamName[_MAX_PATH];
// 	char strLine[1024];
// 	CString str_temp;
// 	for (int i = 0; i < 6; i++)
// 		fgets(strLine, 256, fDpi);
// 
// 	fgets(strLine, 1024, fDpi);
// 	str_temp = strLine;
// 	str_temp.TrimRight();  //消除右侧空格
// 	strcpy(szCamName, str_temp.Right(str_temp.GetLength() - 1 - str_temp.ReverseFind('=')));
// 	
// 	for (int i = 0; i < vCam.size(); i++)
// 	{
// 		if (strcmp(szCamName, vCam[i].CameraName) == 0)
// 		{
// 			Img.CameraID = vCam[i].CameraID;
// 			break;
// 		}
// 	}	
// 
// 	fclose(fDpi);

	return 1;
}

int LasPrjIO::ReadImgGpsInfo(const char* szImgFPath, double& Lat, double& Lon, double& Height)
{
	FILE *fp;
	fopen_s(&fp, szImgFPath, "rb");
	if (!fp) {
		printf("Can't open file.\n");
		return -1;
	}
	fseek(fp, 0, SEEK_END);
	unsigned long fsize = ftell(fp);
	rewind(fp);
	unsigned char *buf = new unsigned char[fsize];
	fread(buf, 1, fsize, fp);
	fclose(fp);

	EXIFInfo result;
	int code = result.parseFrom(buf, fsize);
	delete[] buf;
	if (code) {
		return -1;
	}
	Lat = result.GeoLocation.Latitude;
	Lon = result.GeoLocation.Longitude;
	Height = result.GeoLocation.Altitude;

	return 1;
}

int LasPrjIO::ReadImgGpsInfo()
{
	omp_set_num_threads(4);
#pragma omp parallel for
	for (int i = 0; i < m_ImgInfo.size(); i++)
	{
		ReadImgGpsInfo(m_ImgInfo[i].szImgPath, m_ImgInfo[i].Lat, m_ImgInfo[i].Lon, m_ImgInfo[i].Height);
		//m_ImgInfo[i].Zs = m_ImgInfo[i].Height;
	}		

	return 1;
}

double LasPrjIO::DegToRad(double deg)
{
	return (deg / 180.0 * pi);
}

double LasPrjIO::RadToDeg(double rad)
{
	return (rad / pi * 180.0);
}

double LasPrjIO::ArcLengthOfMeridian(double sm_a,double sm_b,double phi)
{
	double alpha, beta, gamma, delta, epsilon, n;
	double result;
	n = (sm_a - sm_b) / (sm_a + sm_b);
	alpha = ((sm_a + sm_b) / 2.0) * (1.0 + (pow(n, 2.0) / 4.0) + (pow(n, 4.0) / 64.0));
	beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0) + (-3.0 * pow(n, 5.0) / 32.0);
	gamma = (15.0 * pow(n, 2.0) / 16.0) + (-15.0 * pow(n, 4.0) / 32.0);
	delta = (-35.0 * pow(n, 3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);
	epsilon = (315.0 * pow(n, 4.0) / 512.0);
	result = alpha * (phi + (beta * sin(2.0 * phi)) + (gamma * sin(4.0 * phi)) + (delta * sin(6.0 * phi)) + (epsilon * sin(8.0 * phi)));

	return result;
}

inline double LasPrjIO::UTMCentralMeridian(int zone)
{
	return DegToRad(-183.0 + (zone * 6.0));
}

double LasPrjIO::FootpointLatitude(double sm_a,double sm_b,double y)
{
	double y_, alpha_, beta_, gamma_, delta_, epsilon_, n;
	double result;

	n = (sm_a - sm_b) / (sm_a + sm_b);
	alpha_ = ((sm_a + sm_b) / 2.0) * (1 + (pow(n, 2.0) / 4) + (pow(n, 4.0) / 64));
	y_ = y / alpha_;
	beta_ = (3.0 * n / 2.0) + (-27.0 * pow(n, 3.0) / 32.0) + (269.0 * pow(n, 5.0) / 512.0);
	gamma_ = (21.0 * pow(n, 2.0) / 16.0) + (-55.0 * pow(n, 4.0) / 32.0);
	delta_ = (151.0 * pow(n, 3.0) / 96.0) + (-417.0 * pow(n, 5.0) / 128.0);
	epsilon_ = (1097.0 * pow(n, 4.0) / 512.0);
	result = y_ + (beta_ * sin(2.0 * y_)) + (gamma_ * sin(4.0 * y_)) + (delta_ * sin(6.0 * y_)) + (epsilon_ * sin(8.0 * y_));

	return result;
}

void LasPrjIO::MapLatLonToXY(double sm_a,double sm_b,double phi, double lambda, double lambda0, ProjCoor &xy)
{
	double N, nu2, ep2, t, t2, l;
	double l3coef, l4coef, l5coef, l6coef, l7coef, l8coef;
	double tmp;

	ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);
	nu2 = ep2 * pow(cos(phi), 2.0);
	N = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nu2));
	t = tan(phi);
	t2 = t * t;
	tmp = (t2 * t2 * t2) - pow(t, 6.0);
	l = lambda - lambda0;
	l3coef = 1.0 - t2 + nu2;
	l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);
	l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;
	l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;
	l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);
	l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);
	xy.x = N * cos(phi) * l + (N / 6.0 * pow(cos(phi), 3.0) * l3coef * pow(l, 3.0))
		+ (N / 120.0 * pow(cos(phi), 5.0) * l5coef * pow(l, 5.0))
		+ (N / 5040.0 * pow(cos(phi), 7.0) * l7coef * pow(l, 7.0));
	xy.y = ArcLengthOfMeridian(sm_a,sm_b,phi)
		+ (t / 2.0 * N * pow(cos(phi), 2.0) * pow(l, 2.0))
		+ (t / 24.0 * N * pow(cos(phi), 4.0) * l4coef * pow(l, 4.0))
		+ (t / 720.0 * N * pow(cos(phi), 6.0) * l6coef * pow(l, 6.0))
		+ (t / 40320.0 * N * pow(cos(phi), 8.0) * l8coef * pow(l, 8.0));
}

void LasPrjIO::MapXYToLatLon(double sm_a, double sm_b, double x, double y, double lambda0, GeoCorr &philambda)
{
	double phif, Nf, Nfpow, nuf2, ep2, tf, tf2, tf4, cf;
	double x1frac, x2frac, x3frac, x4frac, x5frac, x6frac, x7frac, x8frac;
	double x2poly, x3poly, x4poly, x5poly, x6poly, x7poly, x8poly;

	phif = FootpointLatitude(sm_a,  sm_b, y);
	ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);
	cf = cos(phif);
	nuf2 = ep2 * pow(cf, 2.0);
	Nf = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nuf2));
	Nfpow = Nf;
	tf = tan(phif);
	tf2 = tf * tf;
	tf4 = tf2 * tf2;
	x1frac = 1.0 / (Nfpow * cf);
	Nfpow *= Nf;  
	x2frac = tf / (2.0 * Nfpow);
	Nfpow *= Nf;  
	x3frac = 1.0 / (6.0 * Nfpow * cf);
	Nfpow *= Nf; 
	x4frac = tf / (24.0 * Nfpow);
	Nfpow *= Nf;  
	x5frac = 1.0 / (120.0 * Nfpow * cf);
	Nfpow *= Nf;  
	x6frac = tf / (720.0 * Nfpow);
	Nfpow *= Nf;  
	x7frac = 1.0 / (5040.0 * Nfpow * cf);
	Nfpow *= Nf;  
	x8frac = tf / (40320.0 * Nfpow);
	x2poly = -1.0 - nuf2;
	x3poly = -1.0 - 2 * tf2 - nuf2;
	x4poly = 5.0 + 3.0 * tf2 + 6.0 * nuf2 - 6.0 * tf2 * nuf2 - 3.0 * (nuf2 *nuf2) - 9.0 * tf2 * (nuf2 * nuf2);
	x5poly = 5.0 + 28.0 * tf2 + 24.0 * tf4 + 6.0 * nuf2 + 8.0 * tf2 * nuf2;
	x6poly = -61.0 - 90.0 * tf2 - 45.0 * tf4 - 107.0 * nuf2 + 162.0 * tf2 * nuf2;
	x7poly = -61.0 - 662.0 * tf2 - 1320.0 * tf4 - 720.0 * (tf4 * tf2);
	x8poly = 1385.0 + 3633.0 * tf2 + 4095.0 * tf4 + 1575 * (tf4 * tf2);
	philambda.lat = phif + x2frac * x2poly * (x * x) + x4frac * x4poly * pow(x, 4.0) + x6frac * x6poly * pow(x, 6.0) + x8frac * x8poly * pow(x, 8.0);
	philambda.log = lambda0 + x1frac * x + x3frac * x3poly * pow(x, 3.0) + x5frac * x5poly * pow(x, 5.0) + x7frac * x7poly * pow(x, 7.0);
}

void LasPrjIO::LatLonToUTMXY(double lat, double lon, int nType, int ParaDegree, double *x, double *y)
{
	//ParaDegree = 6; //UTM投影一般只有6度分带
	
	double UTMScaleFactor = 0.9996;
	double sm_a = 6378137.0;
	double sm_b = 6356752.31414;
	double sm_EccSquared = 6.69437999013e-03;
	
	if (0 == nType)  //北京54
	{
		sm_a = 6378245.0;
		sm_b == 6356863.0188;
		sm_EccSquared = 0.006693421614543;
	}
	else if (1 == nType) //西安80
	{
		sm_a = 6378137.0;
		sm_b == 6356755.2882;
		sm_EccSquared = 0.00669438499959;
	}
	else if (2 == nType) //wgs84
	{
		sm_a = 6378137.0;
		sm_b = 6356752.31414;
		sm_EccSquared = 6.69437999013e-03;
	}
	else if (3 == nType) //CGCS2000
	{
		sm_a = 6378137.0;
		sm_b = 6356752.31414;
		sm_EccSquared = 0.006694380022898;
	}
	
	//计算投影带中心
	int zone = 0;
	double m = 0;
	
	if (ParaDegree == 6)
	{
		zone = static_cast<int>((lon + 180.0) / 6) + 1;
		lon = DegToRad(lon);
		lat = DegToRad(lat);
		m = DegToRad(-183.0 + (zone * 6.0));
	}

	ProjCoor xy;
	//double m = UTMCentralMeridian(zone);
	MapLatLonToXY(sm_a,sm_b, lat, lon, m, xy);

	xy.x = xy.x * UTMScaleFactor + 500000.0;
	xy.y = xy.y * UTMScaleFactor;
	if (xy.y < 0.0) //南半球
		xy.y += 10000000.0;

	*x = xy.x;
	*y = xy.y;
}

//根据投影中心经线计算UTM投影坐标
void LasPrjIO::LatLonToUTMXY(double lat, double lon, int nType, double dcentralM, double *x, double *y)
{
	double UTMScaleFactor = 0.9996;
	double sm_a = 6378137.0;
	double sm_b = 6356752.31414;
	double sm_EccSquared = 6.69437999013e-03;

	if (0 == nType)  //北京54
	{
		sm_a = 6378245.0;
		sm_b == 6356863.0188;
		sm_EccSquared = 0.006693421614543;
	}
	else if (1 == nType) //西安80
	{
		sm_a = 6378137.0;
		sm_b == 6356755.2882;
		sm_EccSquared = 0.00669438499959;
	}
	else if (2 == nType) //wgs84
	{
		sm_a = 6378137.0;
		sm_b = 6356752.31414;
		sm_EccSquared = 6.69437999013e-03;
	}
	else if (3 == nType) //CGCS2000
	{
		sm_a = 6378137.0;
		sm_b = 6356752.31414;
		sm_EccSquared = 0.006694380022898;
	}

	double m = 0;
	m = DegToRad(dcentralM);
	lon = DegToRad(lon);
	lat = DegToRad(lat);

	ProjCoor xy;
	MapLatLonToXY(sm_a, sm_b, lat, lon, m, xy);

	xy.x = xy.x * UTMScaleFactor + 500000.0;
	xy.y = xy.y * UTMScaleFactor;
	if (xy.y < 0.0)
		xy.y += 10000000.0;

	*x = xy.x;
	*y = xy.y;
}

//根据经纬度计算UTM投影坐标 批量
void LasPrjIO::LatLonToUTMXY(int nType, int ParaDegree)
{
	omp_set_num_threads(4);
#pragma omp parallel for
	for (int i = 0; i < m_ImgInfo.size(); i++)
	{
		LatLonToUTMXY(m_ImgInfo[i].Lat, m_ImgInfo[i].Lon, nType, ParaDegree, &m_ImgInfo[i].Xs, &m_ImgInfo[i].Ys);
	}
}

//根据投影中央经线计算UTM投影坐标 批量
void LasPrjIO::LatLonToUTMXY(int nType, double dcentralM)
{
	omp_set_num_threads(4);
#pragma omp parallel for
	for (int i = 0; i < m_ImgInfo.size(); i++)
	{
		//LatLonToUTMXY(m_ImgInfo[i].Lat, m_ImgInfo[i].Lon, nType, ParaDegree, &m_ImgInfo[i].Xs, &m_ImgInfo[i].Ys);
		LatLonToUTMXY(m_ImgInfo[i].Lat, m_ImgInfo[i].Lon, nType,  dcentralM, &m_ImgInfo[i].Xs, &m_ImgInfo[i].Ys);
	}
}

//经纬度计算高斯克吕格投影坐标
void LasPrjIO::LatLonToGauXY(double latitude, double longitude, int nType,int ParaDegree, double *x, double *y)
{
	double a = 6378137.0;
	double f = 1.0 / 298.257223;
	if (0 == nType)  //北京54
	{
		a = 6378245.0;
		f = 1.0 / 298.3;
	}
	else if (1 == nType) //西安80
	{
		a = 6378137.0;
		f = 1.0 / 298.257222101;
	}
	else if (2 == nType) //wgs84
	{
		a = 6378137.0;
		f = 1.0 / 298.257223;
	}
	else if (3 == nType) //CGCS2000
	{
		a = 6378137.0;
		f = 1 / 298.257222101;
	}
	
	//	longitude = Dms2Rad(longitude);
	//	latitude = Dms2Rad(latitude);
	int nLo = 1, nLa = 1;
	if (longitude < 0)
	{
		// 		longitude = fabs(longitude);
		// 		nLo = -1;
		longitude += 360;
	}
	if (latitude < 0)
	{
		latitude = fabs(latitude);
		nLa = -1;
	}

	//	latitude = 0;
	int ProjNo = 0;
	
	if (6 == ParaDegree)
	{
		ProjNo = (int)(longitude / ParaDegree) + 1;  
	}
	else if (3 == ParaDegree)
	{
		//	longitude = 1.5;
		ProjNo = (int)((longitude - 1.5) / ParaDegree) + 1;
		if ((longitude >= 358.5 && longitude <= 360) || (longitude >= 0 && longitude <= 1.5))
		{
			ProjNo = 120;
		}
	}

	double longitude1, latitude1, longitude0 = 0.0, latitude0, x0, y0, xval, yval;
	double e2, ee, NN, T, C, A, M, iPI;
	iPI = pi / 180.0;
	if (6 == ParaDegree)
	{
		longitude0 = ProjNo * ParaDegree - ParaDegree / 2;
	}
	else if (3 == ParaDegree)
	{
		longitude0 = ProjNo * 3;
		if ((longitude >= 0 && longitude <= 1.5))
		{
			longitude0 = 0;
		}
	}

	longitude0 = longitude0 * iPI;
	latitude0 = 0;
	longitude1 = longitude * iPI; 
	latitude1 = latitude * iPI; 
	e2 = 2 * f - f*f;
	ee = e2*(1.0 - e2);
	NN = a / sqrt(1.0 - e2*sin(latitude1)*sin(latitude1));
	T = tan(latitude1)*tan(latitude1);
	C = ee*cos(latitude1)*cos(latitude1);
	A = (longitude1 - longitude0)*cos(latitude1);

	M = a*((1 - e2 / 4 - 3 * e2*e2 / 64 - 5 * e2*e2*e2 / 256)*latitude1 - (3 * e2 / 8 + 3 * e2*e2 / 32 + 45 * e2*e2
		*e2 / 1024)*sin(2 * latitude1)
		+ (15 * e2*e2 / 256 + 45 * e2*e2*e2 / 1024)*sin(4 * latitude1) - (35 * e2*e2*e2 / 3072)*sin(6 * latitude1));
	yval = NN*(A + (1 - T + C)*A*A*A / 6 + (5 - 18 * T + T*T + 72 * C - 58 * ee)*A*A*A*A*A / 120);
	xval = M + NN*tan(latitude1)*(A*A / 2 + (5 - T + 9 * C + 4 * C*C)*A*A*A*A / 24
		+ (61 - 58 * T + T*T + 600 * C - 330 * ee)*A*A*A*A*A*A / 720);
	//y0 = 1000000L*(ProjNo)+500000L; //带号+平移
	y0 = 500000L; //只平移
	x0 = 0;
	xval = xval + x0;
	yval = yval + y0;

	*x = yval*nLo;
	*y = xval*nLa;

	return;
}

//根据中央经线计算高斯投影坐标
void LasPrjIO::LatLonToGauXY(double latitude, double longitude, int nType, double dcentralM, double *x, double *y)
{
	double a = 6378137.0;
	double f = 1.0 / 298.257223;
	if (0 == nType)  //北京54
	{
		a = 6378245.0;
		f = 1.0 / 298.3;
	}
	else if (1 == nType) //西安80
	{
		a = 6378137.0;
		f = 1.0 / 298.257222101;
	}
	else if (2 == nType) //wgs84
	{
		a = 6378137.0;
		f = 1.0 / 298.257223;
	}
	else if (3 == nType) //CGCS2000
	{
		a = 6378137.0;
		f = 1 / 298.257222101;
	}

	int nLo = 1, nLa = 1;
	if (longitude < 0)
	{
		longitude += 360;
	}
	if (latitude < 0)
	{
		latitude = fabs(latitude);
		nLa = -1;
	}

	double longitude1, latitude1, longitude0 = 0.0, latitude0, x0, y0, xval, yval;
	double e2, ee, NN, T, C, A, M, iPI;
	iPI = pi / 180.0;
	
	longitude0 = dcentralM;
	if (longitude0 < 0)
		longitude0 += 360;

	longitude0 = longitude0 * iPI;
	latitude0 = 0;
	longitude1 = longitude * iPI;
	latitude1 = latitude * iPI; 
	e2 = 2 * f - f*f;
	ee = e2*(1.0 - e2);
	NN = a / sqrt(1.0 - e2*sin(latitude1)*sin(latitude1));
	T = tan(latitude1)*tan(latitude1);
	C = ee*cos(latitude1)*cos(latitude1);
	A = (longitude1 - longitude0)*cos(latitude1);

	M = a*((1 - e2 / 4 - 3 * e2*e2 / 64 - 5 * e2*e2*e2 / 256)*latitude1 - (3 * e2 / 8 + 3 * e2*e2 / 32 + 45 * e2*e2
		*e2 / 1024)*sin(2 * latitude1)
		+ (15 * e2*e2 / 256 + 45 * e2*e2*e2 / 1024)*sin(4 * latitude1) - (35 * e2*e2*e2 / 3072)*sin(6 * latitude1));
	yval = NN*(A + (1 - T + C)*A*A*A / 6 + (5 - 18 * T + T*T + 72 * C - 58 * ee)*A*A*A*A*A / 120);
	xval = M + NN*tan(latitude1)*(A*A / 2 + (5 - T + 9 * C + 4 * C*C)*A*A*A*A / 24
		+ (61 - 58 * T + T*T + 600 * C - 330 * ee)*A*A*A*A*A*A / 720);
	//y0 = 1000000L*(ProjNo)+500000L;
	y0 = 500000L;
	x0 = 0;
	xval = xval + x0;
	yval = yval + y0;

	*x = yval*nLo; //x y交换，即使以横轴为x 纵轴为y
	*y = xval*nLa;

	return;
}

void LasPrjIO::LatLonToGauXY(int nType, int ParaDegree)
{
	omp_set_num_threads(4);
#pragma omp parallel for
	for (int i = 0; i < m_ImgInfo.size(); i++)
	{
		LatLonToGauXY(m_ImgInfo[i].Lat, m_ImgInfo[i].Lon, nType, ParaDegree, &m_ImgInfo[i].Xs, &m_ImgInfo[i].Ys);
	}
}

void LasPrjIO::LatLonToGauXY(int nType, double dcentralM)
{
	omp_set_num_threads(4);
#pragma omp parallel for
	for (int i = 0; i < m_ImgInfo.size(); i++)
	{
		//LatLonToGauXY(m_ImgInfo[i].Lat, m_ImgInfo[i].Lon, nType, ParaDegree, &m_ImgInfo[i].Xs, &m_ImgInfo[i].Ys);
		LatLonToGauXY(m_ImgInfo[i].Lat, m_ImgInfo[i].Lon, nType, dcentralM, &m_ImgInfo[i].Xs, &m_ImgInfo[i].Ys);
	}
}

//根据投影带号计算中央经线
int LasPrjIO::CalCentralLon(int nProjNo, int ParaDegree, int nProjType, double& dCentralLon)
{
	if (nProjType == 1) //高斯克吕格投影
	{
		if (ParaDegree == 6)
		{
			dCentralLon = 6 * nProjNo - 3;
		}
		else if (ParaDegree == 3)
		{
			dCentralLon = 3 * nProjNo;
		}
		if (dCentralLon > 180)
			dCentralLon -= 360;
	}
	else if (nProjType == 2) //UTM投影
	{
		if (ParaDegree != 6)
		{
			dCentralLon = -9999;
			return -1;
		}
		dCentralLon = -183.0 + (nProjNo * 6.0);
	}
	return 1;
}

//根据经度计算所在投影带的中央经线
int LasPrjIO::CalCentralLon(double Lon, int ParaDegree, int nProjType, double& dCentralLon)
{
	if (nProjType == 1) //高斯克吕格投影
	{
		if (Lon < 0)
			Lon += 360;
		if (6 == ParaDegree)
		{		
			int nProjNo = (int)(Lon / ParaDegree) + 1;   //带号
			dCentralLon = 6 * nProjNo - 3;
		}
		else if (3 == ParaDegree)
		{
			int nProjNo = (int)((Lon - 1.5) / ParaDegree) + 1;
			if ((Lon >= 358.5 && Lon <= 360) || (Lon >= 0 && Lon <= 1.5))
			{
				nProjNo = 120;
			}
			dCentralLon = 3 * nProjNo;
		}
	}
	else if (nProjType == 2) //UTM投影
	{
		if (ParaDegree != 6)
		{
			dCentralLon = -9999;
			return -1;
		}

		if (Lon > 180)
			Lon -= 360;
		int nProjNo = static_cast<int>((Lon + 180.0) / 6) + 1;
		dCentralLon = -183.0 + (nProjNo * 6.0);
	}
	return 1;
}

//UTM投影坐标转大地坐标
void LasPrjIO::UTMXYToLatLon(double x, double y, int nType, double dcentralM/*int zone*/, double& lat, double& lon)
{
	double UTMScaleFactor = 0.9996;
	double sm_a = 6378137.0;
	double sm_b = 6356752.31414;
	double sm_EccSquared = 6.69437999013e-03;

	if (0 == nType)  //北京54
	{
		sm_a = 6378245.0;
		sm_b == 6356863.0188;
		sm_EccSquared = 0.006693421614543;
	}
	else if (1 == nType) //西安80
	{
		sm_a = 6378137.0;
		sm_b == 6356755.2882;
		sm_EccSquared = 0.00669438499959;
	}
	else if (2 == nType) //wgs84
	{
		sm_a = 6378137.0;
		sm_b = 6356752.31414;
		sm_EccSquared = 6.69437999013e-03;
	}
	else if (3 == nType) //CGCS2000
	{
		sm_a = 6378137.0;
		sm_b = 6356752.31414;
		sm_EccSquared = 0.006694380022898;
	}
	
	double cmeridian;
	x -= 500000.0;
	x /= UTMScaleFactor;

	y /= UTMScaleFactor;

	cmeridian = DegToRad(dcentralM);
	GeoCorr philambda;
	MapXYToLatLon(sm_a,sm_b,x, y, cmeridian, philambda);
	lat = philambda.lat/pi*180;
	lon = philambda.log/pi*180;
}

//高斯克吕格投影坐标转大地坐标
void LasPrjIO::GauXYToLatLon(double _x, double _y, int nType, double dcentralM/*int zone*/, double& lat, double& lon)
{
	double a = 6378137.0;
	double f = 1.0 / 298.257223;
	if (0 == nType)  //北京54
	{
		a = 6378245.0;
		f = 1.0 / 298.3;
	}
	else if (1 == nType) //西安80
	{
		a = 6378137.0;
		f = 1.0 / 298.257222101;
	}
	else if (2 == nType) //wgs84
	{
		a = 6378137.0;
		f = 1.0 / 298.257223;
	}
	else if (3 == nType) //CGCS2000
	{
		a = 6378137.0;
		f = 1 / 298.257222101;
	}
	
	int nX = 1, nY = 1;
	double x, y;
	x = _y;
	y = _x; 
	if (x < 0)
	{
		x = fabs(x);
		nX = -1;
	}
	if (y < 0)
	{
		y = fabs(y);
		nY = -1;
	}
	int ProjNo;
	double longitude1, latitude1, longitude0 = 0.0, x0, y0, xval, yval;
	double e1, e2, ee, NN, T, C, M, D, R, u, fai, iPI;
	iPI = pi / 180.0;

	longitude0 = dcentralM;
	longitude0 = longitude0 * iPI; //中央经线
	//y0 = ProjNo * 1000000L + 500000L;
	//x0 = 0;
	//xval = x - x0; yval = y - y0; //带内大地坐标
	xval = x; yval = y- 500000; //带内大地坐标
	e2 = 2 * f - f*f;
	e1 = (1.0 - sqrt(1 - e2)) / (1.0 + sqrt(1 - e2));
	ee = e2 / (1 - e2);
	M = xval;
	u = M / (a*(1 - e2 / 4 - 3 * e2*e2 / 64 - 5 * e2*e2*e2 / 256));
	fai = u + (3 * e1 / 2 - 27 * e1*e1*e1 / 32)*sin(2 * u) + (21 * e1*e1 / 16 - 55 * e1*e1*e1*e1 / 32)*sin(
		4 * u)
		+ (151 * e1*e1*e1 / 96)*sin(6 * u) + (1097 * e1*e1*e1*e1 / 512)*sin(8 * u);
	if (fai >= 1.57)
	{
		lon = 0;
		lat = 90;
		return;
	}
	else if (fai <= -1.57)
	{
		lon = 0;
		lat = -90;
		return;
	}
	C = ee*cos(fai)*cos(fai);
	T = tan(fai)*tan(fai);
	NN = a / sqrt(1.0 - e2*sin(fai)*sin(fai));
	R = a*(1 - e2) / sqrt((1 - e2*sin(fai)*sin(fai))*(1 - e2*sin(fai)*sin(fai))*(1 - e2*sin
		(fai)*sin(fai)));
	D = yval / NN;
	longitude1 = longitude0 + (D - (1 + 2 * T + C)*D*D*D / 6 + (5 - 2 * C + 28 * T - 3 * C*C + 8 * ee + 24 * T*T)*D
		*D*D*D*D / 120) / cos(fai);
	latitude1 = fai - (NN*tan(fai) / R)*(D*D / 2 - (5 + 3 * T + 10 * C - 4 * C*C - 9 * ee)*D*D*D*D / 24
		+ (61 + 90 * T + 298 * C + 45 * T*T - 256 * ee - 3 * C*C)*D*D*D*D*D*D / 720);
	lon = longitude1 / iPI * nY;
	lat = latitude1 / iPI * nX;
}

void LasPrjIO::UTMXYToLatLon(int nType, double dcentralM)
{
	omp_set_num_threads(4);
#pragma omp parallel for
	for (int i = 0; i < m_ImgInfo.size(); i++)
		UTMXYToLatLon(m_ImgInfo[i].Xs, m_ImgInfo[i].Ys, nType, dcentralM, m_ImgInfo[i].Lat, m_ImgInfo[i].Lon);
}

void LasPrjIO::GauXYToLatLon(int nType, double dcentralM)
{
	omp_set_num_threads(4);
#pragma omp parallel for
	for (int i = 0; i < m_ImgInfo.size(); i++)
		GauXYToLatLon(m_ImgInfo[i].Xs, m_ImgInfo[i].Ys, nType, dcentralM, m_ImgInfo[i].Lat, m_ImgInfo[i].Lon);
}