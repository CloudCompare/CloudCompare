#pragma once
#include"PrjInfo.h"
//#include <afx.h>
#include<stdio.h>
#include<string>
#include<sstream>

#ifndef USE_LASPRJIO_AS_LIB
#ifdef _LasPrjIO_LIB_export
#define _LasPrjIO_LIB	 __declspec(dllexport)
#else
#define _LasPrjIO_LIB  __declspec(dllimport)

#ifdef _DEBUG
#ifdef _WIN64
#pragma comment(lib,"LasPrjIO_D64.lib")
#pragma message("Automatically linking with LasPrjIO_D64.lib")
#else
#pragma comment(lib,"LasPrjIO_D.lib")
#pragma message("Automatically linking with LasPrjIO_D.lib")
#endif
#else

#ifdef _WIN64
#pragma comment(lib,"LasPrjIO64.lib")
#pragma message("Automatically linking with LasPrjIO64.lib")
#else
#pragma comment(lib,"LasPrjIO.lib")
#pragma message("Automatically linking with LasPrjIO.lib")
#endif

#endif
#endif
#else
#define _LasPrjIO_LIB
#endif



class _LasPrjIO_LIB LasPrjIO
{
public:
	LasPrjIO();
	~LasPrjIO();

	bool GeneratePrjFiles();  //���ɹ����ļ�
	int LoadPrjFiles(char* lpXMLPath);  
	int LoadPrjFiles2(char* lpXMLPath); //��ȡ����������las��img
	int ReadCamFile(char* lpstrPath, CameraInfo& sCamInfo);
	int ReadPosFile(char* lpstrPath, ImgInfo& sImgInfo);	
	int WriteCamFile(char* lpstrPath, CameraInfo sCamInfo);
	bool WritePrjFile(PrjHdr sPrjHdr);

	int ReadDPrjInfo(char* lpDppFPath);
	int ReadDppFile(char* lpDppFPath,ImgInfo& vImg);
	int ReadDPCamFile(char* lpDPCamFPath,CameraInfo& vCam);
	int ReadDPImgLocFile(char* lpDPImgLocFPath,ImgInfo& vImg);
	int ReadDPVpsFile(char* lpDPVpsFPath,ImgInfo& vImg);
	int ReadDPDpiFile(char* lpDPDpiFPath,ImgData& Img,CameraInfo& vCam);

	int ReadImgGpsInfo();//��ȡӰ��GPS��Ϣ ����
	int ReadImgGpsInfo(const char* szImgFPath, double& Lat, double& Lon, double& Height);//��ȡӰ��GPS��Ϣ

	/*ͶӰ������Ժ���Ϊx�� ����Ϊy��,������δ������ţ�
	����������ƽ��500km����x�����ϼ���500km��
	ͶӰ�����У���˹������ͶӰ��3�Ⱥ�6�����ִַ���ʽ��UTMͶӰֻ��6�ȷִ���
	����Ϊ��������Ϊ������γΪ������γΪ������λΪ��
	���ֺ���������
	nType:  2:WGS84 3: CGCS2000 
	ParaDegree: 3:���ȷִ� 6:���ȷִ���UTMͶӰֻ��6�ȷִ���
	nProjType:ͶӰ���� 1����˹������ͶӰ 2��UTMͶӰ
	nProjNo:ͶӰ����
	dcentralM: ͶӰ�����뾭��	*/
	
	int CalCentralLon(int nProjNo, int ParaDegree,int nProjType, double& dCentralLon);//����ͶӰ���ż������뾭��	
	int CalCentralLon(double Lon, int ParaDegree, int nProjType, double& dCentralLon);//���ݾ��ȼ�������ͶӰ�������뾭��

	//���ݵ㾭�ȼ���ͶӰ����
	void LatLonToUTMXY(double lat, double lon, int nType, int ParaDegree, double *x, double *y); //���ݾ�γ�ȼ���UTMͶӰ���꣬UTMͶӰֻ��6�ȷִ�����ParaDegreeӦΪ6
	void LatLonToGauXY(double lat, double lon, int nType, int ParaDegree, double *x, double *y);//���ݾ�γ�ȼ����˹ͶӰ���꣬UTMͶӰֻ��6�ȷִ�����ParaDegreeӦΪ6
	void LatLonToUTMXY(int nType, int ParaDegree); //���ݾ�γ�ȼ���UTMͶӰ���� ����
	void LatLonToGauXY(int nType, int ParaDegree);//���ݾ�γ�ȼ����˹ͶӰ���� ����

	//�������뾭�߼���ͶӰ����
	void LatLonToUTMXY(double lat, double lon, int nType, double dcentralM, double *x, double *y); //�������뾭�߼���UTMͶӰ����
	void LatLonToGauXY(double lat, double lon, int nType, double dcentralM, double *x, double *y);//�������뾭�߼����˹ͶӰ����
	void LatLonToUTMXY(int nType, double dcentralM); //����ͶӰ���뾭�߼���UTMͶӰ���� ����
	void LatLonToGauXY(int nType, double dcentralM);//����ͶӰ���뾭�߼����˹ͶӰ���� ����

	//����ͶӰ���귴��������
	void UTMXYToLatLon(double x, double y, int nType, double dcentralM/*int zone*/, double& lat, double& lon);//UTMͶӰ����ת�������
	void GauXYToLatLon(double x, double y, int nType, double dcentralM/*int zone*/, double& lat, double& lon);//��˹������ͶӰ����ת�������
	void UTMXYToLatLon(int nType, double dcentralM);
	void GauXYToLatLon(int nType, double dcentralM);

	std::string getErrorMsg() { return m_error_msg; }
protected:
	
	int WriteLasList(char* lpstrPath, LasInfo sLasInfo);
	int WriteImgList(char* lpstrPath, ImgInfo ImgInfo);
	int WritePosFile(char* lpstrPath, ImgInfo sImgInfo);

	int ReadPrjFile(char* lpstrPath, PrjHdr& sPrjHdr);
	int ReadLasList(char* lpstrPath, LasInfo& sLasInfo,PrjHdr& sPrjHdr);	
	int ReadLasList(char* sLasLstPath, LasInfo& sLasInfo);
	int ReadImgList(char* lpstrPath, ImgInfo& sImgInfo,PrjHdr& sPrjHdr);
	int ReadImgList(char* sImgLstPath, ImgInfo& sImgInfo);
	void trim(char *strIn, char *strOut);

	static double RadToDeg(double rad);	
	static double FootpointLatitude(double sm_a,double sm_b,double y);
	static void MapXYToLatLon(double sm_a,double sm_b,double x, double y, double lambda0, GeoCorr &philambda);

	static double DegToRad(double deg);
	static double ArcLengthOfMeridian(double sm_a,double sm_b,double phi);
	static void MapLatLonToXY(double sm_a,double sm_b,double phi, double lambda, double lambda0, ProjCoor &xy);
	static double UTMCentralMeridian(int zone);
	
public:
	PrjHdr m_PrjHdr;
	CameraInfo m_CameraInfo;
	LasInfo m_LasInfo;
	ImgInfo m_ImgInfo;

private:
	static const int m_MaxCamCount = 10; //��������
	static const int m_MaxImgCount = 10000;  //���Ӱ����
	static const int m_MaxLasCount = 1000; //������������
	std::string m_error_msg;
};

