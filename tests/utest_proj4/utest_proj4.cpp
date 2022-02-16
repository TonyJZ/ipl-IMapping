// utest_proj4.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"

#include <stdio.h>
#include <stdlib.h>

#include "proj/proj_api.h"


int main()
{
	// ����һ������54�ĺ���ī����ͶӰ����ϵ
	// +proj=lcc    ͶӰ���ͣ�����ī����ͶӰ
	// +ellps=krass ������
	// +lat_1=25n +lat_2=47n    ά�ȷ�Χ(��׼γ��)
	// +lon_0=117e  ���뾭��Ϊ����117��
	// +x_0=20500000    X��(��)����ƫ����
	// +y_0=0           Y��(��)����ƫ����
	// +units=m         ��λ
	// +k=1.0           ����

	const char* beijing1954 = "+proj=lcc +ellps=krass +lat_1=25n +lat_2=47n +lon_0=117e +x_0=20500000 +y_0=0 +units=m +k=1.0";
	//�������ת����WGS84��׼ 
	//"+towgs84=22,-118,30.5,0,0,0,0"

	projPJ pj;  // ����ϵ����ָ��
				// ��ʼ������ϵ����
	if (!(pj = pj_init_plus(beijing1954))) {
		exit(-1);   // ��ʼ��ʧ�ܣ��˳�����
	}

	// ��ת��������(ͶӰ����)
	// ע������ϵ�����е�+x_0=20500000������ֵӦ��Ҳ�Ǵ��д��ŵ�
	projUV parr[4] = {
		{ 20634500.0,4660000.0 },
		{ 20635000.0,4661000.0 },
		{ 20635500.0,4659000.0 },
		{ 20634000.0,4662000.0 }
	};

	printf("DEG_TO_RAD = %f  (1��=%f����)\n", DEG_TO_RAD, DEG_TO_RAD);

	// ���ת��
	for (int i = 0; i < 4; i++)
	{
		printf("\n--------------ת����%d��---------------\n", i + 1);
		projUV p;

		p = pj_inv(parr[i], pj); // ͶӰ��任(ͶӰ����ת��γ������)
		printf("����54ͶӰ  ����:%10lf,%10lf\n", parr[i].u, parr[i].v);
		printf("����54��γ������:%10lf,%10lf\n", p.u / DEG_TO_RAD, p.v / DEG_TO_RAD);    // �����ʱ�򣬽�����ת��Ϊ��

		p = pj_fwd(p, pj);       // ͶӰ���任(��γ������תͶӰ����)
		printf("����54ͶӰ  ����:%10lf,%10lf\n", p.u, p.v);
	}

	// �ͷ�ͶӰ�����ڴ�
	pj_free(pj);
	return 0;
}

