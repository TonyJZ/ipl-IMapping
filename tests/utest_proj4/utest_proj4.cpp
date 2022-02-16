// utest_proj4.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"

#include <stdio.h>
#include <stdlib.h>

#include "proj/proj_api.h"


int main()
{
	// 定义一个北京54的横轴墨卡托投影坐标系
	// +proj=lcc    投影类型：横轴墨卡托投影
	// +ellps=krass 椭球体
	// +lat_1=25n +lat_2=47n    维度范围(标准纬线)
	// +lon_0=117e  中央经度为东经117度
	// +x_0=20500000    X轴(东)方向偏移量
	// +y_0=0           Y轴(北)方向偏移量
	// +units=m         单位
	// +k=1.0           比率

	const char* beijing1954 = "+proj=lcc +ellps=krass +lat_1=25n +lat_2=47n +lon_0=117e +x_0=20500000 +y_0=0 +units=m +k=1.0";
	//如果你想转换到WGS84基准 
	//"+towgs84=22,-118,30.5,0,0,0,0"

	projPJ pj;  // 坐标系对象指针
				// 初始化坐标系对象
	if (!(pj = pj_init_plus(beijing1954))) {
		exit(-1);   // 初始化失败，退出程序
	}

	// 待转换的坐标(投影坐标)
	// 注意坐标系定义中的+x_0=20500000，坐标值应该也是带有带号的
	projUV parr[4] = {
		{ 20634500.0,4660000.0 },
		{ 20635000.0,4661000.0 },
		{ 20635500.0,4659000.0 },
		{ 20634000.0,4662000.0 }
	};

	printf("DEG_TO_RAD = %f  (1度=%f弧度)\n", DEG_TO_RAD, DEG_TO_RAD);

	// 逐点转换
	for (int i = 0; i < 4; i++)
	{
		printf("\n--------------转换第%d点---------------\n", i + 1);
		projUV p;

		p = pj_inv(parr[i], pj); // 投影逆变换(投影坐标转经纬度坐标)
		printf("北京54投影  坐标:%10lf,%10lf\n", parr[i].u, parr[i].v);
		printf("北京54经纬度坐标:%10lf,%10lf\n", p.u / DEG_TO_RAD, p.v / DEG_TO_RAD);    // 输出的时候，将弧度转换为度

		p = pj_fwd(p, pj);       // 投影正变换(经纬度坐标转投影坐标)
		printf("北京54投影  坐标:%10lf,%10lf\n", p.u, p.v);
	}

	// 释放投影对象内存
	pj_free(pj);
	return 0;
}

