#pragma once

#include "core/interface/IPlatform.h"
#include "spatialreference/interface/IGeoid.h"


//
// XGeoid is to support conversions between ellipsoid heights and geoid heights.
//

namespace ipl
{
//	extern IPlatform	*g_orsPlatform;
//	extern ILogService	*g_orsProcess;

	class XGeoid : public IGeoid
	{
	public:
		XGeoid();
		virtual ~XGeoid();

		virtual	bool Initialize(const std::string &wktGeoid);

		virtual	bool GetDeltaHeight(double latitude, double longitude, double *dh);

		virtual	bool Ellipsoid2Geoid(double latitude, double longitude,
			double ellipsoidHeight, double *geoidHeight);

		virtual	bool Geoid2Ellipsoid(double latitude, double longitude,
			double geoidHeight, double *ellipsoidHeight);

	private:
		std::string GetPath();
		bool LoadData_EGM96();
		bool LoadData_Leica(const std::string &fileName);

	private:
		// �Զ�Ϊ��λ
		double m_lat0;	// γ�����
		double m_lat1;	// γ���յ�
		double m_long0;	// �������
		double m_long1;	// �����յ�
		double m_dLat;	// γ�ȼ��
		double m_dLong;	// ���ȼ��

		int m_nRows;	// ����
		int m_nCols;	// ����

		float *m_pData;

		int	m_errCount;

	public:
		IPL_OBJECT_IMP1(XGeoid, IGeoid, "default", "Geoid")
	};
}

