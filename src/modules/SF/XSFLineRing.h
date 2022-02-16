#pragma once

#include "core/ipldef.h"
#include "core/iplstd.h"
#include "simplefeature/SFBaseDef.h"
#include "simplefeature/interface/ISFGeometry.h"
//#include "simplefeature/SFLineString.h"

namespace ipl
{
	class XSFLineRing : public ISFLineRing
	{
	private:
		iplArray <iplPOINT3D> m_points;
		iplArray <iplPOINT3D>::iterator iter;
		bool m_bIsRing;

	public:
		XSFLineRing() 
		{ 
			m_bIsRing = true;
			m_coordDimension = 1;
		};
		virtual ~XSFLineRing();

		void SetAsRing(bool bIsRing = true) { m_bIsRing = bIsRing; };

		virtual int getDimension() const { return 1; };

		virtual void getEnvelope(iplEnvelope * psEnvelope) const;
		virtual IPL_wkbGeometryType getGeometryType() const;
		virtual const char *getGeometryName() const;

		// 曲线长度
		virtual double getLength() const;

		// 起点
		virtual void startPoint(iplPOINT3D *);

		// 终点
		virtual void endPoint(iplPOINT3D *);

		// 是否闭合
		virtual int  isClosed();

		// ILineString methods
		virtual int   getNumPoints() const;
		virtual void  setNumPoints(int numPoints);

		virtual const iplPOINT3D *getPtsBuf() const;
		virtual void getPoint(int index, iplPOINT3D *points);

		virtual void setPoint(int index, const iplPOINT3D point);
		virtual void setPoint(int index, double x, double y, double z);
		virtual void setPoints(iplPOINT3D *points, int num);

		virtual void addPoint(const iplPOINT3D point);

		virtual void removePoint(int index);

		virtual int isClockwise() const;
		virtual void closeRing();
		virtual double getArea() const;

		virtual IGeoObject *Clone() const;

	public:
		IPL_OBJECT_IMP1(XSFLineRing, ISFLineRing, "default", "SF LineRing")
	};

}

