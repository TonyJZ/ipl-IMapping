#pragma once

#include "geometry/interface/IGeoObject.h"
#include "simplefeature/SFBaseDef.h"



namespace ipl
{
	/************************************************************************/
	/*                               ISFPoint                               */
	/************************************************************************/

	/**
	* Point class.
	*/

	interface ISFPoint : public IGeoObject
	{
	public:
		double x, y, z;

	public:
		virtual int  getDimension() const { return 0; }

		IPL_INTERFACE_DEF(IGeoObject, "SFPoint");
	};

	/************************************************************************/
	/*                            ISFLineString                             */
	/************************************************************************/

	/**
	* Concrete representation of a multi-vertex line.
	*/

	interface ISFLineString : public IGeoObject
	{
	public:

		// 曲线长度
		virtual double getLength() const = 0;

		// 起点
		virtual void startPoint(iplPOINT3D *) = 0;

		// 终点
		virtual void endPoint(iplPOINT3D *) = 0;

		// 是否闭合
		virtual int  isClosed() = 0;

		// ILineString methods
		virtual int   getNumPoints() const = 0;
		virtual void  setNumPoints(int numPoints) = 0;

		virtual const iplPOINT3D *getPtsBuf() const = 0;
		virtual void getPoint(int index, iplPOINT3D *points) = 0;

		virtual void setPoint(int index, const iplPOINT3D point) = 0;
		virtual void setPoint(int index, double x, double y, double z) = 0;
		virtual void setPoints(iplPOINT3D *points, int num) = 0;

		virtual void addPoint(const iplPOINT3D point) = 0;

		virtual void removePoint(int index) = 0;

		IPL_INTERFACE_DEF(IGeoObject, "SFLineString");
	};



	/************************************************************************/
	/*                            ISFLineRing                             */
	/*                                                                      */
	/*      This is an alias for ISFLineString for now.                     */
	/************************************************************************/

	/**
	* Concrete representation of a closed ring.
	*/

	interface ISFLineRing : public ISFLineString
	{
	public:
		virtual int isClockwise() const = 0;
		virtual void closeRing() = 0;
		virtual double getArea() const = 0;

		IPL_INTERFACE_DEF(IGeoObject, "SFLineRing");
	};


	/************************************************************************/
	/*                              ISFPolygon                              */
	/************************************************************************/

	/**
	* Concrete class representing polygons.
	*/

	interface  ISFPolygon : public IGeoObject
	{
	public:

		virtual double getArea() const = 0;

		virtual void  addRing(const ref_ptr<ISFLineRing> pRing) = 0;
		virtual void  addRingDirectly(ref_ptr<ISFLineRing> pRing) = 0;

		virtual  ref_ptr<ISFLineRing> getExteriorRing() = 0;
		virtual const ref_ptr<ISFLineRing> getExteriorRing() const = 0;

		virtual int  getNumInteriorRings() const = 0;

		virtual	ref_ptr<ISFLineRing> getInteriorRing(int i) = 0;
		virtual	const ref_ptr<ISFLineRing> getInteriorRing(int i) const = 0;

		virtual void  closeRings() = 0;

		IPL_INTERFACE_DEF(IGeoObject, "SFPolygon");
	};

	/************************************************************************/
	/*                        ISFGeometryCollection                         */
	/************************************************************************/

	/**
	* A collection of 1 or more geometry objects.
	*/

	interface ISFGeometryCollection : public IGeoObject
	{
	public:
		virtual int	getNumGeometries() const = 0;
		virtual ref_ptr<IGeoObject> getGeometryRef(int index) = 0;

		// 加入一个拷贝
		virtual SFERR addGeometry(const ref_ptr<IGeoObject> geom) = 0;
		virtual SFERR addGeometryDirectly(ref_ptr<IGeoObject> geom) = 0;

		virtual SFERR removeGeometry(int index, int bDelete) = 0;
	};

	/************************************************************************/
	/*                           ISFMultiPolygon                            */
	/************************************************************************/

	/**
	* A collection of non-overlapping orsPolygons.
	*/

	interface ISFMultiPolygon : public ISFGeometryCollection
	{
	public:
		virtual double  getArea() const = 0;

		IPL_INTERFACE_DEF(IGeoObject, "SFMultiPolygon");
	};

	/************************************************************************/
	/*                            ISFMultiPoint                             */
	/************************************************************************/

	/**
	* A collection of ISFPoints.
	*/

	interface ISFMultiPoint : public ISFGeometryCollection
	{
	public:
		IPL_INTERFACE_DEF(IGeoObject, "SFMultiPoint");
	};

	/************************************************************************/
	/*                          ISFMultiLineString                          */
	/************************************************************************/

	/**
	* A collection of ISFLineStrings.
	*/

	interface ISFMultiLineString : public ISFGeometryCollection
	{
	public:
		IPL_INTERFACE_DEF(IGeoObject, "SFMultiLineString");
	};

}
