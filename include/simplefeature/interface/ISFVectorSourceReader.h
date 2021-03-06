#pragma once

#include "simplefeature/interface/ISFVectorSource.h"

namespace ipl
{
	interface ISFVectorSourceReader : public IVectorSourceReader
	{
	public:
		virtual bool Open(const char *pszName, int bUpdate) = 0;
// 		virtual bool Create(const char *pszName, char *pszFormat, ISpatialReference *poSpatialRef) = 0;
// 
// 		virtual int DeleteDataSource(const char *pszName) = 0;

		virtual void Close() = 0;

		virtual bool IsOpen() const = 0;

		virtual ref_ptr<ISFVectorSource> GetVectorSource() = 0;

		/************************************************************************/
		/* 当新建图层意外退出未保存时DBF文件为空,再次打开并保存时就会发生异常
		这是因为OGRVC161D库函数 OGRErr SHPWriteOGRFeature( SHPHandle hSHP, DBFHandle hDBF,OGRFeatureDefn *poFeatureDefn,
		OGRFeature *poFeature )未判断hDBF是否为空的情况。
		本函数通过尝试新建一个字段是否成功来判断DBF文件是否正确
		*/
// 		virtual int TestCapability(const char *) = 0;
// 		virtual int			TestDBF() = 0;
		
		IPL_INTERFACE_DEF(IVectorSourceReader, "SF");
	};
}
