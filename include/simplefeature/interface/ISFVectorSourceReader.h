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
		/* ���½�ͼ�������˳�δ����ʱDBF�ļ�Ϊ��,�ٴδ򿪲�����ʱ�ͻᷢ���쳣
		������ΪOGRVC161D�⺯�� OGRErr SHPWriteOGRFeature( SHPHandle hSHP, DBFHandle hDBF,OGRFeatureDefn *poFeatureDefn,
		OGRFeature *poFeature )δ�ж�hDBF�Ƿ�Ϊ�յ������
		������ͨ�������½�һ���ֶ��Ƿ�ɹ����ж�DBF�ļ��Ƿ���ȷ
		*/
// 		virtual int TestCapability(const char *) = 0;
// 		virtual int			TestDBF() = 0;
		
		IPL_INTERFACE_DEF(IVectorSourceReader, "SF");
	};
}
