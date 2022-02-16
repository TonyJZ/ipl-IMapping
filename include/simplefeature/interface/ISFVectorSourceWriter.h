#pragma once

#include "simplefeature/interface/ISFVectorSource.h"

namespace ipl
{
	interface ISFVectorSourceWriter : public IVectorSourceWriter
	{
	public:
		//����ģʽ��ʱδʵ��
		//virtual bool Open(const char *pszName, int bUpdate) = 0;
		//virtual int DeleteDataSource(const char *pszName) = 0;

		//����ģʽ
		virtual bool Create(const char *pszName, const char *pszFormat/*, ref_ptr<ISpatialReference> pSRS*/) = 0;

		virtual void Close() = 0;

		virtual bool IsOpen() const = 0;

		//virtual ISFVectorLayer *GetLayerByName(const char *) = 0;
		virtual bool SetVectorSource(ref_ptr<ISFVectorSource> pSource) = 0;

/************************************************************************/
/* ���½�ͼ�������˳�δ����ʱDBF�ļ�Ϊ��,�ٴδ򿪲�����ʱ�ͻᷢ���쳣
������ΪOGRVC161D�⺯�� OGRErr SHPWriteOGRFeature( SHPHandle hSHP, DBFHandle hDBF,OGRFeatureDefn *poFeatureDefn,
OGRFeature *poFeature )δ�ж�hDBF�Ƿ�Ϊ�յ������
������ͨ�������½�һ���ֶ��Ƿ�ɹ����ж�DBF�ļ��Ƿ���ȷ
*/
// 		virtual int TestCapability(const char *) = 0;
// 		virtual int			TestDBF() = 0;

		IPL_INTERFACE_DEF(IVectorSourceWriter, "SF");
	};
}

