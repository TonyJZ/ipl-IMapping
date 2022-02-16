#pragma once

#include "core/interface/IObject.h"
#include "core/FastDelegate.h"
#include "core/iplmatrix.h"
#include "core/iplmacro.h"


namespace ipl
{
	//��������
	enum IPLVariantType {
		IPL_V_UNKOWN = 0,
		IPL_V_BOOL = 1,
		IPL_V_I2 = 2,
		IPL_V_I4 = 3,
		IPL_V_R8 = 4,
		IPL_V_STR = 5,		//
		IPL_V_BLOB = 6,		//
		IPL_V_CHILD = 7,	// IProperty
		IPL_V_VECTOR = 8,
		IPL_V_MATRIX = 9,
		IPL_V_OBJECT = 10,	// IObject
		IPL_V_LISTNER = 11,	// prop Listener
		IPL_V_Attribute = 12 //Node��ʵ������
	};

	//
	// �������Լ�����, �������Ա仯֪ͨ�������������Ե�ȡֵ��Χ��Ĭ��ȡֵ��
	//
	//	bool OnPropChanged( const char *propName, IProperty *propControlList )
	//		propName --- �仯��������	
	//		propControlList	--- ���ڱ༭�����Կؼ��б�
	//	���أ�true == ����, false == �ܾ�
	//

	interface IPropControlList;
	interface IPropControl;

	using namespace fastdelegate;
	typedef FastDelegate2<IPropControlList *, IPropControl *, bool> PropertyListener;

#define IPL_PROPERTY_LISTNER _T("PropertyListner")

	//
	//
	// IProperty ʵ������һ�� �����б�attrs)�ӿ� Purpose: <Key,Value>����ʵ��
	//
	//	
	interface IProperty : public IObject
	{
	public:
		// ȫ������
		virtual void copy(const ref_ptr<IProperty> pProperty) = 0;

		// ֻ�����Լ��е�
		virtual void update(const ref_ptr<IProperty> pProperty) = 0;

	public:		//���ӿ�
		virtual bool getAttr(const iplChar * name, bool & value) const = 0;

		virtual bool getAttr(const iplChar * name, ipl_int16 & value) const = 0;
		virtual bool getAttr(const iplChar * name, ipl_int32 &value) const = 0;
		virtual bool getAttr(const iplChar * name, ipl_float64 &value) const = 0;

		// ���ؿ���
		virtual bool getAttr(const iplChar * name, iplString &value) const = 0;
		virtual bool getAttr(const iplChar * name, iplVectorBase &value) const = 0;
		virtual bool getAttr(const iplChar * name, iplMatrixBase &value) const = 0;

		// �����ڲ�ָ��
		virtual bool getAttr(const iplChar * name, const ipl_byte* &pValue, ipl_int32 &nLength) const = 0;
		virtual bool getAttr(const iplChar * name, ref_ptr<IProperty> &value) const = 0;

	public:		//���ӿ�
		virtual bool getAttr(const iplChar * name, iplArray<ipl_int16> &values) const = 0;
		virtual bool getAttr(const iplChar * name, iplArray<ipl_int32> &values) const = 0;
		virtual bool getAttr(const iplChar * name, iplArray<ipl_float64> &values) const = 0;
		virtual bool getAttr(const iplChar * name, iplArray<iplString>	&values) const = 0;

		// �����ڲ�ָ��
		virtual bool getAttr(const iplChar * name, iplArray<const ipl_byte *> &values, iplArray<ipl_int32> &vLength) const = 0;
		virtual bool getAttr(const iplChar * name, iplArray<ref_ptr<IProperty> > &values) const = 0;

	public:		//��ӿ�
		virtual bool setAttr(const iplChar * name, bool value) = 0;
		virtual bool setAttr(const iplChar * name, ipl_int16 value, ipl_uint32 index = 0) = 0;
		virtual bool setAttr(const iplChar * name, ipl_int32 value, ipl_uint32 index = 0) = 0;
		virtual bool setAttr(const iplChar * name, ipl_float64 value, ipl_uint32 index = 0) = 0;
		virtual bool setAttr(const iplChar * name, const iplChar * value, ipl_uint32 index = 0) = 0;
		virtual bool setAttr(const iplChar * name, const ipl_byte* pValue, ipl_int32 nLength, ipl_uint32 index = 0) = 0;
		virtual bool setAttr(const iplChar * name, const iplVectorBase &value) = 0;
		virtual bool setAttr(const iplChar * name, const iplMatrixBase &value) = 0;
		virtual bool setAttr(const iplChar * name, ref_ptr<IProperty> value, ipl_uint32 index = 0) = 0;

	public:		//���ӿ�
		virtual void addAttr(const iplChar * name, bool value, bool bUnique = false) = 0;
		virtual void addAttr(const iplChar * name, ipl_int16 value, bool bUnique = false) = 0;
		virtual void addAttr(const iplChar * name, ipl_int32 value, bool bUnique = false) = 0;
		virtual void addAttr(const iplChar * name, ipl_float64 value, bool bUnique = false) = 0;

		// �ڲ�����
		virtual void addAttr(const iplChar * name, const iplChar * value, bool bUnique = false) = 0;
		//�����ʵ�����ԣ��磺<task name="ImageGeoQualityCheck">
		virtual void addAttribute(const iplChar * name, const iplChar * value, bool bUnique = false) = 0;
		virtual void addAttr(const iplChar * name, const ipl_byte *pValue, ipl_int32 nLength, bool bUnique = false) = 0;
		virtual void addAttr(const iplChar * name, const iplVectorBase &value, bool bUnique = false) = 0;
		virtual void addAttr(const iplChar * name, const iplMatrixBase &value, bool bUnique = false) = 0;

		// �ڲ�����
		virtual void addAttr(const iplChar * name, ref_ptr<IProperty> value, bool bUnique = false) = 0;

	public:
		// ��չ������ָ��
		virtual void addAttr(const iplChar * name, ref_ptr<IObject> value, bool bUnique = false) = 0;
		virtual bool setAttr(const iplChar * name, ref_ptr<IObject> value) = 0;
		virtual bool getAttr(const iplChar * name, ref_ptr<IObject> &value)  const = 0;

		// ���Լ�����
		virtual void addAttr(const iplChar * name, ref_ptr<PropertyListener> value, bool bUnique = false) = 0;
		virtual bool setAttr(const iplChar * name, ref_ptr<PropertyListener> value) = 0;
		virtual bool getAttr(const iplChar * name, ref_ptr<PropertyListener> &value)  const = 0;

	public:
		virtual bool remove(const iplChar * name) = 0;
		virtual bool removeAll() = 0;

		virtual ref_ptr<IProperty> createChild(const iplChar * name, bool bUnique = false) = 0;

		// �������Խڵ�, nodePath��ʽ�磬//ClssHrchy/AllClss
		virtual ref_ptr<IProperty> findNode(const iplChar *nodePath, ref_ptr<IProperty> &pParent) = 0;
		////�ж��Ƿ���ڽڵ�name
		virtual bool findAttrName(const iplChar * name) = 0;

		// �������Ը���
		virtual size_t size() const = 0;

		//�������ƺ����ͣ��������л�
		//ind�����Ե������ţ���ΧΪ0 --- (size()-1);
		virtual void getAttributeInfo(ipl_uint32 ind, iplString &name, IPLVariantType &type,
			ipl_int32 &numOfValues, bool bSort = true) const = 0;

		// ȡ��������
		virtual IPLVariantType getAttrType(const iplChar *attrName) const = 0;

		//�ӿ�
		IPL_INTERFACE_DEF(IObject, _T("property"))
	};

#define IPL_PROPERTY		  _T("ipl.property")
#define IPL_PROPERTY_DEFAULT  _T("ipl.property.default")


	//�������л���
	interface IPropertySerialize : public IObject
	{
	public:
		// ��xml�ֽ����������ԣ�info��Ҫ���ȷ����ڴ�
		virtual bool import(const char *bytes, long length, IProperty *info) = 0;

		// �������Ե�xml�ֽ���
		// Ĭ���������char* bytes�ڶ��Ϸ������ڴ棬Ҫ���û���������ʱҪ����һ��
		virtual bool outport(char *&bytes, long *length, const IProperty *info) = 0;
		//������const char *pHead = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";���ͷ

		virtual bool outport_noUTF_8Head(char *&bytes, long *length, const IProperty *info) = 0;

		// �ͷź���
		virtual bool free(char *&bytes) = 0;

		//�ӿ�
		IPL_INTERFACE_DEF(IObject, _T("propertySerialize"))
	};

#define IPL_PROPERTYSERIALIZE		   _T("ipl.propertySerialize")
#define IPL_PROPERTYSERIALIZE_DEFAULT  _T("ipl.propertySerialize.xml")

}
