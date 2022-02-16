#pragma once

#include "core/interface/IObject.h"
#include "core/FastDelegate.h"
#include "core/iplmatrix.h"
#include "core/iplmacro.h"


namespace ipl
{
	//属性类型
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
		IPL_V_Attribute = 12 //Node真实的属性
	};

	//
	// 定义属性监听器, 用于属性变化通知，设置其它属性的取值范围和默认取值等
	//
	//	bool OnPropChanged( const char *propName, IProperty *propControlList )
	//		propName --- 变化的属性名	
	//		propControlList	--- 用于编辑的属性控件列表
	//	返回：true == 接受, false == 拒绝
	//

	interface IPropControlList;
	interface IPropControl;

	using namespace fastdelegate;
	typedef FastDelegate2<IPropControlList *, IPropControl *, bool> PropertyListener;

#define IPL_PROPERTY_LISTNER _T("PropertyListner")

	//
	//
	// IProperty 实际上是一个 属性列表（attrs)接口 Purpose: <Key,Value>属性实现
	//
	//	
	interface IProperty : public IObject
	{
	public:
		// 全部复制
		virtual void copy(const ref_ptr<IProperty> pProperty) = 0;

		// 只更新自己有的
		virtual void update(const ref_ptr<IProperty> pProperty) = 0;

	public:		//单接口
		virtual bool getAttr(const iplChar * name, bool & value) const = 0;

		virtual bool getAttr(const iplChar * name, ipl_int16 & value) const = 0;
		virtual bool getAttr(const iplChar * name, ipl_int32 &value) const = 0;
		virtual bool getAttr(const iplChar * name, ipl_float64 &value) const = 0;

		// 传回拷贝
		virtual bool getAttr(const iplChar * name, iplString &value) const = 0;
		virtual bool getAttr(const iplChar * name, iplVectorBase &value) const = 0;
		virtual bool getAttr(const iplChar * name, iplMatrixBase &value) const = 0;

		// 传回内部指针
		virtual bool getAttr(const iplChar * name, const ipl_byte* &pValue, ipl_int32 &nLength) const = 0;
		virtual bool getAttr(const iplChar * name, ref_ptr<IProperty> &value) const = 0;

	public:		//复接口
		virtual bool getAttr(const iplChar * name, iplArray<ipl_int16> &values) const = 0;
		virtual bool getAttr(const iplChar * name, iplArray<ipl_int32> &values) const = 0;
		virtual bool getAttr(const iplChar * name, iplArray<ipl_float64> &values) const = 0;
		virtual bool getAttr(const iplChar * name, iplArray<iplString>	&values) const = 0;

		// 传回内部指针
		virtual bool getAttr(const iplChar * name, iplArray<const ipl_byte *> &values, iplArray<ipl_int32> &vLength) const = 0;
		virtual bool getAttr(const iplChar * name, iplArray<ref_ptr<IProperty> > &values) const = 0;

	public:		//变接口
		virtual bool setAttr(const iplChar * name, bool value) = 0;
		virtual bool setAttr(const iplChar * name, ipl_int16 value, ipl_uint32 index = 0) = 0;
		virtual bool setAttr(const iplChar * name, ipl_int32 value, ipl_uint32 index = 0) = 0;
		virtual bool setAttr(const iplChar * name, ipl_float64 value, ipl_uint32 index = 0) = 0;
		virtual bool setAttr(const iplChar * name, const iplChar * value, ipl_uint32 index = 0) = 0;
		virtual bool setAttr(const iplChar * name, const ipl_byte* pValue, ipl_int32 nLength, ipl_uint32 index = 0) = 0;
		virtual bool setAttr(const iplChar * name, const iplVectorBase &value) = 0;
		virtual bool setAttr(const iplChar * name, const iplMatrixBase &value) = 0;
		virtual bool setAttr(const iplChar * name, ref_ptr<IProperty> value, ipl_uint32 index = 0) = 0;

	public:		//增接口
		virtual void addAttr(const iplChar * name, bool value, bool bUnique = false) = 0;
		virtual void addAttr(const iplChar * name, ipl_int16 value, bool bUnique = false) = 0;
		virtual void addAttr(const iplChar * name, ipl_int32 value, bool bUnique = false) = 0;
		virtual void addAttr(const iplChar * name, ipl_float64 value, bool bUnique = false) = 0;

		// 内部拷贝
		virtual void addAttr(const iplChar * name, const iplChar * value, bool bUnique = false) = 0;
		//添加真实的属性，如：<task name="ImageGeoQualityCheck">
		virtual void addAttribute(const iplChar * name, const iplChar * value, bool bUnique = false) = 0;
		virtual void addAttr(const iplChar * name, const ipl_byte *pValue, ipl_int32 nLength, bool bUnique = false) = 0;
		virtual void addAttr(const iplChar * name, const iplVectorBase &value, bool bUnique = false) = 0;
		virtual void addAttr(const iplChar * name, const iplMatrixBase &value, bool bUnique = false) = 0;

		// 内部引用
		virtual void addAttr(const iplChar * name, ref_ptr<IProperty> value, bool bUnique = false) = 0;

	public:
		// 扩展，对象指针
		virtual void addAttr(const iplChar * name, ref_ptr<IObject> value, bool bUnique = false) = 0;
		virtual bool setAttr(const iplChar * name, ref_ptr<IObject> value) = 0;
		virtual bool getAttr(const iplChar * name, ref_ptr<IObject> &value)  const = 0;

		// 属性监听器
		virtual void addAttr(const iplChar * name, ref_ptr<PropertyListener> value, bool bUnique = false) = 0;
		virtual bool setAttr(const iplChar * name, ref_ptr<PropertyListener> value) = 0;
		virtual bool getAttr(const iplChar * name, ref_ptr<PropertyListener> &value)  const = 0;

	public:
		virtual bool remove(const iplChar * name) = 0;
		virtual bool removeAll() = 0;

		virtual ref_ptr<IProperty> createChild(const iplChar * name, bool bUnique = false) = 0;

		// 查找属性节点, nodePath形式如，//ClssHrchy/AllClss
		virtual ref_ptr<IProperty> findNode(const iplChar *nodePath, ref_ptr<IProperty> &pParent) = 0;
		////判断是否存在节点name
		virtual bool findAttrName(const iplChar * name) = 0;

		// 返回属性个数
		virtual size_t size() const = 0;

		//属性名称和类型，用于序列化
		//ind是属性的索引号，范围为0 --- (size()-1);
		virtual void getAttributeInfo(ipl_uint32 ind, iplString &name, IPLVariantType &type,
			ipl_int32 &numOfValues, bool bSort = true) const = 0;

		// 取属性类型
		virtual IPLVariantType getAttrType(const iplChar *attrName) const = 0;

		//接口
		IPL_INTERFACE_DEF(IObject, _T("property"))
	};

#define IPL_PROPERTY		  _T("ipl.property")
#define IPL_PROPERTY_DEFAULT  _T("ipl.property.default")


	//属性序列化器
	interface IPropertySerialize : public IObject
	{
	public:
		// 从xml字节流导入属性，info需要事先分配内存
		virtual bool import(const char *bytes, long length, IProperty *info) = 0;

		// 导出属性到xml字节流
		// 默认输入参数char* bytes在堆上分配了内存，要由用户清理，运行时要保持一致
		virtual bool outport(char *&bytes, long *length, const IProperty *info) = 0;
		//不包含const char *pHead = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";这个头

		virtual bool outport_noUTF_8Head(char *&bytes, long *length, const IProperty *info) = 0;

		// 释放函数
		virtual bool free(char *&bytes) = 0;

		//接口
		IPL_INTERFACE_DEF(IObject, _T("propertySerialize"))
	};

#define IPL_PROPERTYSERIALIZE		   _T("ipl.propertySerialize")
#define IPL_PROPERTYSERIALIZE_DEFAULT  _T("ipl.propertySerialize.xml")

}
