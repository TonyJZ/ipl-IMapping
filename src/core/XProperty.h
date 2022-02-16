#pragma once

#include <map>
#include <string>
#include <vector>

#include "core/interface/IPlatform.h"
#include "core/interface/IProperty.h"
//#include "iplCommon.h"

namespace ipl
{

	struct iplBLOB
	{
		void *pData;
	};


	class iplVariant
	{
	public:
		IPLVariantType type;

	private:
		union {
			bool			vBool;
			ipl_int16		vInt16;
			ipl_int32		vInt32;
			ipl_float64		vFloat64;
			iplBLOB			vBlob;
			ref_ptr<IProperty>			vProperty;
			ref_ptr<IObject>			vObject;
			ref_ptr<PropertyListener>	vPropListener;
		};

		// 	long b_value_length ;
		// 	iplString				s_value ;
		// 	ref_ptr<IProperty>	child_value;

	public:
		iplVariant();
		iplVariant(iplVariant & var);
		~iplVariant();

		void set(bool v) { vBool = v; }
		void set(ipl_int16 v) { vInt16 = v; }
		void set(ipl_int32 v) { vInt32 = v; }
		void set(ipl_float64 v) { vFloat64 = v; }
		void set(const iplChar * str);
		void set(ref_ptr<IProperty> pProperty);

		void set(ref_ptr<IObject> pObject);
		void set(ref_ptr<PropertyListener> pObject) { vPropListener = pObject; };

		void set(const ipl_byte *data, ipl_int32 length);

		void set(const iplVectorBase &value);
		void set(const iplMatrixBase &value);

		const ipl_byte *getBlob(ipl_int32 &length);
		const iplChar * getString();

		void getVector(iplVectorBase &vector);
		void getMatrix(iplMatrixBase &matrix);

		bool getBool() { return vBool; }
		ipl_int16 getInt16() { return vInt16; }
		ipl_int32 getInt32() { return vInt32; }
		ipl_float64 getFloat64() { return vFloat64; }
		ref_ptr<IObject> getObject() { return vObject; };
		ref_ptr<IProperty> getProperty() { return vProperty; };
		ref_ptr<PropertyListener> getPropListener() { return vPropListener; };
	};


	struct PropertyRecord {
	private:
		std::vector<ref_ptr<iplVariant> > variants;
	public:
		~PropertyRecord();

		size_t size() { return variants.size(); }
		ref_ptr<iplVariant> GetVariant(int i) 
		{ 
			ref_ptr<iplVariant> ptr0;
			ptr0.reset();

			if (i<0 || i>variants.size() - 1)
				return ptr0;

			return variants[i]; 
		};
		void AddVariant(ref_ptr<iplVariant> var) { variants.push_back(var); };
	};


	//typedef iplVariant  PropertyRecord;

	typedef std::map<iplString, ref_ptr<PropertyRecord>> PropertyRecordMap;

	class XProperty : public IProperty/*, public orsObjectBase*/
	{
	public:
		XProperty();
		virtual ~XProperty();

		virtual void copy(const ref_ptr<IProperty> pProperty);

		// 只更新自己有的
		virtual void update(const ref_ptr<IProperty> pProperty);

		///////////////////////  单接口 //////////////////////
		virtual bool getAttr(const iplChar * name, bool &value) const;

		virtual bool getAttr(const iplChar * name, ipl_int16 & value) const;
		virtual bool getAttr(const iplChar * name, ipl_int32 &value) const;
		virtual bool getAttr(const iplChar * name, ipl_float64 &value) const;

		// 传回拷贝
		virtual bool getAttr(const iplChar * name, iplString &value) const;
		virtual bool getAttr(const iplChar * name, iplVectorBase &value) const;
		virtual bool getAttr(const iplChar * name, iplMatrixBase &value) const;

		// 传回内部指针
		virtual bool getAttr(const iplChar * name, const ipl_byte* &pValue, ipl_int32 &nLength) const;
		virtual bool getAttr(const iplChar * name, ref_ptr<IProperty> &value) const;

		/////////////////////// 复接口 ///////////////////////
		virtual bool getAttr(const iplChar * name, iplArray<ipl_int16> &values) const;
		virtual bool getAttr(const iplChar * name, iplArray<ipl_int32> &values) const;
		virtual bool getAttr(const iplChar * name, iplArray<ipl_float64> &values) const;
		virtual bool getAttr(const iplChar * name, iplArray<iplString> &values) const;

		// 传回内部指针
		virtual bool getAttr(const iplChar * name, iplArray<const ipl_byte *> &values, iplArray<ipl_int32> &vLength) const;
		virtual bool getAttr(const iplChar * name, iplArray<ref_ptr<IProperty> > &values) const;

		/////////////////////// 变接口 ///////////////////////
		virtual bool setAttr(const iplChar * name, bool value);

		virtual bool setAttr(const iplChar * name, ipl_int16 value, ipl_uint32 index);
		virtual bool setAttr(const iplChar * name, ipl_int32 value, ipl_uint32 index);
		virtual bool setAttr(const iplChar * name, ipl_float64 value, ipl_uint32 index);

		// 内部拷贝
		virtual bool setAttr(const iplChar * name, const iplChar * value, ipl_uint32 index);
		virtual bool setAttr(const iplChar * name, const iplVectorBase &value);
		virtual bool setAttr(const iplChar * name, const iplMatrixBase &value);
		virtual bool setAttr(const iplChar * name, const ipl_byte* pValue, ipl_int32 nLength, ipl_uint32 index);

		// 内部引用
		virtual bool setAttr(const iplChar * name, ref_ptr<IProperty> value, ipl_uint32 index = 0);

		/////////////////////// 增接口 ///////////////////////
		virtual void addAttr(const iplChar * name, bool value, bool bUnique = false);

		virtual void addAttr(const iplChar * name, ipl_int16 value, bool bUnique = false);
		virtual void addAttr(const iplChar * name, ipl_int32 value, bool bUnique = false);
		virtual void addAttr(const iplChar * name, ipl_float64 value, bool bUnique = false);

		// 内部拷贝
		virtual void addAttr(const iplChar * name, const iplChar * value, bool bUnique = false);
		//添加真实的属性，如：<task name="ImageGeoQualityCheck">
		virtual void addAttribute(const iplChar * name, const iplChar * value, bool bUnique = false);
		virtual void addAttr(const iplChar * name, const iplVectorBase &value, bool bUnique = false);
		virtual void addAttr(const iplChar * name, const iplMatrixBase &value, bool bUnique = false);
		virtual void addAttr(const iplChar * name, const ipl_byte *pValue, ipl_int32 nLength, bool bUnique = false);

		// 内部引用
		virtual void addAttr(const iplChar * name, ref_ptr<IProperty> value, bool bUnique = false);

		// 扩展，对象指针
		virtual void addAttr(const iplChar * name, ref_ptr<IObject> value, bool bUnique = false);
		virtual bool setAttr(const iplChar * name, ref_ptr<IObject> value);
		virtual bool getAttr(const iplChar * name, ref_ptr<IObject> &value)  const;

		// 属性监听器
		virtual void addAttr(const iplChar * name, ref_ptr<PropertyListener> value, bool bUnique = false);
		virtual bool setAttr(const iplChar * name, ref_ptr<PropertyListener> value);
		virtual bool getAttr(const iplChar * name, ref_ptr<PropertyListener> &value)  const;

		//////////////////////////////////////////////////////////////////////////
		// 查找属性节点
		virtual ref_ptr<IProperty> findNode(const iplChar *nodePath, ref_ptr<IProperty> &pParent);

		virtual bool findAttrName(const iplChar * name);//判断是否存在节点name


		virtual bool remove(const iplChar * name);
		virtual bool removeAll();

		virtual ref_ptr<IProperty> createChild(const iplChar * name, bool bUnique = false);

		//轮询接口
		//属性个数
		virtual size_t size() const
		{
			// return m_records.size();
			return m_attrNames.size();
		}

		//属性名称和类型
		//ind是属性的索引号，范围为0 --- (size()-1);
		virtual void getAttributeInfo(ipl_uint32 ind, iplString &name, IPLVariantType &type,
			ipl_int32 &valuenum, bool bSort = true)  const;

		virtual IPLVariantType getAttrType(const iplChar *attrName) const;

	public:
		// 不排序的属性名
		std::vector <iplString>	m_attrNames;
		PropertyRecordMap m_records;

		//接口宏定义
		IPL_OBJECT_IMP1(XProperty, IProperty, _T("default"), _T("property"))
	};

}
