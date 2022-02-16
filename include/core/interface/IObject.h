#pragma once


#include "core/ipldef.h"
#include "core/iplstd.h"

namespace ipl
{
	interface IProperty;

	interface IObject
	{
		// 用于析构派生类
	public: //用boost智能指针，必须在析构函数中析构
		virtual ~IObject() {};

	public:
		
		// 类名, 短描述
		virtual iplString	getClassName()	const { return _T("Unknown"); };

		// 层次化对象ID, 区分对象的唯一标记, 实际ID由接口ID和对象ID连接而成
		// 命名规则与eclipse相似，由对象ID的层次名字可以构建插件树
		virtual iplString	getClassID()	const { return _T("ipl"); }

		// 得到detailed description
		virtual iplString	getClassDesc() const { return _T("empty"); }

		// 得到实现的接口类名
		virtual iplArray<iplString> getInterfaceNames() const = 0;

		// 根据接口名取得类指针
		virtual void *queryInterface(const iplChar *className)  const = 0;

		///////////////////// 属性操作 ////////////////////////////////////////////
		// 得到属性, 可以用于获取内部数据, 序列化等
		virtual const IProperty *getProperty() const { return NULL; };

		// 用于从xml恢复对象状态，xml->property->object
		virtual bool initFromProperty(IProperty *property) { return true; };

		IProperty *getProperty()
		{
			return const_cast<IProperty *> ((static_cast<const IObject&>(*this)).getProperty());
		};


		// 	//设置属性
		// 	virtual void propertyChangeNotify() {} ;
		// 	
		// 	virtual bool checkAttr(const iplChar * name,	ors_int16 value, ors_int32 index  ) { return true;	};
		// 	virtual bool checkAttr(const iplChar * name,	ors_int32 value, ors_int32 index  ) { return true;	};
		// 	virtual bool checkAttr(const iplChar * name,	ors_float64 value, ors_int32 index  )  { return true;	};
		// 	virtual bool checkAttr(const iplChar * name,	const iplChar * value, ors_int32 index  )  { return true;	} ;
		// 	
		// 	// 得到属性约束
		// 	virtual iplString getAttrContrainedDescription( const iplChar * name ) const { return _T("empty");}

	};



	//
	// 对象的描述信息
	//
	interface IObjectDesc
	{
	public:
		virtual const iplChar *getName() = 0;
		virtual const iplChar *getID() = 0;
		virtual const iplChar *getDesc() = 0;
		virtual const iplChar *getPluginName() = 0;
		virtual const iplArray<iplString> &getInterfaceNames() const = 0;
	};


//#include "core/interface/IProperty.h"

}

