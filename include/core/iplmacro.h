#pragma once

#include <string>
#include "core/ipldef.h"
#include "core/iplstd.h"

namespace ipl
{
	//
	// 接口名由"父接口名" + "." + "本接口名" 构成
	//
	// 对象都是继承自IObject, 如果不重复定义纯虚函数，则会有编译的模糊性问题
	//
#define IPL_INTERFACE_DEF(parentInteface, interfaceID) public:\
	virtual iplString	getClassID() const { return iplString(parentInteface::getClassID() + _T(".") + interfaceID);}\
	virtual void* queryInterface(const iplChar *className) const = 0;\
	virtual iplArray<iplString> getInterfaceNames() const = 0;
	//virtual iplString	getClassName(){ return iplString(interfaceName);}

	//2018.1.15 扩展点，支持跨级扩展，谨慎处理多接口扩展
	//宏定义，为了实现IObject的接口，开发者需要自己实现类的头文件中加入
	//IPL_OBJECT_IMP0 : 自己的类直接从IObject继承而来，或
	//IPL_OBJECT_IMP1 : 自己的类从一个主接口继承而来，或
	//IPL_OBJECT_IMP2 : 自己的类不仅实现了一个主接口，还实现了1个副接口，或
	//IPL_OBJECT_IMP3 : 自己的类不仅实现了一个主接口，还实现了2个副接口
	//一般而言，IPL_OBJECT_IMP1被使用，其他三个宏一般不使用

	struct IPL_INTERFACE_INTMAP_ENTRY
	{
		const iplChar*	name;
		unsigned long dw;
		void* flag;
	};

#define IPL_VTABLE_PACKING	0x10000000
#define IPL_ITL_PACKING 8
#define IPL_VTABLE_PTR(base, derived)	((ipl_ptr2int)(static_cast<base*>((derived*)IPL_ITL_PACKING))-IPL_ITL_PACKING)
#define IPL_VTABLE_PTR2(base,base2, derived)	((ipl_ptr2int)((base*)(base2*)(derived*)IPL_ITL_PACKING)-IPL_ITL_PACKING)

#define IPL_BEGIN_VTABLE_MAP(derived) public: \
	typedef derived _Derived;\
	static IPL_INTERFACE_INTMAP_ENTRY* _GetEntries() { \
		static const IPL_INTERFACE_INTMAP_ENTRY _entries[] = {

#define IPL_INTERFACE_ENTRY(base) { _T(#base), IPL_VTABLE_PTR(base,_Derived),(void*)1},
#define IPL_INTERFACE_ENTRY2(base, base2) { _T(#base), IPL_VTABLE_PTR2(base,base2,_Derived),(void*)1},

#define IPL_END_VTABLE_MAP {0,0,0}}; return (IPL_INTERFACE_INTMAP_ENTRY*)_entries; } \
	virtual void* queryInterface(const iplChar * className) const \
	{ \
		int i = 0; \
		while (_GetEntries()[i].flag != NULL) {\
			if ( _tcsicmp(_GetEntries()[i].name, className ) == 0) {\
				return (void*)((ipl_ptr2int)this + _GetEntries()[i].dw); \
				} i++;\
		} \
		return NULL;\
	}\
	virtual iplArray<iplString> getInterfaceNames() const \
	{\
		\
		int i = 0; \
		iplArray<iplString> names; \
		while (_GetEntries()[i].name != NULL) { \
			names.push_back(_GetEntries()[i++].name); \
		} \
		return names; \
	}

	// 对IObject中两个虚函数的宏实现
	// 对象名由"主接口名" + "." + "具体对象名" 构成
	//
#define IPL_OBJECT_DEF_NORMAL(primaryInteface, objID, objName) public: \
	virtual iplString	getClassID() const {return iplString(primaryInteface::getClassID() + _T(".") + objID);}\
	virtual iplString	getClassName() const {return iplString(objName);}

#define IPL_OBJECT_DEF_NORMAL_NO_Primary(objID, objName) public: \
	virtual iplString	getClassID() const {return iplString(IObject::getClassID() + _T(".") + objID);}\
	virtual iplString	getClassName() const {return iplString(objName);}

#define IPL_OBJECT_IMP0(selftype, objID ,objName) \
	IPL_OBJECT_DEF_NORMAL_NO_Primary( objID, objName) \
	IPL_BEGIN_VTABLE_MAP(selftype) \
		IPL_INTERFACE_ENTRY(IObject) \
	IPL_END_VTABLE_MAP

#define IPL_OBJECT_IMP1(selftype,primaryInteface, objID, objName) \
	IPL_OBJECT_DEF_NORMAL(primaryInteface, objID,objName) \
	IPL_BEGIN_VTABLE_MAP(selftype) \
		IPL_INTERFACE_ENTRY(IObject) \
		IPL_INTERFACE_ENTRY(primaryInteface) \
	IPL_END_VTABLE_MAP

#define IPL_OBJECT_IMP2(selftype,primaryInteface,interface2, objID, objName) \
	IPL_OBJECT_DEF_NORMAL(primaryInteface, objID,objName) \
	IPL_BEGIN_VTABLE_MAP(selftype) \
		IPL_INTERFACE_ENTRY(IObject) \
		IPL_INTERFACE_ENTRY(primaryInteface) \
		IPL_INTERFACE_ENTRY(interface2) \
	IPL_END_VTABLE_MAP

#define IPL_OBJECT_IMP3(selftype,primaryInteface,interface2,interface3, objID, objName) \
	IPL_OBJECT_DEF_NORMAL(primaryInteface, objID, objName) \
	IPL_BEGIN_VTABLE_MAP(selftype) \
		IPL_INTERFACE_ENTRY(IObject) \
		IPL_INTERFACE_ENTRY(primaryInteface) \
		IPL_INTERFACE_ENTRY(interface2) \
		IPL_INTERFACE_ENTRY(interface3) \
	IPL_END_VTABLE_MAP

#define IPL_OBJECT_IMP4(selftype,primaryInteface,interface2,interface3, interface4, objID, objName) \
	IPL_OBJECT_DEF_NORMAL(primaryInteface, objID, objName) \
	IPL_BEGIN_VTABLE_MAP(selftype) \
	IPL_INTERFACE_ENTRY(IObject) \
	IPL_INTERFACE_ENTRY(primaryInteface) \
	IPL_INTERFACE_ENTRY(interface2) \
	IPL_INTERFACE_ENTRY(interface3) \
	IPL_INTERFACE_ENTRY(interface4) \
	IPL_END_VTABLE_MAP

#define IPL_PTR_CAST(T,p) 	      ((p != NULL)? (T*)((p)->queryInterface(#T)) : 0)

	//通过宏，来注册插件对象，每个dll都有一个全局instance，自动生成两个函数
	//1）得到插件对象函数
	//2）得到插件编译期间的版本号，以用作兼容性判断

	interface IPlatform;

#define IPL_REGISTER_PLUGIN(pluginclass) \
	static pluginclass g_pluginInstance; \
	static IPlatform *g_platformInstance;\
	extern "C" PLUGIN_API IPlugin *iplGetPluginInstance(IPlatform *platformInstance)\
    { 	 g_platformInstance = platformInstance; return &g_pluginInstance;}\
	extern "C" PLUGIN_API const iplChar *iplGetPlatformMajorVersion(){return IPL_PLATFORM_VERSION;}\
	IPlatform* getPlatform()	{return g_platformInstance;}


//常用服务
//注册服务
#define IPL_registerServiceName		"ipl.service.register" 

#define IPL_pluginManagerName		"ipl.service.pluginManager" 

//全局日志服务
#define IPL_logServiceName			"ipl.service.log" 

//错误处理服务
#define IPL_lastErrorServiceName	"ipl.service.lastError"

//(name,value)的实现
#define IPL_keywordProperty			"ipl.property.keyword"

//XML属性二进制序列化器
#define IPL_binaryPropertySerialize	"ipl.propertySerialize.binary"
#define IPL_xmlPropertySerialize	"ipl.propertySerialize.binary"

}
