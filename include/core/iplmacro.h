#pragma once

#include <string>
#include "core/ipldef.h"
#include "core/iplstd.h"

namespace ipl
{
	//
	// �ӿ�����"���ӿ���" + "." + "���ӿ���" ����
	//
	// �����Ǽ̳���IObject, ������ظ����崿�麯��������б����ģ��������
	//
#define IPL_INTERFACE_DEF(parentInteface, interfaceID) public:\
	virtual iplString	getClassID() const { return iplString(parentInteface::getClassID() + _T(".") + interfaceID);}\
	virtual void* queryInterface(const iplChar *className) const = 0;\
	virtual iplArray<iplString> getInterfaceNames() const = 0;
	//virtual iplString	getClassName(){ return iplString(interfaceName);}

	//2018.1.15 ��չ�㣬֧�ֿ缶��չ�����������ӿ���չ
	//�궨�壬Ϊ��ʵ��IObject�Ľӿڣ���������Ҫ�Լ�ʵ�����ͷ�ļ��м���
	//IPL_OBJECT_IMP0 : �Լ�����ֱ�Ӵ�IObject�̳ж�������
	//IPL_OBJECT_IMP1 : �Լ������һ�����ӿڼ̳ж�������
	//IPL_OBJECT_IMP2 : �Լ����಻��ʵ����һ�����ӿڣ���ʵ����1�����ӿڣ���
	//IPL_OBJECT_IMP3 : �Լ����಻��ʵ����һ�����ӿڣ���ʵ����2�����ӿ�
	//һ����ԣ�IPL_OBJECT_IMP1��ʹ�ã�����������һ�㲻ʹ��

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

	// ��IObject�������麯���ĺ�ʵ��
	// ��������"���ӿ���" + "." + "���������" ����
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

	//ͨ���꣬��ע��������ÿ��dll����һ��ȫ��instance���Զ�������������
	//1���õ����������
	//2���õ���������ڼ�İ汾�ţ��������������ж�

	interface IPlatform;

#define IPL_REGISTER_PLUGIN(pluginclass) \
	static pluginclass g_pluginInstance; \
	static IPlatform *g_platformInstance;\
	extern "C" PLUGIN_API IPlugin *iplGetPluginInstance(IPlatform *platformInstance)\
    { 	 g_platformInstance = platformInstance; return &g_pluginInstance;}\
	extern "C" PLUGIN_API const iplChar *iplGetPlatformMajorVersion(){return IPL_PLATFORM_VERSION;}\
	IPlatform* getPlatform()	{return g_platformInstance;}


//���÷���
//ע�����
#define IPL_registerServiceName		"ipl.service.register" 

#define IPL_pluginManagerName		"ipl.service.pluginManager" 

//ȫ����־����
#define IPL_logServiceName			"ipl.service.log" 

//���������
#define IPL_lastErrorServiceName	"ipl.service.lastError"

//(name,value)��ʵ��
#define IPL_keywordProperty			"ipl.property.keyword"

//XML���Զ��������л���
#define IPL_binaryPropertySerialize	"ipl.propertySerialize.binary"
#define IPL_xmlPropertySerialize	"ipl.propertySerialize.binary"

}
