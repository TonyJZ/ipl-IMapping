#pragma once


#include "core/interface/IPlatform.h"
#include "core/interface/IProperty.h"
#include "core/interface/ILogService.h"
#include "core/interface/IRegisterService.h"


#include "iplPluginDLL.h"


namespace ipl
{
	//对象描述
	class XObjectDesc : public IObjectDesc
	{
	public:
// 		virtual void addRef() { internalAddRef(); }
// 		virtual void release() { internalRelease(); }

	public:
		virtual const iplChar *getName() { return m_name.c_str(); }
		virtual const iplChar *getID() { return m_id.c_str(); }
		virtual const iplChar *getDesc() { return m_desc.c_str(); }
		virtual const iplChar *getPluginName() { return m_pluginName.c_str(); }

		virtual const iplArray<iplString>	&getInterfaceNames() const { return m_interfaceNames; }

	public:
		iplString				m_name;			// 对象名字
		iplString				m_id;			// 对象ID, 区分对象的唯一标记
		iplString				m_desc;			// 对象描述
		iplString				m_pluginName;		// 对象所在的插件（dll）文件名

		ref_ptr<IProperty>	m_propertys;		// 对象属性
		iplArray<iplString>		m_interfaceNames;	// 对象接口名字
	};


	typedef std::map<iplString, ref_ptr<IObjectDesc> >	iplObjDescMap;


	//算法注册服务
	class XRegisterService : public IRegisterService, public IPluginManager
	{
	public:
		XRegisterService();
		virtual ~XRegisterService();

		//服务接口
		//启动时被调用
		virtual bool startup(IPlatform *platform);

		//关闭时被调用
		virtual void shutdown();

		//是否启动成功
		virtual bool isok();

		//注册插件，如果之前有，则将替换之前的构造函数
		//注意，该接口可被系统内部调用，而不一定非经过插件调用
		//在系统启动时，系统内置对象将加入到注册服务中，如属性序列化器
		bool registerObject(IPL_OBJECT_CREATOR_FUN objCreator);

		//加入插件，如果isForced开关被打开，将强制更新插件XML树
		bool addPlugin(const iplChar * pluginName, bool isForced = false);

		//移除插件，如果isForced开关被打开，将强制更新插件XML树
		bool removePlugin(const iplChar * pluginName, bool isForced = false);

		//得到插件相关信息：（插件版本号，主版本号）
		iplArray<iplPluginInfo> &getPluginInfos();

		bool getPluginInfo(const iplChar *pluginName, iplPluginInfo &info);

		//scan文件夹内部所有的插件
		bool scanDir(const iplChar * dirName, bool isForced = false);

		//得到插件目录
		iplString getPluginDir();

		//得到插件树文件名称
		iplString getPluginTreeFileName();


		// 通过对象名称得到描述
		virtual IObjectDesc* getObjectDescByID(const iplChar * objID);

		// 得到所有的对象描述
		virtual iplArray<IObjectDesc* > getAllObjectDescs();

		// 得到兼容某个接口的算法描述
		//!!!!!注意，interfaceClass是interface的类名称，而不是接口的名字
		virtual iplArray<IObjectDesc* > getObjectDescsByInterface(const iplChar * interfaceName);

		// 创建对象实例
		virtual IObject *createObject(const iplChar *objectID);

	protected:
		//判断插件是否在插件XML树上存在
		bool pluginExistInTree(const iplChar * pluginName);

		//读取XML插件树
		void writeXML();

		//写XML插件树
		void readXML();

		//得到插件节点
		//ref_ptr<IProperty> findPluginNode(const iplChar * pluginName);

		//序列化方法，当该方法被调用时间，shutdown时会保存
		void serializePlugin(IProperty *pluginTree, iplPluginDLL *plugin);

		//在XML树上移除一个插件描述
		void unSerializePlugin(const iplChar * pluginName);

		//序列化插件内的算法对象(被serializePlugin调用)
		void serializeObject(IProperty *pluginNode, IObject *obj);

		//通过插件名称得到插件的绝对路径
		iplString getPluginPathByName(const iplChar * pluginPath);

		//通过绝对路径得到插件名
		iplString getPluginNameByPath(const iplChar * pathName);

		//设置保存标志，当shutup时，真正save
		void setSaveFlag() { m_needSave = true; }

		//从xml节点内抽取对象描述，并加入到描述Map中
		void updateDescFromPluginProperty(const IProperty *node);

	private:
		//如果XML产生变化，将需要重新保存m_keywordList
		bool	m_needSave;

		// 插件树
		ref_ptr<IProperty>	m_objTree;

		// XML序列化器
		ref_ptr<IPropertySerialize> m_serializer;

		// 当前日志
		ref_ptr<ILogService> getLogService();

	private:
		// 已加载的插件(DLL)中抽取的object创建者 (对象名，创建函数指针)
		iplObjCreatorMap	m_creatorMap;

		//对象描述Map(对象名，对象描述)
		iplObjDescMap m_objDescMap;

		// 当前在加载的插件,用于探测插件内部有哪些object
		ref_ptr<iplPluginDLL>  m_runningPlugin;

		// 已加载的插件实体（dll）map
		iplPluginDLLMap m_loadedPlugins;

		// 注册插件的相关信息
		iplArray<iplPluginInfo> m_registeredPluginInfos;

	private:

		//系统平台
		IPlatform *m_platform;

	public:

		IPL_OBJECT_DEF_NORMAL(IRegisterService, "default", "Register Service")

			IPL_BEGIN_VTABLE_MAP(XRegisterService)
			IPL_INTERFACE_ENTRY2(IObject, IRegisterService)
			IPL_INTERFACE_ENTRY(IRegisterService)
			IPL_INTERFACE_ENTRY(IPluginManager)
			IPL_INTERFACE_ENTRY(IService)
			IPL_END_VTABLE_MAP
	};

}
