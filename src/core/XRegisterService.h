#pragma once


#include "core/interface/IPlatform.h"
#include "core/interface/IProperty.h"
#include "core/interface/ILogService.h"
#include "core/interface/IRegisterService.h"


#include "iplPluginDLL.h"


namespace ipl
{
	//��������
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
		iplString				m_name;			// ��������
		iplString				m_id;			// ����ID, ���ֶ����Ψһ���
		iplString				m_desc;			// ��������
		iplString				m_pluginName;		// �������ڵĲ����dll���ļ���

		ref_ptr<IProperty>	m_propertys;		// ��������
		iplArray<iplString>		m_interfaceNames;	// ����ӿ�����
	};


	typedef std::map<iplString, ref_ptr<IObjectDesc> >	iplObjDescMap;


	//�㷨ע�����
	class XRegisterService : public IRegisterService, public IPluginManager
	{
	public:
		XRegisterService();
		virtual ~XRegisterService();

		//����ӿ�
		//����ʱ������
		virtual bool startup(IPlatform *platform);

		//�ر�ʱ������
		virtual void shutdown();

		//�Ƿ������ɹ�
		virtual bool isok();

		//ע���������֮ǰ�У����滻֮ǰ�Ĺ��캯��
		//ע�⣬�ýӿڿɱ�ϵͳ�ڲ����ã�����һ���Ǿ����������
		//��ϵͳ����ʱ��ϵͳ���ö��󽫼��뵽ע������У����������л���
		bool registerObject(IPL_OBJECT_CREATOR_FUN objCreator);

		//�����������isForced���ر��򿪣���ǿ�Ƹ��²��XML��
		bool addPlugin(const iplChar * pluginName, bool isForced = false);

		//�Ƴ���������isForced���ر��򿪣���ǿ�Ƹ��²��XML��
		bool removePlugin(const iplChar * pluginName, bool isForced = false);

		//�õ���������Ϣ��������汾�ţ����汾�ţ�
		iplArray<iplPluginInfo> &getPluginInfos();

		bool getPluginInfo(const iplChar *pluginName, iplPluginInfo &info);

		//scan�ļ����ڲ����еĲ��
		bool scanDir(const iplChar * dirName, bool isForced = false);

		//�õ����Ŀ¼
		iplString getPluginDir();

		//�õ�������ļ�����
		iplString getPluginTreeFileName();


		// ͨ���������Ƶõ�����
		virtual IObjectDesc* getObjectDescByID(const iplChar * objID);

		// �õ����еĶ�������
		virtual iplArray<IObjectDesc* > getAllObjectDescs();

		// �õ�����ĳ���ӿڵ��㷨����
		//!!!!!ע�⣬interfaceClass��interface�������ƣ������ǽӿڵ�����
		virtual iplArray<IObjectDesc* > getObjectDescsByInterface(const iplChar * interfaceName);

		// ��������ʵ��
		virtual IObject *createObject(const iplChar *objectID);

	protected:
		//�жϲ���Ƿ��ڲ��XML���ϴ���
		bool pluginExistInTree(const iplChar * pluginName);

		//��ȡXML�����
		void writeXML();

		//дXML�����
		void readXML();

		//�õ�����ڵ�
		//ref_ptr<IProperty> findPluginNode(const iplChar * pluginName);

		//���л����������÷���������ʱ�䣬shutdownʱ�ᱣ��
		void serializePlugin(IProperty *pluginTree, iplPluginDLL *plugin);

		//��XML�����Ƴ�һ���������
		void unSerializePlugin(const iplChar * pluginName);

		//���л�����ڵ��㷨����(��serializePlugin����)
		void serializeObject(IProperty *pluginNode, IObject *obj);

		//ͨ��������Ƶõ�����ľ���·��
		iplString getPluginPathByName(const iplChar * pluginPath);

		//ͨ������·���õ������
		iplString getPluginNameByPath(const iplChar * pathName);

		//���ñ����־����shutupʱ������save
		void setSaveFlag() { m_needSave = true; }

		//��xml�ڵ��ڳ�ȡ���������������뵽����Map��
		void updateDescFromPluginProperty(const IProperty *node);

	private:
		//���XML�����仯������Ҫ���±���m_keywordList
		bool	m_needSave;

		// �����
		ref_ptr<IProperty>	m_objTree;

		// XML���л���
		ref_ptr<IPropertySerialize> m_serializer;

		// ��ǰ��־
		ref_ptr<ILogService> getLogService();

	private:
		// �Ѽ��صĲ��(DLL)�г�ȡ��object������ (����������������ָ��)
		iplObjCreatorMap	m_creatorMap;

		//��������Map(����������������)
		iplObjDescMap m_objDescMap;

		// ��ǰ�ڼ��صĲ��,����̽�����ڲ�����Щobject
		ref_ptr<iplPluginDLL>  m_runningPlugin;

		// �Ѽ��صĲ��ʵ�壨dll��map
		iplPluginDLLMap m_loadedPlugins;

		// ע�����������Ϣ
		iplArray<iplPluginInfo> m_registeredPluginInfos;

	private:

		//ϵͳƽ̨
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
