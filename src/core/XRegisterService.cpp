/********************************************************************

	purpose:
				ʵ�ֲ���ĵǼǷ���

*********************************************************************/

// #include "orsBase/orsConstDef.h"
// #include "orsBase/orsUtil.h"
// #include "orsBase/orsILastErrorService.h"


#include "commonAPIs/iplutility.h"
#include "XRegisterService.h"
#include "XXMLSerialize.h"
#include "XProperty.h"

// #include "orsBase/orsTypedef.h"
// 
// #include "orsBase/iplString.h"

using namespace ipl;
extern IPlatform * _getPlatform();

//��������л��õı�ǩ

#define IPL_Property					"Property"

#define IPL_OBJECT_ID					"ID"

#define IPL_OBJECT_NAME					"Name"

#define IPL_OBJECT_DESC					"Desc"

#define IPL_OBJECT_PARENT_INTERACE		"Interface"

#define IPL_OBJECT_PROPERTYS			"Property"

#define IPL_PLUGIN_TREE_FIRST_START_UP	"First_start_up"

#define IPL_PLUGIN						"Plugin"

#define IPL_PLUGIN_NAME					"Name"

#define IPL_OBJECT						"Object"

#define IPL_PLUGIN_VERSION				"Version"

#define IPL_PLUGIN_PROVIDER				"Provider"

#define IPL_PLUGIN_ID					"ID"


//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////

bool g_lazyLoad = true;

void XRegisterService::readXML()
{
	//1)����XML�����
	iplString  filename = getPluginDir().c_str();
	filename += "/";
	filename += getPluginTreeFileName().c_str();

	FILE *fp;
	if( g_lazyLoad )
		fp = fopen(filename.c_str(), "rb" );
	else
		fp = NULL;

	if( fp != NULL )
	{
		//�õ��ļ���С
		fseek(fp,0,SEEK_END);
		long file_len = ftell(fp);

		fseek(fp,0,SEEK_SET);

		//��ȡ��Ϣ
		char *buf = new char[file_len+10];
		memset(buf,0,file_len+10);
		fread(buf, sizeof(char), file_len, fp);

//		printf(buf);

		//������Ϣ
		ref_ptr<IPropertySerialize> serializer(new XXMLSerialize);
		ref_ptr<IProperty> rootP ( new XProperty);

		serializer->import( buf, file_len+10, rootP.get() );
		rootP->getAttr( "OpenRS_Plugin_System", m_objTree);

		delete buf;
		fclose(fp);
	}
}

XRegisterService::XRegisterService()
{
	m_runningPlugin = NULL;
	m_objTree = ref_ptr<IProperty>(new XProperty);
	m_needSave = false;
}

XRegisterService::~XRegisterService()
{

}

iplString XRegisterService::getPluginTreeFileName()
{
#ifdef _WIN32
	return "plugintree.xml";
#elif WIN32
	return "plugintree.xml";
#elif WIN64
	return "plugintree.xml";
#elif _WIN64
	return "plugintree.xml";
#else
	return "plugintree_linux.xml";
#endif
}

void XRegisterService::writeXML()
{
	//1)����XML�����
	iplString  filename = getPluginDir().c_str();
	filename += "/";
	filename += getPluginTreeFileName().c_str();

	FILE *fp = fopen(filename.c_str(),"wb");
	if(fp != NULL)
	{
		//������Ϣ
		char *buf = NULL;
		long len = 0;
		ref_ptr<IPropertySerialize> serializer(new XXMLSerialize);
		ref_ptr<IProperty> rootP(new XProperty);

		rootP->addAttr( "OpenRS_Plugin_System", m_objTree );

		serializer->outport(buf, &len, rootP.get() );

		fwrite(buf, sizeof(char), len, fp);

		if( NULL != buf )
			delete buf;

		fclose(fp);
	}
}

void XRegisterService::updateDescFromPluginProperty( const IProperty *pluginProperty )
{
	iplString	pluginName;
	iplString	pluginVersion;
	iplString	pluginID;
	iplString	pluginProvider;

	iplArray<ref_ptr<IProperty> > objNodes;

	pluginProperty->getAttr(IPL_PLUGIN_NAME, pluginName);
	pluginProperty->getAttr(IPL_PLUGIN_VERSION, pluginVersion);
	pluginProperty->getAttr(IPL_OBJECT, objNodes );
	pluginProperty->getAttr(IPL_PLUGIN_PROVIDER, pluginProvider);
	pluginProperty->getAttr(IPL_PLUGIN_ID, pluginID );

	//�������Ϣ��������Ϣ������
	iplPluginInfo pluginInfo;
	pluginInfo.pluginName = pluginName;
	pluginInfo.pluginVersion = pluginVersion;
	pluginInfo.pluginProvider = pluginProvider;
	pluginInfo.pluginID = pluginID;

	// ɾ��ԭ��Ϣ
	iplArray<iplPluginInfo>::iterator iter;
	for(iter = m_registeredPluginInfos.begin(); iter != m_registeredPluginInfos.end(); iter++)
	{
		if( iter->pluginName == pluginName )
		{
			m_registeredPluginInfos.erase( iter );
			break;
		}
	}

	// ������Ϣ
	m_registeredPluginInfos.push_back(pluginInfo);

	for(unsigned j=0;j<objNodes.size();j++)
	{
		IProperty *objNode = objNodes[j].get();
		XObjectDesc* objDesc = new XObjectDesc;

		objNode->getAttr(IPL_OBJECT_NAME, objDesc->m_name);
		objNode->getAttr(IPL_OBJECT_ID, objDesc->m_id );
		objNode->getAttr(IPL_OBJECT_DESC, objDesc->m_desc );
		objNode->getAttr(IPL_OBJECT_PARENT_INTERACE, objDesc->m_interfaceNames );

		//ȡ�������������л��洢
		//objNode->getAttr(IPL_OBJECT_PROPERTYS,objDesc->m_propertys);

		objDesc->m_pluginName = pluginName;

//		assert( !objDesc->m_id.isEmpty() );

		if( !objDesc->m_id.empty() )	{
			m_objDescMap.erase(	objDesc->m_id.c_str() );
			m_objDescMap.insert(
				iplObjDescMap::value_type( objDesc->m_id.c_str(), ref_ptr<IObjectDesc>(objDesc)) );
		}
		else	{
//			_getPlatform()->logPrint( IPL_LOG_WARNING, "Empty Object ID, name = %s\n", objDesc->m_name.c_str() );
			assert( !objDesc->m_id.empty() );
		}
	}
}

ref_ptr<ILogService> XRegisterService::getLogService()
{
	return _getPlatform()->getLogService();
}

bool XRegisterService::startup(IPlatform *platform)
{
	getLogService()->debug( "iplCore", "XRegisterService: startup" );

	//////////////////////////////////////////////////////////////////////////

	//����ƽָ̨��
	m_platform = platform;

	//����XML�����
	readXML();

	//�ж��Ƿ��ǵ�һ�μ���
	iplString isfirst = "false";

	m_objTree->getAttr(IPL_PLUGIN_TREE_FIRST_START_UP,isfirst);
	if(isfirst == "false"){

		getLogService()->info( "iplCore", "Scan Plugins" );

		//ɨ��Ŀ¼
		scanDir( getPluginDir().c_str(), false );

		//�����һ�μ��ر�ʾ
		m_objTree->addAttr(IPL_PLUGIN_TREE_FIRST_START_UP,"true");

		//��Ҫ����
		setSaveFlag();
	}

	//���ɶ�������Map
	iplArray<ref_ptr<IProperty> > pluginNodes;
	m_objTree->getAttr( IPL_PLUGIN, pluginNodes);
	for(unsigned i=0;i<pluginNodes.size();i++)
	{
		updateDescFromPluginProperty( pluginNodes[i].get() );
	}


	return true;
}


void XRegisterService::shutdown()
{
	//if(m_needSave)
	//	writeXML();

	getLogService()->debug( "iplCore", "XRegisterService: shutdown" );

	m_creatorMap.clear();

	m_objTree = NULL;

	m_runningPlugin = NULL;

	m_objDescMap.clear();

	m_loadedPlugins.clear();

	m_serializer = NULL;

}

bool XRegisterService::isok()
{
	return true;
}

//
// ���ö��󴴽���ע�����,
//
//

extern const iplChar *g_curModuleName;

bool XRegisterService::registerObject(IPL_OBJECT_CREATOR_FUN objCreator )
{
	// ������ʱ����
	ref_ptr<IObject> obj = ref_ptr<IObject>(objCreator(true));

	if( NULL == obj.get() ) {
		m_platform->logPrint( IPL_LOG_WARNING, "Failed Creating Object!");
//		m_platform->getLastErrorService()->setErrorInfo(0, "Failed Creating Object!");
		return false;
	}

	// ȡ����ID
	iplString objName = obj->getClassName();
	iplString objID = obj->getClassID();
	if( objID.empty() )
		return false;

	// ���󴴽�map
	m_creatorMap.erase( objID.c_str() );
	m_creatorMap.insert( iplObjCreatorMap::value_type( objID.c_str(), objCreator ) );

	if( NULL == m_runningPlugin.get() && NULL != g_curModuleName )
	{
		iplObjDescMap::iterator iter;

		// �Ƿ��Ѿ�ע���
		iter = m_objDescMap.find( objID.c_str() );

		// ע��module�еĶ���
		if( iter == m_objDescMap.end() )
		{
			// �ڲ�����ע�ᣬ�����ڶ�����������û���ҵ�
			XObjectDesc *objDesc(new XObjectDesc);
			
			// ���������ɽӿ�����ʵ��������
			objDesc->m_name = objName;
			objDesc->m_id = objID;
			objDesc->m_desc= obj->getClassDesc();
			//if(m_runningPlugin != NULL)
			
			objDesc->m_pluginName = g_curModuleName;
			
			// �ӿ���
			objDesc->m_interfaceNames = obj->getInterfaceNames();
			objDesc->m_propertys = ref_ptr<IProperty>(obj->getProperty());
			
			m_objDescMap.insert(
				iplObjDescMap::value_type( objID.c_str(), ref_ptr<IObjectDesc>(objDesc) ) );
		}
	}

	if( m_runningPlugin != NULL ) {
		m_runningPlugin->m_objectCreators.erase( objID.c_str());
		m_runningPlugin->m_objectCreators.insert(
			std::map<iplString , IPL_OBJECT_CREATOR_FUN>::value_type( objID.c_str(), objCreator) );
	}

	return true;
}


IObject *XRegisterService::createObject(const iplChar * objectID)
{
	IObject* obj = NULL;

	if (NULL == objectID) 
		return NULL;

	iplObjCreatorMap::iterator iter = m_creatorMap.find( objectID );

	if(iter == m_creatorMap.end()){
		//���û���ҵ�������
		//�ж��Ƿ��ж��������
		iplObjDescMap::iterator iter;

		iter = m_objDescMap.find( objectID );
		//���û������������NULL
		if(iter == m_objDescMap.end()) {
			iplString msg = "Fail to create object: ";
			msg = msg + objectID;

			getLogService()->error( "iplCore", msg.c_str() );

			return NULL;
		}
		else{
			ref_ptr<IObjectDesc> description = iter->second;
			iplString pluginName = description->getPluginName();

			if( !addPlugin( pluginName.c_str(), false ) ) {
				iplString msg = "Fail to create object: ";
				msg = msg + objectID;
				getLogService()->error( "iplCore", msg.c_str() );

				return NULL;
			}

			//�ٴ��ҵ������ߣ�OK
			iplObjCreatorMap::iterator iter2 = m_creatorMap.find( objectID );
			if(iter2 == m_creatorMap.end())
				return NULL;
			IPL_OBJECT_CREATOR_FUN creator = iter2->second;
			obj = creator(false);
		}
	}else {
		//����ҵ������ߣ�OK
		IPL_OBJECT_CREATOR_FUN creator = iter->second;
		obj = creator(false);
	}

	return obj;
}

iplString XRegisterService::getPluginDir()
{
	iplString  dir;

	get_directory_exe(dir);
	dir = dir + "/" + "plugins";
	return dir.c_str();
}

iplString XRegisterService::getPluginNameByPath(const iplChar * pluginPath)
{
	iplString  pluginName;
	iplString  pluginDir = getPluginDir().c_str();
	iplString  pluginPathSTL = pluginPath;
	for(unsigned i=pluginDir.length()+1;i<pluginPathSTL.size();i++)
	{
		pluginName += pluginPathSTL[i];
	}

	return pluginName.c_str();
}

iplString XRegisterService::getPluginPathByName(const iplChar * pluginName)
{
	iplString  pluginDir = getPluginDir().c_str();
	iplString  name = pluginDir + "/" + pluginName;
	return name.c_str();
}

bool XRegisterService::addPlugin(const iplChar * pluginName, bool isForced)
{
	//�õ������·����
	iplString pathName = getPluginPathByName(pluginName);

	//�ж��Ƿ��������ع�
	std::map<iplString ,ref_ptr<iplPluginDLL> >::iterator iter =
		m_loadedPlugins.find(pluginName);

	if(iter != m_loadedPlugins.end()){
		return false;
	}

	// ������release���ֹ��������debug��Ϊ��λ������ò�Ҫ�����쳣
#ifndef _DEBUG
	try
#endif	
	{
		//�������
		ref_ptr<iplPluginDLL> plugin ( new iplPluginDLL( pluginName, pathName.c_str() ));
		
		//���øò��Ϊ��ǰ���
		m_runningPlugin = plugin;
		
		// ���ز��
		if( plugin->load(m_platform) )
		{

			//���뵽���ʵ��map��
			m_loadedPlugins.erase( pluginName );
			m_loadedPlugins.insert( iplPluginDLLMap::value_type( pluginName, m_runningPlugin) );
			
			//����XML�������Ϣ
			if( isForced ){
				//�����ǿ�Ƽ���
				//���֮ǰ������Ѿ����ڸýڵ㣬��ɾ��
				if(pluginExistInTree( pluginName) )
					unSerializePlugin(pluginName);
				
				serializePlugin( m_objTree.get(), plugin.get() );
			}else{
				//����Ƿ�ǿ�Ƽ���
				//���֮ǰ������Ѿ����ڸýڵ㣬���ù�
				//���û�д��ڣ�����
				if( !pluginExistInTree(pluginName) )
					serializePlugin( m_objTree.get(), plugin.get() );
			}

			return true;
		}
		else	{
			iplString msg = "Fail to add plugin: ";
			msg = msg + pluginName;
			
			getLogService()->warn( "iplCore", msg.c_str() );
			
			return false;
		}
	}
#ifndef _DEBUG
	catch ( ... )
	{
		iplString msg = "Exception during adding plugin: ";
		msg = msg + pluginName;

		getLogService()->error( "iplCore", msg.c_str() );
	}
#endif

	return true;
}

//Ŀǰ���ṩ��̬ж�ط���
bool XRegisterService::removePlugin(const iplChar * pluginName,bool isForced)
{
//	m_platform->getLastErrorService()->setErrorInfo(0,"remove plugin isn't support");
	return false;
}


bool XRegisterService::scanDir(const iplChar * dirName,bool isForced)
{
	iplString dir = getPluginDir();
	
	printf("scan dir :%s\n",dir.c_str());

	//�ҵ�Ŀ¼�����е�dll
	iplArray<iplString> allDlls;
	
	findAllFiles(dir.c_str(),PLUGIN_EXTENSION, allDlls );

	//���ز��
	for(unsigned i=0;i<allDlls.size();i++)
	{
		iplString &pathName = allDlls[i];
		iplString pluginName = getPluginNameByPath( pathName.c_str() );
		const  char* name = pluginName.c_str();
		addPlugin( pluginName.c_str(), isForced);
		printf("plugin dll load :%s\n",pluginName.c_str());
	}

	return true;
}

IObjectDesc*  XRegisterService::getObjectDescByID(const iplChar *objId )
{
	iplObjDescMap::iterator iter;
	iter = m_objDescMap.find( objId );
	if(iter != m_objDescMap.end())
		return iter->second.get();
	else
		return NULL;
}

iplArray<IObjectDesc* > XRegisterService::getAllObjectDescs()
{
	iplArray<IObjectDesc* > descriptions;
	iplObjDescMap::iterator iter;

	for(iter = m_objDescMap.begin();iter != m_objDescMap.end();iter++)
	{
		descriptions.push_back(iter->second.get());
	}
	return descriptions;
}

iplArray<IObjectDesc* > XRegisterService::getObjectDescsByInterface( const iplChar *interfaceName)
{
	iplArray<IObjectDesc* > descriptions;

	iplObjDescMap::iterator iter;
	for(iter = m_objDescMap.begin(); iter != m_objDescMap.end();iter++)
	{
		IObjectDesc *pDesc = iter->second.get();

		iplArray<iplString> interfaceNames = pDesc->getInterfaceNames();
		for(unsigned i=0;i<interfaceNames.size();i++)
		{
			iplString &name = interfaceNames[i];
			if(name == interfaceName){
				descriptions.push_back(pDesc );
				break;
			}
		}
	}
	return descriptions;
}

void XRegisterService::serializePlugin( IProperty *pluginTree, iplPluginDLL *plugin)
{
	iplString  msg = "Serialize Plugin : " + plugin->m_pluginName;

	getLogService()->debug( "iplCore", msg.c_str() );

	//���������Ϣ
	const iplChar * name = plugin->m_pluginName.c_str();
	const iplObjCreatorMap &objectCreators = plugin->m_objectCreators;

	//���л�����
	ref_ptr<IProperty> pluginNode( new XProperty);
	pluginNode->addAttr( IPL_PLUGIN_NAME, name );
	pluginNode->addAttr(IPL_PLUGIN_ID,plugin->m_plugin->getID());

	//����汾��,ID,�ṩ��
	pluginNode->addAttr( IPL_PLUGIN_VERSION,plugin->m_plugin->getVersion());
	pluginNode->addAttr(IPL_PLUGIN_PROVIDER,plugin->m_plugin->getProvider());

	//���л�objects
	for(iplObjCreatorMap::const_iterator iter = objectCreators.begin(); iter != objectCreators.end(); iter++)
	{
		IPL_OBJECT_CREATOR_FUN creator = iter->second;

		ref_ptr<IObject> obj = ref_ptr<IObject>(creator(true));

		serializeObject( pluginNode.get(), obj.get() );
	}

	//������
	pluginTree->addAttr( IPL_PLUGIN, pluginNode );

	//���¶���������
	updateDescFromPluginProperty( pluginNode.get() );

	//���ñ����ǩ
	setSaveFlag();
}

void XRegisterService::unSerializePlugin(const iplChar * pluginName)
{
	iplString msg = "unSerialize Plugin : ";
	msg += pluginName;

	getLogService()->debug( "iplCore", msg.c_str() );

	//�ҵ��ò��
	std::map<iplString ,ref_ptr<iplPluginDLL> >::iterator iter;
	iter = m_loadedPlugins.find( pluginName );
	if(iter == m_loadedPlugins.end())
		return;

	ref_ptr<iplPluginDLL> plugin = iter->second;

	//ȥ�����ʵ��map�в��
	m_loadedPlugins.erase( pluginName );

	//ȥ���ò���Ķ��󹤳�
	for(iplObjCreatorMap::iterator iter2 = plugin->m_objectCreators.begin();
		iter2 != plugin->m_objectCreators.end();iter2++)
	{
		iplString  name = iter2->first;
		m_creatorMap.erase(name);
		m_objDescMap.erase(name);
	}
}

void XRegisterService::serializeObject( IProperty *pluginNode, IObject *obj)
{
	ref_ptr<IProperty> objNode( new XProperty);
	assert(objNode != NULL);

	// ���л�������Ϣ

	objNode->addAttr( IPL_OBJECT_NAME, obj->getClassName().c_str() );
	objNode->addAttr( IPL_OBJECT_ID, obj->getClassID().c_str() );
	objNode->addAttr( IPL_OBJECT_DESC, obj->getClassDesc().c_str() );

	iplArray<iplString> interfaces = obj->getInterfaceNames();
	for(unsigned i=0;i<interfaces.size(); i++)
	{
		objNode->addAttr(IPL_OBJECT_PARENT_INTERACE, interfaces[i].c_str() );
	}

	//ref_ptr<IProperty> ps = obj->getProperty();

	//gw�޸�2008.11.11,��ʵ���������л�
	//if( obj->getProperty() != NULL )
	//	objNode->addAttr(IPL_OBJECT_PROPERTYS,obj->getProperty());

	pluginNode->addAttr( IPL_OBJECT, objNode);

	//��Ҫ���±���
	m_needSave = true;
}

bool XRegisterService::pluginExistInTree(const iplChar * pluginName)
{
	iplArray<ref_ptr<IProperty> > pluginNodes;

	m_objTree->getAttr(IPL_PLUGIN,pluginNodes);

	for(unsigned i=0;i<pluginNodes.size();i++)
	{
		ref_ptr<IProperty> pluginNode = pluginNodes[i];

		iplString name;

		pluginNode->getAttr(IPL_PLUGIN_NAME,name);

		if(name == pluginName)
			return true;
	}

	return false;
}

iplArray<iplPluginInfo> &XRegisterService::getPluginInfos()
{
	return m_registeredPluginInfos;
}

bool XRegisterService::getPluginInfo(const iplChar *pluginName,iplPluginInfo &info)
{
	for ( unsigned i=0;i<m_registeredPluginInfos.size();i++)
	{
		iplPluginInfo &_info = m_registeredPluginInfos[i];
		if(_info.pluginName == pluginName)
		{
			info = _info;
			return true;
		}
	}

	return false;
}
