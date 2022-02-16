#pragma once

#include <map>


#include "iplCommon.h"
#include "core/interface/IPlugin.h"
#include "core/interface/IObject.h"
#include "core/iplmacro.h"

namespace ipl
{
	typedef std::map<iplString, IPL_OBJECT_CREATOR_FUN> iplObjCreatorMap;

	class iplPluginDLL : public IObject
	{
	public:
		iplPluginDLL(const iplChar * name, const iplChar * path);
		virtual ~iplPluginDLL();

		//���������ط���
	public:
		bool load(IPlatform* platform);
		void unload();

	public:
		iplString 			m_pluginName;
		iplString 			m_pathName;

		iplObjCreatorMap	m_objectCreators;

		iplPluginHandle		m_hDLL;

		IPlugin*			m_plugin;

	public:

		//�ӿڶ���
		IPL_OBJECT_IMP0(iplPluginDLL, _T("plugindll"), _T("plugindll"));
	};


	typedef std::map<iplString, ref_ptr<iplPluginDLL> >	 iplPluginDLLMap;


}

