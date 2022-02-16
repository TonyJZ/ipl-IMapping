#pragma once


#include "core/ipldef.h"
#include "core/iplstd.h"

namespace ipl
{
	interface IProperty;

	interface IObject
	{
		// ��������������
	public: //��boost����ָ�룬��������������������
		virtual ~IObject() {};

	public:
		
		// ����, ������
		virtual iplString	getClassName()	const { return _T("Unknown"); };

		// ��λ�����ID, ���ֶ����Ψһ���, ʵ��ID�ɽӿ�ID�Ͷ���ID���Ӷ���
		// ����������eclipse���ƣ��ɶ���ID�Ĳ�����ֿ��Թ��������
		virtual iplString	getClassID()	const { return _T("ipl"); }

		// �õ�detailed description
		virtual iplString	getClassDesc() const { return _T("empty"); }

		// �õ�ʵ�ֵĽӿ�����
		virtual iplArray<iplString> getInterfaceNames() const = 0;

		// ���ݽӿ���ȡ����ָ��
		virtual void *queryInterface(const iplChar *className)  const = 0;

		///////////////////// ���Բ��� ////////////////////////////////////////////
		// �õ�����, �������ڻ�ȡ�ڲ�����, ���л���
		virtual const IProperty *getProperty() const { return NULL; };

		// ���ڴ�xml�ָ�����״̬��xml->property->object
		virtual bool initFromProperty(IProperty *property) { return true; };

		IProperty *getProperty()
		{
			return const_cast<IProperty *> ((static_cast<const IObject&>(*this)).getProperty());
		};


		// 	//��������
		// 	virtual void propertyChangeNotify() {} ;
		// 	
		// 	virtual bool checkAttr(const iplChar * name,	ors_int16 value, ors_int32 index  ) { return true;	};
		// 	virtual bool checkAttr(const iplChar * name,	ors_int32 value, ors_int32 index  ) { return true;	};
		// 	virtual bool checkAttr(const iplChar * name,	ors_float64 value, ors_int32 index  )  { return true;	};
		// 	virtual bool checkAttr(const iplChar * name,	const iplChar * value, ors_int32 index  )  { return true;	} ;
		// 	
		// 	// �õ�����Լ��
		// 	virtual iplString getAttrContrainedDescription( const iplChar * name ) const { return _T("empty");}

	};



	//
	// �����������Ϣ
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

