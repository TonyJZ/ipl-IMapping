#pragma once
#include "core/interface/IObject.h"
#include "core/interface/IProperty.h"
#include "core/interface/IProgressMsg.h"
#include "core/iplmacro.h"

namespace ipl
{
	/////////////////////////////////////////////////////////////////////////

	// ��ִ�ж���ӿ�
	interface IExecutor : public IObject
	{
	public:
		// �����ļ�
		virtual ref_ptr<IProperty> getInputFileNames() = 0;

		// �ڲ�����
		virtual ref_ptr<IProperty> getParameterArgs() = 0;

		// ����ļ�
		virtual ref_ptr<IProperty> getOutputFileNames() = 0;

		virtual bool setArguments(ref_ptr<IProperty> inputFileNames, ref_ptr<IProperty> parameterArgs, ref_ptr<IProperty> outputFileNames) = 0;

		//////////////////////////////////////////////////////////////////////////
		// ���л��ӿڣ���orsIObject�Ѷ���
		// virtual const IProperty *getProperty() const = 0;

		//���������Ϣ
		// virtual bool initFromProperty( IProperty *property ) = 0;

		////////////////////////////////////////////////////////////
		// ȡ�Զ����㷨���ý���Ķ���ID. Ĭ��û���Զ������
		//
		// ע�⣺�����㷨�ڲ�ֱ��ʵ�ָ����ý��棬Ӧ�����ý�����Ϊһ����չ��GUI����ﵥ��ʵ�֡�
		//       ����Ӱ���㷨�������ֲ��
		//virtual const orsChar *getConfigDlg() { return NULL; };

		IPL_INTERFACE_DEF(IObject, _T("executor"))
	};


// 	interface orsIExeConfigDlg : public orsIExtension
// 	{
// 	public:
// 		virtual bool config(orsIExecute *pExeObj) = 0;
// 
// 		ORS_INTERFACE_DEF(orsIExtension, _T("exeConfigDlg"));
// 	};



	//////////////////////////////////////////////////////////////////////////

	//�ǲ����㷨�ӿ�
	interface ISimpleExe : public IExecutor
	{
	public:
		//����õ��������
		virtual ref_ptr<IProperty> execute(IProgressMsg *process) = 0;

		IPL_INTERFACE_DEF(IExecutor, _T("simple"))
	};

	// 
	// interface orsILicenseExe: public orsISimpleExe
	// {
	// public:
	// 	virtual bool executeAuthrize(enum orsServiceTYPE servertype) = 0;
	// 	ORS_INTERFACE_DEF( orsIExecute, _T("License") );
	// };


	//
	// ��������˹���������Ŀǰֻ���ڱ���ִ��
	//	 ����������ⲿ.exe�����޽�����ʾ
	//
// 	interface orsIGuiExe : public orsIExecute
// 	{
// 	public:
// 		//����õ��������
// 		virtual ref_ptr<IProperty> execute() = 0;
// 
// 		ORS_INTERFACE_DEF(orsIExecute, _T("gui"));
// 	};


	//
	// �ֲ�ʽ���д������
	//
	interface IParallelExe : public IExecutor
	{
	public:
		// ��� nTasks = 0, �㷨�Լ������������������ո���������������
		virtual iplArray<ref_ptr<IProperty> > getTasks(int nTasks = 0) = 0;

		//ͨ��Job��Ϣ��Task��Ϣ���м���õ������Ϣ,taskOutput���ⲿ�����ڴ�<map>
		virtual ref_ptr<IProperty> taskExecute(ref_ptr<IProperty> taskInput, IProgressMsg *process) = 0;

		//����������ϲ�<reduce>
		virtual ref_ptr<IProperty> taskCombine(iplArray<ref_ptr<IProperty> > taskInputs, IProgressMsg *process) = 0;

		IPL_INTERFACE_DEF(IExecutor, _T("parallel"));
	};

#define INPUT_FILE_NAMES _T("InputFileNames")
#define OUTPUT_FILE_NAMES _T("OutputFileNames")
#define PARAMETER_ARGS _T("ParameterArgs")

	template <typename IExeInteface>
	class IExeHelper : public IExeInteface
	{
	protected:
		IExeHelper(bool bForRegister)
		{
			if (!bForRegister) {
				m_jobArguments = getPlatform()->createProperty();

				m_inputFileNames = getPlatform()->createProperty();
				m_parameterArgs = getPlatform()->createProperty();
				m_outputFileNames = getPlatform()->createProperty();

				m_jobArguments->addAttr(INPUT_FILE_NAMES, m_inputFileNames);
				m_jobArguments->addAttr(OUTPUT_FILE_NAMES, m_outputFileNames);
				m_jobArguments->addAttr(PARAMETER_ARGS, m_parameterArgs);
			}
		}

	public:
		virtual ref_ptr<IProperty> getInputFileNames()
		{
			return m_inputFileNames;
		}

		// �ڲ�����
		virtual ref_ptr<IProperty> getParameterArgs()
		{
			return m_parameterArgs;
		}

		// ����ļ�
		virtual ref_ptr<IProperty> getOutputFileNames()
		{
			return m_outputFileNames;
		}

		virtual const ref_ptr<IProperty> getProperty() const
		{
			return m_jobArguments;
		}

		//���������Ϣ
		virtual bool initFromProperty(IProperty *property)
		{
			ref_ptr<IProperty> inputFileNames;
			ref_ptr<IProperty> parameterArgs;
			ref_ptr<IProperty> outputFileNames;

			property->getAttr(INPUT_FILE_NAMES, inputFileNames);
			property->getAttr(PARAMETER_ARGS, parameterArgs);
			property->getAttr(OUTPUT_FILE_NAMES, outputFileNames);

			// ���ݾɵĵ��÷�ʽ
			if (NULL == inputFileNames.get())
				inputFileNames = property;
			if (NULL == parameterArgs.get())
				parameterArgs = property;
			if (NULL == outputFileNames.get())
				outputFileNames = property;

			// �Զ������ڲ�����
			if (inputFileNames.get() != m_inputFileNames.get())
				m_inputFileNames->update(inputFileNames.get());

			if (outputFileNames.get() != m_outputFileNames.get())
				m_outputFileNames->update(outputFileNames.get());

			if (parameterArgs.get() != m_parameterArgs.get())
				m_parameterArgs->update(parameterArgs.get());

			return this->setArguments(inputFileNames.get(), parameterArgs.get(), outputFileNames.get());
		}

	protected:
		ref_ptr<IProperty> m_jobArguments;
		ref_ptr<IProperty> m_inputFileNames;
		ref_ptr<IProperty> m_parameterArgs;
		ref_ptr<IProperty> m_outputFileNames;

	};


	// ���н�����
#define IPL_PARALLEL_NUM_PROCESSES	_T("Num Of Processes")
#define IPL_MPI_CMD		_T("MPI_Command")

#define IPL_WebSERVICE	_T("Web Service")

}
