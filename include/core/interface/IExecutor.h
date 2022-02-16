#pragma once
#include "core/interface/IObject.h"
#include "core/interface/IProperty.h"
#include "core/interface/IProgressMsg.h"
#include "core/iplmacro.h"

namespace ipl
{
	/////////////////////////////////////////////////////////////////////////

	// 可执行对象接口
	interface IExecutor : public IObject
	{
	public:
		// 输入文件
		virtual ref_ptr<IProperty> getInputFileNames() = 0;

		// 内部参数
		virtual ref_ptr<IProperty> getParameterArgs() = 0;

		// 输出文件
		virtual ref_ptr<IProperty> getOutputFileNames() = 0;

		virtual bool setArguments(ref_ptr<IProperty> inputFileNames, ref_ptr<IProperty> parameterArgs, ref_ptr<IProperty> outputFileNames) = 0;

		//////////////////////////////////////////////////////////////////////////
		// 序列化接口，在orsIObject已定义
		// virtual const IProperty *getProperty() const = 0;

		//输入参数信息
		// virtual bool initFromProperty( IProperty *property ) = 0;

		////////////////////////////////////////////////////////////
		// 取自定义算法配置界面的对象ID. 默认没有自定义界面
		//
		// 注意：不在算法内部直接实现该配置界面，应把配置界面作为一个扩展在GUI插件里单独实现。
		//       否则影响算法插件的移植性
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

	//非并行算法接口
	interface ISimpleExe : public IExecutor
	{
	public:
		//计算得到输出参数
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
	// 带界面的人工交互程序，目前只能在本地执行
	//	 本程序调用外部.exe程序，无进度显示
	//
// 	interface orsIGuiExe : public orsIExecute
// 	{
// 	public:
// 		//计算得到输出参数
// 		virtual ref_ptr<IProperty> execute() = 0;
// 
// 		ORS_INTERFACE_DEF(orsIExecute, _T("gui"));
// 	};


	//
	// 分布式并行处理程序
	//
	interface IParallelExe : public IExecutor
	{
	public:
		// 如果 nTasks = 0, 算法自己决定任务数，否则按照给定的任务数运行
		virtual iplArray<ref_ptr<IProperty> > getTasks(int nTasks = 0) = 0;

		//通过Job信息和Task信息进行计算得到输出信息,taskOutput在外部分配内存<map>
		virtual ref_ptr<IProperty> taskExecute(ref_ptr<IProperty> taskInput, IProgressMsg *process) = 0;

		//进行子任务合并<reduce>
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

		// 内部参数
		virtual ref_ptr<IProperty> getParameterArgs()
		{
			return m_parameterArgs;
		}

		// 输出文件
		virtual ref_ptr<IProperty> getOutputFileNames()
		{
			return m_outputFileNames;
		}

		virtual const ref_ptr<IProperty> getProperty() const
		{
			return m_jobArguments;
		}

		//输入参数信息
		virtual bool initFromProperty(IProperty *property)
		{
			ref_ptr<IProperty> inputFileNames;
			ref_ptr<IProperty> parameterArgs;
			ref_ptr<IProperty> outputFileNames;

			property->getAttr(INPUT_FILE_NAMES, inputFileNames);
			property->getAttr(PARAMETER_ARGS, parameterArgs);
			property->getAttr(OUTPUT_FILE_NAMES, outputFileNames);

			// 兼容旧的调用方式
			if (NULL == inputFileNames.get())
				inputFileNames = property;
			if (NULL == parameterArgs.get())
				parameterArgs = property;
			if (NULL == outputFileNames.get())
				outputFileNames = property;

			// 自动更新内部属性
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


	// 并行进程数
#define IPL_PARALLEL_NUM_PROCESSES	_T("Num Of Processes")
#define IPL_MPI_CMD		_T("MPI_Command")

#define IPL_WebSERVICE	_T("Web Service")

}
