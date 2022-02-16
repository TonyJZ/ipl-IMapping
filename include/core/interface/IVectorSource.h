#pragma once

#include "core/ipldef.h"
#include "core/iplstd.h"
#include "core/iplmacro.h"
#include "core/interface/IDataSource.h"

//Tony 2018-01-27  为其他矢量格式预留的扩展点
namespace ipl
{
	interface IVectorSource : public IDataSource
	{
	public:

		//		virtual std::string	getDesc() = 0;

		IPL_INTERFACE_DEF(IDataSource, _T("VectorSource"));
	};


	interface IVectorSourceReader : public IObject
	{
	public:
		virtual iplFileFormatList GetSupportedFormats() = 0;
		virtual const iplChar *getFilePath() const = 0;
		//		virtual std::string	getDesc() = 0;

		IPL_INTERFACE_DEF(IObject, _T("VectorSourceReader"));
	};

	interface IVectorSourceWriter : public IObject
	{
	public:
		virtual iplFileFormatList GetSupportedFormats() = 0;
		virtual const iplChar *getFilePath() const = 0;
		//		virtual std::string	getDesc() = 0;

		IPL_INTERFACE_DEF(IObject, _T("VectorSourceWriter"));
	};

#define IPL_VECTORSOURCE_SF_READER			_T("ipl.VectorSourceReader.SF.OGR")
#define IPL_VECTORSOURCE_SF_WRITER			_T("ipl.VectorSourceWriter.SF.OGR")

}
