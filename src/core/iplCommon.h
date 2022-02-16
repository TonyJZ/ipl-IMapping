#pragma once

#include "core/ipldef.h"
#include "core/iplstd.h"

//#include "orsBase/iplString.h"

// #if defined(_UNICODE) || defined(UNICODE)
// typedef std::wstring		stdString;
// #else
// typedef std::string			stdString;
// #endif

namespace ipl
{
	// Encoding and decoding Base64 code
	class iplBase64
	{
	public:

		// Encodes binary data to Base64 code
		// Returns size of encoded data.
		static int encode(const unsigned char* inData,
			int dataLength,
			iplString & outCode);

		// Decodes Base64 code to binary data
		// Returns size of decoded data.
		static int decode(const iplString &inCode,
			int codeLength,
			unsigned char* outData);

		// Returns maximum size of decoded data based on size of Base64 code.
		static int getDataLength(int codeLength);

		// Returns maximum length of Base64 code based on size of uncoded data.
		static int getCodeLength(int dataLength);

	};

	bool findAllFiles(const iplChar * dir, const iplChar * extend, iplArray <iplString> &fileList);

//	iplString  ToStdString(const iplString &str);

#ifdef _WIN32
#define PLUGIN_EXTENSION	"*.dll;*.lnk"
#else
#define PLUGIN_EXTENSION	"*.so"
#endif

	typedef void*	iplPluginHandle;
	typedef void*	iplSymbolHandle;

	iplPluginHandle loadDynamicLib(const iplChar *name);
	void			unloadDynamicLib(iplPluginHandle handle);
	iplSymbolHandle getDynamicSymbol(iplPluginHandle handle, const iplChar * name);

//	iplString  getExeDir();

// 取给点类型的无效值

	ipl_float64 iplGetDefaultNULL(iplDataTYPE type);

	// 取类型的的范围
	ipl_float64 iplGetDefaultMin(iplDataTYPE type);
	ipl_float64 iplGetDefaultMax(iplDataTYPE type);

	//辅助方法，得到类型的字节数sizeof
	size_t iplGetSizeOfType(iplDataTYPE type);
}
