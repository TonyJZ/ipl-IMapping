/*
* Software License Agreement (Proprietary License)
*
*
*  Copyright (c) 2017-, Appropolis, Inc.
*
*  All rights reserved.
*
*  rights described below...
*
*/

#include "core/iplcore.h"

using namespace ipl;

iplException::iplException() { code = 0; line = 0; }

iplException::iplException(int _code, const std::string& _err, const std::string& _func, const std::string& _file, int _line)
	: code(_code), err(_err), func(_func), file(_file), line(_line)
{
	formatMessage();
}

iplException::~iplException() throw() {}

/*!
\return the error description and the context as a text string.
*/
const char* iplException::what() const throw() { return msg.c_str(); }

void iplException::formatMessage()
{
	// 	if (func.size() > 0)
	// 		msg = format("%s:%d: error: (%d) %s in function %s\n", file.c_str(), line, code, err.c_str(), func.c_str());
	// 	else
	// 		msg = format("%s:%d: error: (%d) %s\n", file.c_str(), line, code, err.c_str());
}


