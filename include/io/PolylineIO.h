#pragma once
//this is a simply polygon 


#include <core/iplcore.h>
#include <fstream>

namespace ipl
{
	struct simpleLineSeg
	{
		double x[2];
		double y[2];
	};

	bool write_2Dpolylines_TXT(const std::string filename, const std::vector <simpleLineSeg> lineSegs)
	{
		std::ofstream  ofs;
		ofs.open(filename);

		if (!ofs.is_open())
		{
			std::cout << "error: can't open " << filename << std::endl;
			return (false);
		}

		for (int i = 0; i < lineSegs.size(); i++)
		{
			ofs << lineSegs[i].x[0] << ' ' << lineSegs[i].y[0] << ' '
				<< lineSegs[i].x[1] << ' ' << lineSegs[i].y[1] << std::endl;
		}
		
		ofs.close();
		return true;
	}
}

