// test_openmp.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"

#include <iostream>
#include <vector>
#include <omp.h>

#include <boost/unordered_map.hpp>

#include <boost/shared_ptr.hpp>

class implementation
{
public:
	~implementation() { std::cout << "destroying implementation\n"; }
	
	boost::shared_ptr<std::string> do_something()
	{
		boost::shared_ptr<std::string> name(new std::string("this is implementation test!"));

		return name;
	}
};

boost::shared_ptr<std::string> do_something()
{
	boost::shared_ptr<std::string> name(new std::string("this is function test!"));

	return name;
}

void test_smart_ptr()
{
	//≤‚ ‘÷«ƒ‹÷∏’Î
	implementation imp1;

	boost::shared_ptr<std::string> sp1 = imp1.do_something();

	std::cout << *sp1 << std::endl;

	boost::shared_ptr<std::string> sp2 = do_something();

	std::cout << *sp2 << std::endl;
}

void test_openMP()
{
	boost::unordered_map<int, float> face_att_list_;
	std::vector<bool> flag;
	for (int i = 0; i < 100; i++)
		face_att_list_.insert(std::make_pair(i, 0.0));

	flag.resize(100, false);
	boost::unordered_map<int, float>::const_iterator iter;

#pragma omp parallel private(iter)
	{
		for (iter = face_att_list_.begin(); iter != face_att_list_.end(); ++iter)
		{
#pragma omp single nowait 
			{
				std::cout << iter->first << std::endl;

				if (flag.at(iter->first))
				{
					assert(false);
				}
				else
				{
					flag.at(iter->first) = true;
				}
			}
		}
	}
}

int main()
{
	test_smart_ptr();
	test_openMP();

    return 0;
}

