// utest_commonAPIs.cpp : Defines the entry point for the console application.
//

#include "commonAPIs/iplstring.h"
//#include "commonAPIs/iplfiles.h"
#include "commonAPIs/iplutility.h"

int main(int argc, char * argv[])
{
	std::string str0, str1;

	str0 = "1AdoG8!";

	ipl::convert2lower(str0, str1);
	ipl::convert2upper(str0, str1);

	std::string exeDir, etcDir;

	// 	std::cout << boost::filesystem::current_path() << std::endl;
	// 	std::cout << boost::filesystem::initial_path() << std::endl;

	// 	getchar();
	// 	return 0;

	ipl::get_directory_exe(exeDir);
	ipl::get_directory_etc(etcDir);

	std::string tempDir = "D:/iplTestData/utest/test_iplcore/1";
	ipl::create_folder(tempDir);
	ipl::copy_folder(etcDir, tempDir);
	ipl::delete_folder(tempDir);

	ipl::copy_file("D:/temp/3Dpolygon_out.kml", "D:/iplTestData/utest/test_iplcore/1/3Dpolygon_out.kml");
	ipl::delete_file("D:/iplTestData/utest/test_iplcore/1/3Dpolygon_out.kml");

	std::vector <std::string> filenames;

	ipl::scan_folders("D:/data_tracking_benchmark", filenames);
	ipl::scan_files("D:/data_tracking_benchmark/3DMOT2015", ".mp4", filenames);

	std::string stem, ext;
	ipl::extract_file_name(filenames[0], stem, ext);
	ipl::extract_pure_file_name(filenames[0], stem, ext);

	std::string path;
	path = "D:\\zlib\\";
	std::string dir;
	ipl::get_dir_from_path(path, dir);

	ipl::empty_folder("D:/iplTestData/utest/test_iplCommonAPIs");

	return 0;
}

