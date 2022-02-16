#include "commonAPIs/iplutility.h"


#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace boost::filesystem;
using namespace boost::algorithm;
using namespace ipl;



bool ipl::create_folder(const std::string str_folderpath)
{
	string standard_folder = replace_all_copy(str_folderpath, "\\", "/");
	path lPtest(standard_folder);

	if (!exists(lPtest))
	{
		return create_directories(lPtest);
	}

	return true;
}

bool ipl::delete_folder(const std::string dir)
{
	boost::filesystem::path p(dir);

	if (!exists(p))
		return false;

	if (!is_directory(p))
		return false;

	remove_all(p);
	return true;
}

bool ipl::empty_folder(const std::string dir)
{
	boost::filesystem::path p(dir);

	if (!boost::filesystem::is_directory(p))
	{
		return false;
	}

	directory_iterator it(p);
	directory_iterator itEnd;

	while (it != itEnd)
	{
		boost::system::error_code  ec;
		path  rPath = it->path();

		++it;
		remove_all(rPath, ec);
	}
	return true;
}

bool ipl::copy_folder(const std::string srcDir, const std::string dstDir)
{
	boost::filesystem::path src_path(srcDir);
	boost::filesystem::path dst_path(dstDir);

	if (!exists(src_path))
		return false;

	if (!exists(dst_path))
	{
		create_folder(dstDir);
	}

	for (boost::filesystem::directory_iterator it(src_path); it != boost::filesystem::directory_iterator(); ++it)
	{
		const boost::filesystem::path newSrc = it->path();
		const boost::filesystem::path newDst = dst_path / it->path().filename();
		if (boost::filesystem::is_directory(newSrc))
		{
			copy_folder(newSrc.string(), newDst.string());
		}
		else if (boost::filesystem::is_regular_file(newSrc))
		{
			boost::filesystem::copy_file(newSrc, newDst, boost::filesystem::copy_option::overwrite_if_exists);
		}
		else
		{
			fprintf(stderr, "Error: unrecognized file - %s", newSrc.string().c_str());
		}
	}

	return true;
}

int ipl::scan_files(const std::string str_path, const std::string str_extname, std::vector <std::string> &filenames)
{
	path fullpath(str_path);
	filenames.clear();

	if (!exists(fullpath))
	{
		return (0);
	}

	if (!is_directory(fullpath))        // 是否是目录
	{
		return (0);
	}

	directory_iterator end_iter;
	for (directory_iterator file_itr(fullpath); file_itr != end_iter; ++file_itr)
	{
		if (!is_directory(*file_itr) && (extension(*file_itr) == str_extname))        // 文件后缀
		{
			filenames.push_back(file_itr->path().string());    //获取文件名
		}
	}

	return filenames.size();
}

int ipl::scan_folders(const std::string str_path, std::vector <std::string> &subfolders)
{
	subfolders.clear();
	if (is_directory(str_path))
	{
		directory_iterator tmp_directory_end;
		directory_iterator tmp_dir_it(str_path);

		for (tmp_dir_it; tmp_dir_it != tmp_directory_end; tmp_dir_it++)
		{
			//path child_dest_path = dest_path;
			if (is_directory(*tmp_dir_it))
			{
				string parent_path = (*tmp_dir_it).path().parent_path().string();
				parent_path += '/';
				string tmp_dir_name = (*tmp_dir_it).path().filename().string();
				string tmp_dir_stem = (*tmp_dir_it).path().stem().string();
				tmp_dir_name = parent_path + tmp_dir_name;
				subfolders.push_back(tmp_dir_name);
			}
		}
	}

	return subfolders.size();
}

bool ipl::check_exist(const std::string fullpath)
{
	boost::filesystem::path p(fullpath);

	if (boost::filesystem::exists(p))  //判断文件存在性    
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool ipl::copy_file(const std::string orgfilePath, const std::string trgfilePath, bool bOverwrite)
{
	if (!check_exist(orgfilePath))
		return false;

	boost::filesystem::path org_path(orgfilePath);
	boost::filesystem::path trg_path(trgfilePath);

	if (!exists(trg_path.parent_path()))
		create_directories(trg_path.parent_path());

	if (bOverwrite)
	{
		boost::filesystem::copy_file(org_path, trg_path, copy_option::overwrite_if_exists);
	}
	else
	{
		boost::filesystem::copy_file(org_path, trg_path, copy_option::fail_if_exists);
	}
	return true;
}

bool ipl::delete_file(const std::string filePath)
{
	boost::filesystem::path file(filePath);

	if (boost::filesystem::exists(file))
		return boost::filesystem::remove(file);
	else
		return false;
}

bool ipl::get_directory_exe( std::string &dir )
{
	
	std::string exdir = boost::filesystem::initial_path<boost::filesystem::path>().string();
	
	dir = replace_all_copy(exdir, "\\", "/");
	return true;
}

bool ipl::get_directory_etc( std::string &dirPath )
{
	boost::filesystem::path exe_path = boost::filesystem::initial_path<boost::filesystem::path>();
	
	boost::filesystem::path parent_path = exe_path.parent_path();

	boost::filesystem::path etc_path = parent_path / "etc"; //path支持重载/运算符  

	if (boost::filesystem::exists(etc_path))  //判断文件存在性    
	{
		dirPath = replace_all_copy(etc_path.string(), "\\", "/"); 
		return true;
	}
	else
	{
		return false;
	}
}



// bool ipl::shell_execute( const iplChar *cmdLine, bool bWaite)
// {
// #ifdef ipl_PLATFORM_WINDOWS
// 
// 	if( !bWaite ) {
// 		if( WinExec( cmdLine, SW_SHOW ) > 31 )
// 			return true;
// 		return false;
// 	}
// 	
// 	STARTUPINFO si; //一些必备参数设置
// 	memset(&si, 0, sizeof(STARTUPINFO));
// 	si.cb = sizeof(STARTUPINFO);
// 	si.dwFlags = STARTF_USESHOWWINDOW;
// 	si.wShowWindow = SW_SHOW;
// 	PROCESS_INFORMATION pi; //必备参数设置结束
// 	
// 	if( CreateProcess( NULL, (iplChar *)cmdLine, NULL,NULL,FALSE,0,NULL,NULL, &si, &pi) )
// 	{
// 		WaitForSingleObject( pi.hProcess, INFINITE );
// 		
// 		return true;
// 	}
// 
// #endif
// 	
// 	return false;
// }


