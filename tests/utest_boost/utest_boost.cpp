// utest_boost.cpp : Defines the entry point for the console application.
//

#include <boost/filesystem.hpp>
#include <boost/algorithm/string_regex.hpp>
#include <iostream>

using namespace std;
using namespace boost::filesystem;
using namespace boost::algorithm;

bool create_folder(const std::string str_folderpath)
{
	string standard_folder = replace_all_copy(str_folderpath, "\\", "/");
	path lPtest(standard_folder);

	try
	{
		if (!exists(lPtest))
		{
			create_directories(lPtest);
		}
	}
	catch (boost::filesystem::filesystem_error e)
	{
		cout << e.what() << endl;
	}

	



	return true;
}

bool delete_folder(const std::string dir)
{
	boost::filesystem::path p(dir);

	if (!exists(p))
		return false;

	if (!is_directory(p))
		return false;

	remove_all(p);
	return true;
}

bool empty_folder(const std::string dir)
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

bool copy_folder(const std::string srcDir, const std::string dstDir)
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

int scan_files(const std::string str_path, const std::string str_extname, std::vector <std::string> &filenames)
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

int scan_folders(const std::string str_path, std::vector <std::string> &subfolders)
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

bool check_exist(const std::string fullpath)
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

bool copy_file(const std::string orgfilePath, const std::string trgfilePath, bool bOverwrite)
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

bool delete_file(const std::string filePath)
{
	boost::filesystem::path file(filePath);

	if (boost::filesystem::exists(file))
		return boost::filesystem::remove(file);
	else
		return false;
}

bool get_directory_exe(std::string &dir)
{

	std::string exdir = boost::filesystem::initial_path<boost::filesystem::path>().string();

	dir = replace_all_copy(exdir, "\\", "/");
	return true;
}

bool get_directory_etc(std::string &dirPath)
{
	boost::filesystem::path exe_path = boost::filesystem::initial_path<boost::filesystem::path>();

	boost::filesystem::path parent_path = exe_path.parent_path();

	printf("parent_path: %s\n", parent_path.string().c_str());

	boost::filesystem::path etc_path = parent_path / "etc"; //path支持重载/运算符  

	printf("etc_path: %s\n", etc_path.string().c_str());
	if (boost::filesystem::exists(etc_path))  //判断文件存在性    
	{
		printf("here\n");
		dirPath = replace_all_copy(etc_path.string(), "\\", "/");
		printf("dirPath: %s\n", dirPath.c_str());
		return true;
	}
	else
	{
		printf("false\n");
		return false;
	}
}


int main()
{
	printf("begin\n");
	std::string exedir;
	get_directory_exe(exedir);
	printf("exedir: %s\n", exedir.c_str());

	std::string newfolder = exedir + "/newfolder";
	create_folder(newfolder);

	std::vector <std::string> filenames;
	scan_files(exedir, ".so", filenames);

    return 0;
}

