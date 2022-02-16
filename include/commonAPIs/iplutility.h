#pragma once

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

#include <core/ipldef.h>
#include <core/iplstd.h>

namespace ipl
{
	
	IPL_BASE_API bool get_directory_exe(std::string &dir);
	
	IPL_BASE_API bool get_directory_etc(std::string &dir);

	/** \brief create a folder
	* \param[in] str_folder the folder name with full path
	* \param[out] true/false create succeed or not
	*/
	IPL_BASE_API bool  create_folder(const std::string str_folderpath);

	/** \brief recusively copy files or directory from $srcDir to $dstDir
	* \param[in] srcDir the original folder
	* \param[in] dstDir the target folder path
	* \param[out] true/false copy succeed or not
	*/
	IPL_BASE_API bool copy_folder(const std::string srcDir, const std::string dstDir);

	/** \brief delete a folder even if the folder is not empty
	* \param[in] dir the folder name with full path
	* \param[out] true/false delete succeed or not
	*/
	IPL_BASE_API bool delete_folder(const std::string dir);

	/** \brief empty a folder 
	* \param[in] dir the folder name with full path
	* \param[out] true/false succeed or not
	*/
	IPL_BASE_API bool empty_folder(const std::string dir);

	/** \brief search files with the giving extension in a giving path.  case sensitive
	* \param[in] str_folder the folder name with full path
	* \param[in] str_extname the extension of files like ".jpg"
	* \param[out] filenames  the found files
	*/
	IPL_BASE_API int  scan_files(const std::string str_path, const std::string str_extname, std::vector <std::string> &filenames);


	/** \brief scan subfolders in a giving path, do not recursively search
	* \param[in] str_path the folder name with full path
	* \param[out] subfolders the searched subfolders in the given path
	*
	*/
	IPL_BASE_API int scan_folders(const std::string str_path, std::vector <std::string> &subfolders);

	/** \brief check the giving path exists or not
	* \param[in] fullpath the folder name or file name
	* \param[out] true/false the given path is existing or not
	*/
	IPL_BASE_API bool check_exist(const std::string fullpath);

	/** \brief copy file to a giving path
	* \param[in] orgfilePath the original file
	* \param[in] tarfilePath the target file path
	* \param[in] bOverwrite the overwrite flag
	* \param[out] true/false copy succeed or not
	*/
	IPL_BASE_API bool copy_file(const std::string orgfilePath, const std::string trgfilePath, bool bOverwrite = true);

	/** \brief delete a file
	* \param[in] filePath the giving file
	* \param[out] true/false delete succeed or not
	*/
	IPL_BASE_API bool delete_file(const std::string filePath);




	// 	bool EmptyFolder(const orsChar *filePath);
	// 
	// 	bool MoveFile(const orsChar *filePath, const orsChar *newFilePath, bool bFailIfExists);
	// 	bool MoveFolder(const orsChar *srcDir, const orsChar *dstDir, bool bFailIfExists);

}

