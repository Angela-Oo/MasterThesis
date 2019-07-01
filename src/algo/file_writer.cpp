#include "stdafx.h"
#include "file_writer.h"
#include <experimental/filesystem>

void FileWriter::write(std::string text)
{
	std::ofstream file;
	file.open(_file_path, std::ios_base::app);
	file << text;
	file.close();
}

FileWriter::FileWriter(std::string file_path)
	: _file_path(file_path)
{
	std::experimental::filesystem::path dir(file_path);
	try {
		std::experimental::filesystem::create_directories(dir.parent_path());
	}
	catch (std::exception e)
	{
		std::cout << file_path << e.what() << std::endl;
	}

	std::ofstream file;
	file.open(_file_path, std::fstream::out);
	file << std::endl;
	file.close();
}

