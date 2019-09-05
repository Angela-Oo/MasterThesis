#include "file_writer.h"
#include <experimental/filesystem>
#include <fstream>
#include <iostream>

void FileWriter::write(std::string text)
{	
	try {
		std::ofstream file;
		file.open(_file_path, std::ios_base::app);
		file << text;
		file.close();
		std::cout << text;
	}
	catch (...) {
		std::cout << "could not write to file" << std::endl;
	}
}

FileWriter::FileWriter(std::string file_path)
	: _file_path(file_path)
{
	std::experimental::filesystem::path dir(file_path);
	try {
		std::experimental::filesystem::create_directories(dir.parent_path());
		std::ofstream file;
		file.open(_file_path, std::fstream::out);
		file << std::endl;
		file.close();
	}
	catch (std::exception e)
	{
		std::cout << file_path << e.what() << std::endl;
	}

}

