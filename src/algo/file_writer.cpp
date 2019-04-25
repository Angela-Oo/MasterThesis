#include "stdafx.h"
#include "file_writer.h"

void FileWriter::write(std::string text)
{
	std::ofstream file;
	file.open(_file_path, std::ios_base::app);
	file << text;
}

FileWriter::FileWriter(std::string file_path)
	: _file_path(file_path)
{
	std::ofstream file;
	file.open(_file_path, std::fstream::out);
	file << std::endl;
}

