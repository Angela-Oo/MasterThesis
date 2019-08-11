#pragma once
#include <string>

class FileWriter {
	std::string _file_path;
public:
	void write(std::string text);
public:
	FileWriter(std::string file_path);
};
