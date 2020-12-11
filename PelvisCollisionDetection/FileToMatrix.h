#pragma once
#include <iostream>
#include <string>
#include <fstream>
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>
#include <array>
using std::string;
class FileToMatrix {
public:
	FileToMatrix(string s_FileAddress);
	vtkSmartPointer<vtkMatrix4x4> GetTransformMatrix();
private:
	string m_FileAddress;
	vtkSmartPointer<vtkMatrix4x4> m_TransformMatric;
};