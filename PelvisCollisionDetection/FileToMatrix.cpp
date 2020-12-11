#include "FileToMatrix.h"
FileToMatrix::FileToMatrix(string s_FileAddress) {
	this->m_FileAddress = s_FileAddress;
}

//读取txt，行主列，文本为列主列，需要转置。
vtkSmartPointer<vtkMatrix4x4> FileToMatrix::GetTransformMatrix() {
	std::fstream in(m_FileAddress);
	if (in.is_open()) {
		string str;
		double TransformArray[16] = { 0 };
		for (int i = 0; i < 16; i++) {
			if ((std::getline(in, str))) {
				TransformArray[i] = atof(str.c_str());
			}
		}
		m_TransformMatric = vtkSmartPointer<vtkMatrix4x4>::New();
		m_TransformMatric->DeepCopy(TransformArray);
		m_TransformMatric->Transpose();
		m_TransformMatric->GetData();
		in.close();
	}
	return m_TransformMatric;
	
}