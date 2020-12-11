#pragma once
#include <vtkOBBTree.h>
#include <vtkMatrix4x4.h>
#include <vtkPolyData.h>

#include "vtkBox.h"
#include "vtkCellArray.h"
#include "vtkCellData.h"
#include "vtkCommand.h"
#include "vtkIdList.h"
#include "vtkIdTypeArray.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkLine.h"
#include "vtkLookupTable.h"
#include "vtkMath.h"
#include "vtkMatrix4x4.h"
#include "vtkMatrixToLinearTransform.h"
#include "vtkOBBTree.h"
#include "vtkObjectFactory.h"
#include "vtkPlane.h"
#include "vtkPointData.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkPolygon.h"
#include "vtkSmartPointer.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkTransform.h"
#include "vtkTriangle.h"
#include "vtkTrivialProducer.h"
#include "vtkUnsignedCharArray.h"
#include <thread>

#include <vtkCollisionDetectionFilter.h>

#define COLLISETMacro(name , type) void set##name(type _arge) {this->name = _arge;}
#define COLLIGETMacro(name , type) type get##name() {return this->name;}

class collisionDetect {
public:
	enum CollisionModes
	{
		VTK_ALL_CONTACTS = 0,
		VTK_FIRST_CONTACT = 1,
		VTK_HALF_CONTACTS = 2
	};

	//��ײ����
	collisionDetect();
	~collisionDetect();

	//�ж��Ƿ���ײ
	bool isCollision(vtkPolyData* unaffectedSidePoly, vtkMatrix4x4* unaffectedSideMatrix,
		vtkPolyData* affectedSidePoly, vtkMatrix4x4* affectedSideMatrix);
	bool isCollision();

	//�ҵ���ײ����
	int GetNumberOfContacts();

	//����ν�����
	int IntersectPolygonWithPolygon(int npts, double* pts, double bounds[6], int npts2, double* pts2,
		double bounds2[6], double tol2, double x1[2], double x2[3], int CollisionMode);

	//��ʼ����ײ��
	void initCollisionTree(vtkPolyData* unaffectedSidePoly, vtkPolyData* affectedSidePoly);

	//�����ײ���ģʽ
	int GetCollisionMode();


	//SET&GET
	COLLISETMacro(affectedSideMatrix, vtkMatrix4x4*);
	COLLIGETMacro(affectedSideMatrix, vtkMatrix4x4*);

	COLLISETMacro(unaffectedSideMatrix, vtkMatrix4x4*);
	COLLIGETMacro(unaffectedSideMatrix, vtkMatrix4x4*);

	COLLISETMacro(affectedSideContactCells, vtkIdTypeArray*);
	COLLIGETMacro(affectedSideContactCells, vtkIdTypeArray*);

	COLLISETMacro(unaffectedSideContactCells, vtkIdTypeArray*);
	COLLIGETMacro(unaffectedSideContactCells, vtkIdTypeArray*);

	COLLISETMacro(affectedSidePoly, vtkPolyData*);
	COLLIGETMacro(affectedSidePoly, vtkPolyData*);

	COLLISETMacro(unaffectedSidePoly, vtkPolyData*);
	COLLIGETMacro(unaffectedSidePoly, vtkPolyData*);

	COLLISETMacro(ContactPolydata, vtkPolyData*);
	COLLIGETMacro(ContactPolydata, vtkPolyData*);

	COLLISETMacro(unaffectedSideTree, vtkOBBTree*);
	COLLIGETMacro(unaffectedSideTree, vtkOBBTree*);

	COLLISETMacro(affectedSideTree, vtkOBBTree*);
	COLLIGETMacro(affectedSideTree, vtkOBBTree*);

	COLLISETMacro(CellTolerance, int);
	COLLIGETMacro(CellTolerance, int);

	COLLISETMacro(BoxTolerance, int);
	COLLIGETMacro(BoxTolerance, int);

	COLLISETMacro(affectedSideTreeMaxLevel, int);
	COLLIGETMacro(affectedSideTreeMaxLevel, int);

	COLLISETMacro(unAffectedSideTreeMaxLevel, int);
	COLLIGETMacro(unAffectedSideTreeMaxLevel, int);

	bool isReady;

private:

	//������ײ����������
	int affectedSideTreeMaxLevel;
	int unAffectedSideTreeMaxLevel;

	//������ײ��
	vtkOBBTree* unaffectedSideTree;
	vtkOBBTree* affectedSideTree;

	//���յİ�Χ�������ٴ��ڵļ���cell
	int NumberOfCellsPerNode;

	//��Χ�е��ݲ�
	double BoxTolerance;

	//���������
	vtkPolyData* affectedSidePoly;
	vtkPolyData* unaffectedSidePoly;
	vtkPolyData* ContactPolydata;

	//��ײģʽ
	int collisionMode;

	//����ͽ������ײcell
	vtkIdTypeArray* affectedSideContactCells;
	vtkIdTypeArray* unaffectedSideContactCells;

	//����ͽ���ı任����
	vtkMatrix4x4* affectedSideMatrix;
	vtkMatrix4x4* unaffectedSideMatrix;


	//��ײcell���ݲ�
	int CellTolerance;


	std::thread* initThread;
};