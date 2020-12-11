#include "collisionDetect.h"

collisionDetect::collisionDetect() {
	this->affectedSideTree = vtkOBBTree::New();
	this->unaffectedSideTree = vtkOBBTree::New();
	NumberOfCellsPerNode = 2;
	BoxTolerance = 0.0;
	CellTolerance = 0.0;

	affectedSidePoly = nullptr;
	unaffectedSidePoly = nullptr;
	ContactPolydata = nullptr;

	//碰撞模式
	collisionMode = 2;

	affectedSideContactCells = vtkIdTypeArray::New();
	unaffectedSideContactCells = vtkIdTypeArray::New();

	affectedSideMatrix = nullptr;
	unaffectedSideMatrix = nullptr;

	ContactPolydata = vtkPolyData::New();

	CellTolerance = 0;

	isReady = false;

	affectedSideTreeMaxLevel = 20;
	unAffectedSideTreeMaxLevel = 20;
}

collisionDetect::~collisionDetect() {

	unaffectedSideTree->Delete();
	affectedSideTree->Delete();

	affectedSideContactCells->Delete();
	unaffectedSideContactCells->Delete();
}

int collisionDetect::IntersectPolygonWithPolygon(int npts, double * pts, double bounds[6], int npts2, double * pts2, double bounds2[6], double tol2, double x1[2], double x2[3], int CollisionMode)
{
	double n[3], n2[3], coords[3];
	int i, j;
	double *p1, *p2, *q1, ray[3], ray2[3];
	double t, u, v;
	double* x[2];
	int Num = 0;
	x[0] = x1;
	x[1] = x2;
	
	//  Intersect each edge of first polygon against second
	//
	vtkPolygon::ComputeNormal(npts2, pts2, n2);
	vtkPolygon::ComputeNormal(npts, pts, n);
	
	int parallel_edges = 0;
	for (i = 0; i < npts; i++)
	{
		p1 = pts + 3 * i;
		p2 = pts + 3 * ((i + 1) % npts);
	
		for (j = 0; j < 3; j++)
		{
			ray[j] = p2[j] - p1[j];
		}
		if (!vtkBox::IntersectBox(bounds2, p1, ray, coords, t))
		{
			continue;
		}
	
		if ((vtkPlane::IntersectWithLine(p1, p2, n2, pts2, t, x[Num])) == 1)
		{
			if ((npts2 == 3 && vtkTriangle::PointInTriangle(x[Num], pts2, pts2 + 3, pts2 + 6, tol2)) ||
				(npts2 > 3 && vtkPolygon::PointInPolygon(x[Num], npts2, pts2, bounds2, n2) == 1))
			{
				Num++;
				if (collisionMode != collisionDetect::VTK_ALL_CONTACTS || Num == 2)
				{
					return 1;
				}
			}
		}
		else
		{
			// cout << "Test for overlapping" << endl;
			// test to see if cells are coplanar and overlapping...
			parallel_edges++;
			if (parallel_edges > 1) // cells are parallel then...
			{
				// cout << "cells are parallel" << endl;
				// test to see if they are coplanar
				q1 = pts2;
				for (j = 0; j < 3; j++)
				{
					ray2[j] = p1[j] - q1[j];
				}
				if (vtkMath::Dot(n, ray2) == 0.0) // cells are coplanar
				{
					// cout << "cells are coplanar" << endl;
					// test to see if coplanar cells overlap
					// ie, if one of the tris has a vertex in the other
					for (int ii = 0; ii < npts; ii++)
					{
						for (int jj = 0; jj < npts2; jj++)
						{
							if (vtkLine::Intersection(pts + 3 * ii, pts + 3 * ((ii + 1) % npts), pts2 + 3 * jj,
								pts2 + 3 * ((jj + 1) % npts2), u, v) == 2)
							{
								// cout << "Found an overlapping one!!!" << endl;
								for (int k = 0; k < 3; k++)
								{
									x[Num][k] =
										pts[k + 3 * ii] + u * (pts[k + (3 * ((ii + 1) % npts))] - pts[k + 3 * ii]);
								}
								Num++;
								if (collisionMode != collisionDetect::VTK_ALL_CONTACTS || Num == 2)
								{
									return 1;
								}
							}
						}
					}
	
				} // end if cells are coplanar
			}   // end if cells are parallel
		}     // end else
}

//  Intersect each edge of second polygon against first
//

for (i = 0; i < npts2; i++)
{
	p1 = pts2 + 3 * i;
	p2 = pts2 + 3 * ((i + 1) % npts2);

	for (j = 0; j < 3; j++)
	{
		ray[j] = p2[j] - p1[j];
	}

	if (!vtkBox::IntersectBox(bounds, p1, ray, coords, t))
	{
		continue;
	}

	if ((vtkPlane::IntersectWithLine(p1, p2, n, pts, t, x[Num])) == 1)
	{
		if ((npts == 3 && vtkTriangle::PointInTriangle(x[Num], pts, pts + 3, pts + 6, tol2)) ||
			(npts > 3 && vtkPolygon::PointInPolygon(x[Num], npts, pts, bounds, n) == 1))
		{
			Num++;
			if (collisionMode != collisionDetect::VTK_ALL_CONTACTS || Num == 2)
			{
				return 1;
			}
		}
	}

}
return 0;
}

 static void  init(collisionDetect* self ,vtkPolyData * unaffectedSidePoly, vtkPolyData * affectedSidePoly) {
	 //更新树并构建树，如果数据集没变换那么久不会更新
	 vtkOBBTree* unaffectedSideTree;
	 vtkOBBTree* affectedSideTree;
	 unaffectedSideTree = self->getunaffectedSideTree();
	 affectedSideTree = self->getaffectedSideTree();

	 unaffectedSideTree->SetDataSet(unaffectedSidePoly);
	 unaffectedSideTree->AutomaticOn();
	 unaffectedSideTree->SetNumberOfCellsPerNode(2);
	 unaffectedSideTree->SetMaxLevel(self->getunAffectedSideTreeMaxLevel());
	 unaffectedSideTree->BuildLocator();
	 int level = unaffectedSideTree->GetLevel();

	 affectedSideTree->SetDataSet(affectedSidePoly);
	 affectedSideTree->AutomaticOn();
	 affectedSideTree->SetNumberOfCellsPerNode(2);
	 unaffectedSideTree->SetMaxLevel(self->getaffectedSideTreeMaxLevel());
	 affectedSideTree->BuildLocator();

	 //设置树的包围盒容差
	 unaffectedSideTree->SetTolerance(self->getBoxTolerance());
	 affectedSideTree->SetTolerance(self->getBoxTolerance());

	self->isReady = true;
}

void collisionDetect::initCollisionTree(vtkPolyData * unaffectedSidePoly, vtkPolyData * affectedSidePoly)
{
	initThread = new std::thread(init,this, unaffectedSidePoly, affectedSidePoly);
	initThread->detach();
}
int collisionDetect::GetCollisionMode() {
	return collisionMode;
}
static int ComputeCollisions(
	vtkOBBNode* nodeA, vtkOBBNode* nodeB, vtkMatrix4x4* Xform, void* clientdata)
{
	int numIdsA, numIdsB;
	vtkIdList *IdsA, *IdsB, *pointIdsA, *pointIdsB;
	vtkCellArray* cells;
	vtkIdType cellPtIds[2];
	vtkIdTypeArray *contactcells1, *contactcells2;
	vtkPoints* contactpoints;
	IdsA = nodeA->Cells;
	IdsB = nodeB->Cells;
	numIdsA = IdsA->GetNumberOfIds();
	numIdsB = IdsB->GetNumberOfIds();

	// clientdata is a pointer to this object... need to cast it as such
	collisionDetect* self = reinterpret_cast<collisionDetect*>(clientdata);

	int FirstContact = 0;

	vtkPolyData* unaffectedSidePoly = self->getunaffectedSidePoly();
	vtkPolyData* affectedSidePoly = self->getaffectedSidePoly();
	contactcells1 = self->getunaffectedSideContactCells();
	contactcells2 = self->getaffectedSideContactCells();
	contactpoints = self->getContactPolydata()->GetPoints();


	if (self->GetCollisionMode() == collisionDetect::VTK_ALL_CONTACTS)
	{
		cells = self->getContactPolydata()->GetLines();
	}
	else
	{
		cells = self->getContactPolydata()->GetVerts();
	}

	float Tolerance = self->getCellTolerance();
	
	if (self->GetCollisionMode() == collisionDetect::VTK_FIRST_CONTACT)
	{
		FirstContact = 1;
	}
	vtkIdType cellIdA, cellIdB;

	double x1[4], x2[4], xnew[4];
	double ptsA[9], ptsB[9];
	double boundsA[6], boundsB[6];
	vtkIdType i, j, k, m, n, p, v;
	double *point, in[4], out[4];

	// Loop thru the cells/points in IdsA
	for (i = 0; i < numIdsA; i++)
	{
		cellIdA = IdsA->GetId(i);
		pointIdsA = unaffectedSidePoly->GetCell(cellIdA)->GetPointIds();
		unaffectedSidePoly->GetCellBounds(cellIdA, boundsA);

		// Initialize ptsA
		// It might speed things up if ptsA and ptsB only had to be an
		// array of pointers to the cell vertices, rather than allocating here.
		for (j = 0; j < 3; j++)
		{
			for (k = 0; k < 3; k++)
			{
				ptsA[j * 3 + k] = unaffectedSidePoly->GetPoints()->GetPoint(pointIdsA->GetId(j))[k];
			}
		}

		// Loop thru each cell IdsB and test for collision
		for (m = 0; m < numIdsB; m++)
		{
			cellIdB = IdsB->GetId(m);
			pointIdsB = affectedSidePoly->GetCell(cellIdB)->GetPointIds();
			affectedSidePoly->GetCellBounds(cellIdB, boundsB);

			// Initialize ptsB
			for (n = 0; n < 3; n++)
			{
				point = affectedSidePoly->GetPoints()->GetPoint(pointIdsB->GetId(n));
				// transform the vertex
				in[0] = point[0];
				in[1] = point[1];
				in[2] = point[2];
				in[3] = 1.0;
				Xform->MultiplyPoint(in, out);
				out[0] = out[0] / out[3];
				out[1] = out[1] / out[3];
				out[2] = out[2] / out[3];
				for (p = 0; p < 3; p++)
				{
					ptsB[n * 3 + p] = out[p];
				}
			}
			// Calculate the bounds for the xformed cell
			boundsB[0] = boundsB[2] = boundsB[4] = VTK_DOUBLE_MAX;
			boundsB[1] = boundsB[3] = boundsB[5] = VTK_DOUBLE_MIN;
			for (v = 0; v < 9; v = v + 3)
			{
				if (ptsB[v] < boundsB[0])
					boundsB[0] = ptsB[v];
				if (ptsB[v] > boundsB[1])
					boundsB[1] = ptsB[v];
				if (ptsB[v + 1] < boundsB[2])
					boundsB[2] = ptsB[v + 1];
				if (ptsB[v + 1] > boundsB[3])
					boundsB[3] = ptsB[v + 1];
				if (ptsB[v + 2] < boundsB[4])
					boundsB[4] = ptsB[v + 2];
				if (ptsB[v + 2] > boundsB[5])
					boundsB[5] = ptsB[v + 2];
			}
			// Test for intersection
			if (self->IntersectPolygonWithPolygon(
				3, ptsA, boundsA, 3, ptsB, boundsB, Tolerance, x1, x2, self->GetCollisionMode()))
			{
				contactcells1->InsertNextValue(cellIdA);
				contactcells2->InsertNextValue(cellIdB);
				// transform x back to "world space"
				// could speed this up by testing for identity matrix
				// and skipping the next transform.
				x1[3] = x2[3] = 1.0;
				self->getunaffectedSideMatrix()->MultiplyPoint(x1, xnew);
				xnew[0] = xnew[0] / xnew[3];
				xnew[1] = xnew[1] / xnew[3];
				xnew[2] = xnew[2] / xnew[3];
				cellPtIds[0] = contactpoints->InsertNextPoint(xnew);
				if (self->GetCollisionMode() == collisionDetect::VTK_ALL_CONTACTS)
				{
					self->getunaffectedSideMatrix()->MultiplyPoint(x2, xnew);
					xnew[0] = xnew[0] / xnew[3];
					xnew[1] = xnew[1] / xnew[3];
					xnew[2] = xnew[2] / xnew[3];
					cellPtIds[1] = contactpoints->InsertNextPoint(xnew);
					// insert a new line
					cells->InsertNextCell(2, cellPtIds);
				}
				else
				{
					// insert a new vert
					cells->InsertNextCell(1, cellPtIds);
				}

				if (FirstContact)
				{
					// return the negative of the number of box tests to find first contact
					// this will call a halt to the proceedings
					
					//return (-1 - self->GetNumberOfBoxTests());
				}
			}
		}
	}
	return 1;
}

bool collisionDetect::isCollision(vtkPolyData* unaffectedSidePoly, vtkMatrix4x4* unaffectedSideMatrix,
	vtkPolyData* affectedSidePoly, vtkMatrix4x4* affectedSideMatrix)
{
	while (!isReady) {

	}
	this->unaffectedSidePoly = unaffectedSidePoly;
	this->unaffectedSideMatrix = unaffectedSideMatrix;
	this->affectedSidePoly = affectedSidePoly;
	this->affectedSideMatrix = affectedSideMatrix;

	vtkPoints* contactsPoints = vtkPoints::New();
	ContactPolydata->SetPoints(contactsPoints);
	contactsPoints->Delete();

	if (this->collisionMode == collisionDetect::VTK_ALL_CONTACTS)
	{ // then create a lines cell array
		vtkCellArray* lines = vtkCellArray::New();
		ContactPolydata->SetLines(lines);
		lines->Delete();
	}
	else
	{ // else create a verts cell array
		vtkCellArray* verts = vtkCellArray::New();
		ContactPolydata->SetVerts(verts);
		verts->Delete();
	}

	//更新树并构建树，如果数据集没变换那么久不会更新
	if (unaffectedSideTree->GetLevel() == 0 && affectedSideTree -> GetLevel() == 0) {

		unaffectedSideTree->SetDataSet(unaffectedSidePoly);
		unaffectedSideTree->AutomaticOn();
		unaffectedSideTree->SetNumberOfCellsPerNode(this->NumberOfCellsPerNode);
		unaffectedSideTree->SetMaxLevel(affectedSideTreeMaxLevel);
		unaffectedSideTree->BuildLocator();
		int level = unaffectedSideTree->GetLevel();

		affectedSideTree->SetDataSet(affectedSidePoly);
		affectedSideTree->AutomaticOn();
		affectedSideTree->SetNumberOfCellsPerNode(this->NumberOfCellsPerNode);
		unaffectedSideTree->SetMaxLevel(unAffectedSideTreeMaxLevel);
		affectedSideTree->BuildLocator();
	}


	//设置树的包围盒容差
	unaffectedSideTree->SetTolerance(this->BoxTolerance);
	affectedSideTree->SetTolerance(this->BoxTolerance);

	// 创建变换矩阵
	vtkMatrix4x4* matrix = vtkMatrix4x4::New();
	vtkMatrix4x4* tmpMatrix = vtkMatrix4x4::New();

	if (unaffectedSideMatrix != nullptr || affectedSideMatrix != nullptr)
	{
		vtkMatrix4x4::Invert(unaffectedSideMatrix, tmpMatrix);
		// the sequence of multiplication is significant
		vtkMatrix4x4::Multiply4x4(tmpMatrix, affectedSideMatrix, matrix);
	}
	else
	{
		// vtkWarningMacro(<< "Set two transforms or two matrices");
		return 1;
	}

	//包围盒碰撞检测
	int boxTests = unaffectedSideTree->IntersectWithOBBTree(affectedSideTree, matrix, ComputeCollisions, this);


	matrix->Delete();
	tmpMatrix->Delete();

	if (GetNumberOfContacts() > 0) {
		return true;
	}
	return false;
}
bool collisionDetect::isCollision()
{
	if (GetNumberOfContacts() > 0) {
		return true;
	}
	return false;
}
int collisionDetect::GetNumberOfContacts() {
	int number = affectedSideContactCells->GetNumberOfTuples();
	return affectedSideContactCells->GetNumberOfTuples();
}

