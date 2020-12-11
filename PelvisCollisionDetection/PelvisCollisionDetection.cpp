// PelvisCollisionDetection.cpp: 定义应用程序的入口点。
//

#include "PelvisCollisionDetection.h"

using namespace std;

auto leftM = vtkSmartPointer<vtkMatrix4x4>::New();
auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
auto renderer = vtkSmartPointer<vtkRenderer>::New();
auto stl_left_reader = vtkSmartPointer<vtkSTLReader>::New();
auto stl_right_reader = vtkSmartPointer<vtkSTLReader>::New();
auto interpot = vtkSmartPointer<vtkTransformInterpolator>::New();
auto rightM = vtkSmartPointer<vtkMatrix4x4>::New();
auto interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
auto colors = vtkSmartPointer<vtkNamedColors>::New();
collisionDetect* collion = new collisionDetect();
auto RightActor = vtkSmartPointer<vtkActor>::New();
auto LeftActor = vtkSmartPointer<vtkActor>::New();

//检测碰撞的方法


// Handle mouse events
class MouseInteractorStyle2 : public vtkInteractorStyleTrackballActor
{
public:
	static MouseInteractorStyle2* New();
	vtkTypeMacro(MouseInteractorStyle2, vtkInteractorStyleTrackballCamera);
	bool pciked = false;
	vtkActor* pickedActor = nullptr;

	virtual void OnLeftButtonDown() override
	{
		//int* clickPos = this->GetInteractor()->GetEventPosition();

		//// Pick from this location.
		//vtkSmartPointer<vtkPropPicker>  picker =
		//	vtkSmartPointer<vtkPropPicker>::New();
		//vtkRenderer* s = this->GetCurrentRenderer();
		//
		//picker->Pick(clickPos[0], clickPos[1], 0, renderer);

		//double* pos = picker->GetPickPosition();
		//std::cout << "Pick position (world coordinates) is: "
		//	<< pos[0] << " " << pos[1]
		//	<< " " << pos[2] << std::endl;
		//pickedActor = picker->GetActor();
		//std::cout << "Picked actor: " << picker->GetActor() << std::endl;
		//// Forward events
		//this->pciked = true;
		vtkInteractorStyleTrackballActor::OnLeftButtonDown();

	}
	void OnMouseMove() override {

		vtkInteractorStyleTrackballActor::OnMouseMove();
	}
private:

};
vtkStandardNewMacro(MouseInteractorStyle2);


//变换矩阵组合
vtkSmartPointer<vtkMatrix4x4> GetTransform() {
	//复位矩阵读取
	FileToMatrix* reducFile = new FileToMatrix(".//Config//reduc.txt");
	vtkSmartPointer<vtkMatrix4x4> reducM = reducFile->GetTransformMatrix();
	
	//坐标系变换读取
	FileToMatrix* TransformFile = new FileToMatrix(".//Config//transform.txt");
	vtkSmartPointer<vtkMatrix4x4> TransformM = TransformFile->GetTransformMatrix();

	//变化结果
	auto ResultTransformM = vtkSmartPointer<vtkMatrix4x4>::New();

	//矩阵计算
	vtkMatrix4x4::Multiply4x4(reducM, TransformM, ResultTransformM);
	TransformM->Invert();
	vtkMatrix4x4::Multiply4x4(TransformM, ResultTransformM, ResultTransformM);

	return ResultTransformM;
}
int i = 0;
bool collision = false;
void callBackFunc(vtkObject* caller, unsigned long eid, void* clientdata, void* calldata) {
	vtkRenderWindowInteractor* renderWindow = dynamic_cast<vtkRenderWindowInteractor*>(caller);
	int collisionIndex = -1;
	vtkPolyData* ContactsPolydata;

	auto transformM = vtkSmartPointer<vtkTransform>::New();
	if (collion->isCollision(stl_right_reader->GetOutput(),RightActor->GetMatrix() , stl_left_reader->GetOutput(), LeftActor->GetMatrix())) {
		int a = 10;
	}
	//if(i<100 && !collision)
	//{
	//	renderWindow->Render();
	//	//插值取矩阵
	//	interpot->InterpolateTransform(i, transformM);
	//	leftM->DeepCopy(transformM->GetMatrix());
	//	if (collion->isCollision(stl_right_reader->GetOutput(), rightM, stl_left_reader->GetOutput(), leftM)) {
	//		ContactsPolydata = collion->getContactPolydata();
	//		collision = true;
	//	}
	//	++i;
	//}
}

int main() 
{	
	//初始化左右盆骨	
	stl_left_reader->SetFileName(".//Config//left.stl");
	stl_left_reader->Update();

	stl_right_reader->SetFileName(".//Config//right.stl");
	stl_right_reader->Update();

	leftM->DeepCopy(GetTransform());
	//插值算法
	interpot->AddTransform(0, leftM);
	leftM->Invert();
	interpot->AddTransform(100, leftM);
	leftM->DeepCopy(GetTransform());

	//碰撞检测初始化
	
	collion->initCollisionTree(stl_right_reader->GetOutput(), stl_left_reader->GetOutput());

	//创建坐标轴
	auto axe = vtkSmartPointer<vtkAxesActor>::New();
	axe->SetTotalLength(100, 100, 100);
	axe->SetShaftType(0);
	axe->SetCylinderRadius(0.02);
	axe->SetAxisLabels(0);
	axe->VisibilityOff();

	//渲染步骤
	//构建Actor
	auto RightMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	RightMapper->SetInputConnection(stl_right_reader->GetOutputPort());
	RightActor->SetMapper(RightMapper);
	RightActor->GetProperty()->BackfaceCullingOn();
	RightActor->SetUserMatrix(rightM);

	auto LeftMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	LeftMapper->SetInputConnection(stl_left_reader->GetOutputPort());
	LeftMapper->ScalarVisibilityOff();
	LeftActor->SetMapper(LeftMapper);
	LeftActor->GetProperty()->BackfaceCullingOn();
	LeftActor->SetUserMatrix(leftM);

	//碰撞网格
	auto ContactsCellmapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	ContactsCellmapper->SetInputData(collion->getContactPolydata());
	ContactsCellmapper->SetResolveCoincidentTopologyToPolygonOffset();
	auto ContactsCellActor = vtkSmartPointer<vtkActor>::New();
	ContactsCellActor->SetMapper(ContactsCellmapper);
	ContactsCellActor->GetProperty()->SetColor(colors->GetColor3d("Red").GetData());
	ContactsCellActor->GetProperty()->SetLineWidth(3.0);

	auto txt = vtkSmartPointer<vtkTextActor>::New();
    txt->GetTextProperty()->SetFontSize(18);

	renderer->UseHiddenLineRemovalOn();
	renderer->AddActor(LeftActor);
	renderer->AddActor(RightActor);
	renderer->AddActor(txt);
	renderer->AddActor(axe);
	renderer->SetBackground(colors->GetColor3d("Gray").GetData());

	renderWindow->SetSize(1280, 1280);
	renderWindow->AddRenderer(renderer);
	interactor->SetRenderWindow(renderWindow);
	interactor->Initialize();
	auto callfunc = vtkSmartPointer<vtkCallbackCommand>::New();
	callfunc->SetCallback(callBackFunc);
	interactor->AddObserver(vtkCommand::TimerEvent, callfunc);
	interactor->CreateRepeatingTimer(100);
	auto interactorstlye = vtkSmartPointer<MouseInteractorStyle2>::New();
	interactor->SetInteractorStyle(interactorstlye);
	renderer->ResetCameraClippingRange();
	renderWindow->Render();



	auto mapper8 = vtkSmartPointer<vtkPolyDataMapper>::New();
	auto actor8 = vtkSmartPointer<vtkActor>::New();
	actor8->SetMapper(mapper8);
	actor8->GetProperty()->BackfaceCullingOn();
	actor8->GetProperty()->SetDiffuseColor(
		colors->GetColor3d("Red").GetData());
	actor8->GetProperty()->SetRepresentationToWireframe();
	mapper8->SetInputData(collion->getContactPolydata());
	renderer->AddActor(actor8);

	interactor->Start();
	return 0;
}
