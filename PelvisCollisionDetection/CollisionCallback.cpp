#include "CollisionCallback.h"


void CollisionCallback::Execute(vtkObject * caller, unsigned long eventId, void * callData)
{
	std::string Classname = caller->GetClassName();
	switch (eventId)
	{
	case vtkCommand::StartEvent:
		m_begin = clock();
		break;
	case vtkCommand::EndEvent:
		m_end = clock();
		break;
	default:
		break;
	}
	m_totalTime = (double)(m_end - m_begin) / CLOCKS_PER_SEC;

}
