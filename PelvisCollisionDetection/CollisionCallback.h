#pragma once
#include <vtkCommand.h>
#include <string>
#include <time.h>
class CollisionCallback : public vtkCommand
{
	vtkTypeMacro(CollisionCallback, vtkCommand);
public:
	virtual void Execute(vtkObject * caller, unsigned long eventId, void * callData) override;

	double getTotalTime() { return m_totalTime; }

	static CollisionCallback *New()
	{
		return new CollisionCallback;
	}
private:
	double m_totalTime = 0.0;
	clock_t m_begin;
	clock_t m_end;
};

