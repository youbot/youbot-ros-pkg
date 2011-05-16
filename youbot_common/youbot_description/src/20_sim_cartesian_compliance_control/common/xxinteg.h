#ifndef XX_INTEG_H
#define XX_INTEG_H

#include "xxtypes.h"

class Submodel20sim;	//Forward declaration

class IntegrationMethod
{
  public:
	IntegrationMethod(void) {	};
	virtual ~IntegrationMethod(void) {	};
	virtual void Initialize(Submodel20sim* themodel) = 0;
	virtual void Terminate() = 0;
	virtual void Step() = 0;
	
	//variables
	bool major;	//integration step: major/minor step
  protected:
	Submodel20sim* model;
};

class Euler: public IntegrationMethod
{
public:
	Euler(void);
	~Euler(void);
	void Initialize(Submodel20sim* themodel);
	void Terminate();
	void Step();

};

class Discrete: public IntegrationMethod
{
public:
	Discrete(void);
	~Discrete(void);
	void Initialize(Submodel20sim* themodel);
	void Terminate();
	void Step();
};

class RungeKutta4: public IntegrationMethod
{
  public:
	RungeKutta4(void);
	~RungeKutta4(void);
	void Initialize(Submodel20sim* themodel);
	void Terminate();
	void Step();
	
  private:
	XXDouble* q0;
	XXDouble* q1;
	XXDouble* q2;
	XXDouble* q3;
	XXDouble* q4;
};

class RungeKutta2: public IntegrationMethod
{
  public:
	RungeKutta2(void);
	~RungeKutta2(void);
	void Initialize(Submodel20sim* themodel);
	void Terminate();
	void Step();
	
  private:
	XXDouble* q0;
};

#endif

