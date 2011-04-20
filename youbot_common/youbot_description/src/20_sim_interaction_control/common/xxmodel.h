#ifndef SUBMODEL20SIM_H
#define SUBMODEL20SIM_H

#include "xxtypes.h"
#include "xxmatrix.h"


class Submodel20sim
{
	protected:
		friend class IntegrationMethod;
		friend class Discrete;
		friend class Euler;
		friend class RungeKutta2;
		friend class RungeKutta4;
		virtual void CalculateDynamic (void) {};

		bool     m_initialize;
		bool     m_major;

	public:
		virtual ~Submodel20sim(){};
		
		XXDouble m_step_size;
		XXDouble m_start_time;
		XXDouble m_finish_time;
		XXDouble m_time;

		/* the variable count */
		int m_number_constants;
		int m_number_parameters;
		int m_number_initialvalues;
		int m_number_variables;
		int m_number_states;
		int m_number_rates;
		int m_number_matrices;
		int m_number_unnamed;

		/* the variable arrays are allocated in the derived submodel class */
		XXDouble* m_C;					/* constants */
		XXDouble* m_P;					/* parameters */
		XXDouble* m_I;				/* initial values */
		XXDouble* m_V;					/* variables */
		XXDouble* m_s;						/* states */
		XXDouble* m_R;						/* rates (or new states) */
		XXMatrix* m_M;					/* matrices */
		XXDouble* m_U;					/* unnamed */
		XXDouble* m_workarray;
};

#endif 	// SUBMODEL20SIM_H

