#ifndef PIDCONTROL_ALGORITHM_H
#define PIDCONTROL_ALGORITHM_H
#include <Temperature.h>
class PIDcontrol_algorithm
{
    private:
		float *kp;
		float *ki;
		float *taimmeantemperature;
    public:
		float costvalue;
		int coolsection;
		const Continuous_Caster* CasterOne;
		PIDcontrol_algorithm(float[], float[], const Continuous_Caster &, float*);
		~PIDcontrol_algorithm();
		void compute_costvalue(float*);
		void updateh(float*, const float*);
		void print();
		void outputdata(const Temperature &, const float*);
};
#endif
