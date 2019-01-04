#include <PIDcontrol_algorithm.h>
#include <Temperature.h>
#include <iostream>
#include <fstream>
#include <Mesh.h>
#include <Temperature.h>
#include <math.h>
using namespace std;
PIDcontrol_algorithm::PIDcontrol_algorithm(float m_kp[], float m_ki[], const Continuous_Caster & Caster, float* m_taimmeantemperature)
{
	CasterOne = &Caster;
	coolsection = CasterOne->coolsection;
	costvalue = 0.0f;
	kp = new float[coolsection]();
	ki = new float[coolsection]();
	taimmeantemperature = m_taimmeantemperature;
	for (int i = 0; i < coolsection; i++)
	{
		kp[i] = m_kp[i];
		ki[i] = m_ki[i];
	}
}

PIDcontrol_algorithm::~PIDcontrol_algorithm()
{
	delete[] kp;
	delete[] ki;
	taimmeantemperature = 0;
}

void PIDcontrol_algorithm::updateh(float*h_init, const float* meantemperature)
{
	for (int i = 0; i < coolsection; i++)
	{
		h_init[i + CasterOne->moldsection] += kp[i] * (meantemperature[i] - taimmeantemperature[i]);
	}
}

void PIDcontrol_algorithm::compute_costvalue(float* measuredtemperature)
{
	costvalue = 0.0f;
	for (int i = 0; i < coolsection; i++)
		costvalue += (measuredtemperature[i] - taimmeantemperature[i]) * (measuredtemperature[i] - taimmeantemperature[i]);
	costvalue = float(pow(costvalue / coolsection, 0.5));
}

void PIDcontrol_algorithm::print()
{
	cout << "cost value = " << costvalue << endl;
}

void PIDcontrol_algorithm::outputdata(const Temperature & m_SteelTemperature, const float* h_init)
{
	ofstream outputfile;
	outputfile.open("C:\\T_Result_3DCPU_PID.csv", ios::app);
	for (int i = 0; i < m_SteelTemperature.mesh->ny; i++)
		outputfile << m_SteelTemperature.T_Surface[i] << ",";
	outputfile << endl;
	outputfile.close();
}