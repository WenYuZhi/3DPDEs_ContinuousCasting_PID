#include <Inputdata.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
using namespace std;
Inputdata::Inputdata()
{

}

void Inputdata::read_continuouscaster()
{
	ifstream fin;
	fin.open("C:\\Thesis_Project\\3DPDEs_ContinuousCasting_MPC_Sparse_Stochastic_Gradient\\3DPDEs_ContinuousCasting_MPC\\3DPDEs_ContinuousCasting_MPC\\c.txt");
	string str;
	while (!fin.eof())
	{
		getline(fin, str);
		cout << str << endl;
	}
	fin.close();
}

void Inputdata::read_roll()
{

}

void Inputdata::read_steelsize()
{
}

void Inputdata::read_steel_componment()
{

}