// TestApplication.cpp : Defines the entry point for the console application.
//


#include <stdio.h>
#include <tchar.h>
#include <SimulationContainer.h>

int main()
{
	SimulationContainer container(2);
	container.addAgent("1");
	container.addAgent("2");
	container.addMolecule("1");
    return 0;
}

