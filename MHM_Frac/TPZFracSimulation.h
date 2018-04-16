//
//  TPZFracSimulation.hpp
//  PZ
//
//  Created by Philippe Devloo on 26/09/17.
//

#ifndef TPZFracSimulation_hpp
#define TPZFracSimulation_hpp

#include "TPZFracSet.h"
#include "TPZGmshReader.h"
#include "TPZMHMixedHybridMeshControl.h"
#include <stdio.h>

class TPZFracSimulation
{
    TPZAutoPointer<TPZMHMixedHybridMeshControl> fMHM;
    
    TPZGmshReader fGmsh;
    
    TPZFracSet fFracSet;
    
    /// fracture simulation type 0 - steady state 1 - time dependent
    int fSimulationType = 0;
    
    /// value of the initial pressure for time evolution problems
    REAL fInitialPressure = 1;

public:
    /// time stepping table
    TPZManVector<std::pair<STATE,int> > fTimeSteps;
private:
    /// post processing file name
    std::string fPostProcessRootname;
    
    /// map between material names and material id
    std::map<std::string, int> fMaterialIds;
    
public:
    /// post processing boundary names
    TPZManVector<std::pair<std::string,std::string> > fPostProcnames;

    
    TPZFracSimulation(TPZAutoPointer<TPZMHMixedHybridMeshControl> MHM) : fMHM(MHM)
    {
        if(!fMHM) DebugStop();
    }
    
    /// Build an MHM object with information given by the root name
    // two files must be present
    // rootname.data
    // rootname.msh
    void ReadDataFile(const std::string &rootname);
    
    /// material id for a given name
    int MaterialId(const std::string &matname)
    {
        if(fMaterialIds.find(matname) == fMaterialIds.end())
        {
            std::cout << "Material name " << matname << " NOT FOUND!!!\n";
            return 0;
        }
        else
        {
            return fMaterialIds[matname];
        }
    }
    
    int SimulationType()
    {
        return fSimulationType;
    }
    
    REAL InitialPressure()
    {
        return fInitialPressure;
    }
    
    TPZAutoPointer<TPZMHMixedHybridMeshControl> MHM()
    {
        return fMHM;
    }
private:
    
    /// read the data in the data file and put the information in right places
    // creates material objects for the computational meshes
    void ReadDataFile(std::ifstream &input);
    
    /// reads the preamble of the data file
    void ReadPreamble(std::ifstream &input);
    
    /// reads the fracture information of the data file
    void ReadFractures(std::ifstream &input);
    
    /// creates and inserts a Darcy or ParabolicDarcy object in the mesh
    void InsertDarcyMaterial(int matid, REAL permeability, REAL rho);

    /// creates and inserts the boundary condition objects
    void InsertDarcyBCMaterial(int matid, int dimension, int bctype, REAL val);
    
    /// adjust the geometric element read from gmesh
    void AdjustGeometricMesh(const std::string &rootname);
    
    /// adjust the boundary condition type of the elements
    void SetStandardBoundaryElements();

};
#endif /* TPZFracSimulation_hpp */
