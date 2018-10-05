//
//  TPZMatFractureBB.h
//  Benchmark0a
//
//  Created by Pablo Carvalho on 20/08/18.
//

#ifndef TPZMatFractureBB_h
#define TPZMatFractureBB_h

#include <stdio.h>
#include <iostream>
#include "pzdiscgal.h"
#include "TPZMatWithMem.h"
#include "tpzautopointer.h"
#include "TPZMemoryFracDFN.h"

/// Material which implements a Lagrange Multiplier

template <class TMEM = TPZMemoryFracDFN>
class TPZMatFractureBB : public TPZMatWithMem< TMEM, TPZDiscontinuousGalerkin>
{
    protected:
    
    // Number of state variables
    int fNStateVariables;
    
    // Dimensiona associated with the material
    int fDimension;
    
    STATE fMultiplier;
    
    public :
    // Simple constructor
    TPZMatFractureBB() : TPZRegisterClassId(&TPZMatFractureBB::ClassId), TPZMatWithMem<TMEM,
    TPZDiscontinuousGalerkin >()
    {
        
    }
    // Constructor with the index of the material object within the vector
    TPZMatFractureBB(int nummat, int dimension, int nstate) : TPZRegisterClassId(&TPZMatFractureBB::ClassId),TPZMatWithMem<TMEM,
    TPZDiscontinuousGalerkin>(nummat), fNStateVariables(nstate), fDimension(dimension), fMultiplier(1.)
    {
        
    }
    
    // Copy constructor
    TPZMatFractureBB(const TPZMatFractureBB &copy) : TPZRegisterClassId(&TPZMatFractureBB::ClassId),TPZMatWithMem<TMEM,
    TPZDiscontinuousGalerkin>(copy), fNStateVariables(copy.fNStateVariables), fDimension(copy.fDimension), fMultiplier(copy.fMultiplier)
    {
        
    }
    
    TPZMatFractureBB &operator=(const TPZMatFractureBB &copy)
    {
        TPZDiscontinuousGalerkin::operator=(copy);
        fNStateVariables = copy.fNStateVariables;
        fDimension = copy.fDimension;
        fMultiplier = copy.fMultiplier;
        return *this;
    }
    
//    TPZMatFractureBB *NewMaterial()
//    {
//        return new TPZMatFractureBB(*this);
//    }
    
    // Destructor
    ~TPZMatFractureBB()
    {
        
    }
    
    // Returns the integrable dimension of the material
    virtual int Dimension() const
    {
        return fDimension;
    }
    
    virtual void SetMultiplier(STATE mult)
    {
        fMultiplier = mult;
    }
    
    
    virtual std::string Name()
    {
        return "TPZMatFractureBB";
    }
    
    
    // Fill material data parameter with necessary requirements for the ContributeInterface method.
    // Here, in base class, all requirements are considered as necessary. \n
    // Each derived class may optimize performance by selecting only the necessary data.
    virtual void FillDataRequirementsInterface(TPZMaterialData &data)
    {
        data.SetAllRequirements(false);
    }

    // Contribute methods
    virtual void Contribute(TPZVec<TPZMaterialData> &datavec, REAL weight, TPZFMatrix<STATE> &ek, TPZFMatrix<STATE> &ef){
        DebugStop();
    }
    
    virtual void Contribute(TPZMaterialData &data, REAL weight, TPZFMatrix<STATE> &ek, TPZFMatrix<STATE> &ef);
    
    virtual void ContributeBC(TPZMaterialData &data, REAL weight, TPZFMatrix<STATE> &ek, TPZFMatrix<STATE> &ef, TPZBndCond &bc){
        DebugStop();
    }
    
    virtual void ContributeBCInterface(TPZMaterialData &data, TPZMaterialData &dataleft, REAL weight, TPZFMatrix<STATE> &ek,TPZFMatrix<STATE> &ef,TPZBndCond &bc){
        DebugStop();
    }
    
    // It computes a contribution to stiffness matrix and load vector at one integration point
    virtual void ContributeInterface(TPZMaterialData &data, TPZMaterialData &dataleft, TPZMaterialData &dataright, REAL weight, TPZFMatrix<STATE> &ek, TPZFMatrix<STATE> &ef){
        DebugStop();
    }
    
    // Computes a contribution to the stiffness matrix and load vector at one integration point to multiphysics simulation
    virtual void ContributeInterface(TPZMaterialData &data, TPZVec<TPZMaterialData> &dataleft, TPZVec<TPZMaterialData> &dataright, REAL weight, TPZFMatrix<STATE> &ek, TPZFMatrix<STATE> &ef){
        DebugStop();
    }
    
    // It computes a contribution to residual vector at one integration point
    virtual void ContributeInterface(TPZMaterialData &data, TPZMaterialData &dataleft, TPZMaterialData &dataright, REAL weight, TPZFMatrix<STATE> &ef){
        DebugStop();
    }
    
    // Computes a contribution to residual vector at one integration point
    virtual void ContributeInterface(TPZMaterialData &data, TPZVec<TPZMaterialData> &dataleft, TPZVec<TPZMaterialData> &dataright, REAL weight, TPZFMatrix<STATE> &ef)
    {
        DebugStop();
        ContributeInterface(data, dataleft[0], dataright[0], weight, ef);
    }
    
    virtual int NStateVariables()
    {
        return fNStateVariables;
    }
    
    // Updates the leak off memory
    void UpdateMemory(TPZVec<TPZMaterialData> &datavec);
    
    // Updates the leak off memory
    void UpdateMemory(TPZMaterialData &data, TPZVec<TPZMaterialData> &datavec);
    

public:
    
    // Unique identifier for serialization purposes
    virtual int ClassId() const;
    
    
    // Saves the element data to a stream
    virtual void Write(TPZStream &buf, int withclassid) const;
    
    
    // Reads the element data from a stream
    virtual void Read(TPZStream &buf, void *context);
    

};


#endif /* TPZMatFractureBB_h */
