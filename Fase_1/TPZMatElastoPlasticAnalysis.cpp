//
//  TPZMatElastoPlasticAnalysis.hpp
//  Benchmark0a
//
//  Created by Pablo Carvalho on 14/09/18.
//


#include "TPZMatElastoPlasticAnalysis.h"

TPZMatElastoPlasticAnalysis::TPZMatElastoPlasticAnalysis() : TPZAnalysis(){
    
    m_simulation_data = NULL;
    m_X_n.Resize(0, 0);
    m_X.Resize(0, 0);
    m_error = 0;
    m_dx_norm = 0;
    m_k_iterations = 0;
    m_post_processor = NULL;
    m_var_names.resize(0);
    
}

TPZMatElastoPlasticAnalysis::~TPZMatElastoPlasticAnalysis(){
    
}

TPZMatElastoPlasticAnalysis::TPZMatElastoPlasticAnalysis(const TPZMatElastoPlasticAnalysis & other){
    
    m_simulation_data   = other.m_simulation_data;
    m_X_n               = other.m_X_n;
    m_X                 = other.m_X;
    m_error             = other.m_error;
    m_dx_norm           = other.m_dx_norm;
    m_k_iterations      = other.m_k_iterations;
    m_post_processor    = other.m_post_processor;
    m_var_names         = other.m_var_names;
    
}

void TPZMatElastoPlasticAnalysis::ConfigurateAnalysis(DecomposeType decomposition, TPZSimulationData * simulation_data){
    SetSimulationData(simulation_data);
    TPZStepSolver<STATE> step;
    unsigned int number_threads = m_simulation_data->Get_n_threads();
    
    if(!Mesh()){
        std::cout << "Call SetCompMesh method." << std::endl;
        DebugStop();
    }
    
    switch (decomposition) {
        case ECholesky:
        {
            TPZSkylineStructMatrix struct_mat(Mesh());
            struct_mat.SetNumThreads(number_threads);
            this->SetStructuralMatrix(struct_mat);
        }
            break;
        case ELDLt:
        {
            TPZSymetricSpStructMatrix struct_mat(Mesh());
            struct_mat.SetNumThreads(number_threads);
            this->SetStructuralMatrix(struct_mat);
        }
            break;
        default:
        {
            DebugStop();
        }
            break;
    }
    step.SetDirect(decomposition);
    this->SetSolver(step);
    this->Solution().Resize(Mesh()->Solution().Rows(), 1);
    m_X.Resize(Mesh()->Solution().Rows(), 1);
    m_X_n.Resize(Mesh()->Solution().Rows(), 1);
    
    int n_threads = m_simulation_data->Get_n_threads();
    m_post_processor = new TPZPostProcAnalysis;
    m_post_processor->SetCompMesh(Mesh());
    
    TPZManVector<std::pair<int, TPZManVector<int,12>>,12>  material_ids = m_simulation_data->MaterialIds();
    TPZManVector<int,10> post_mat_id(1);
    
    int matid = 1;
    post_mat_id[0] = matid;
    
    // @TODO:: MS, please transfer from xml file
    m_var_names.Push("ux");
    m_var_names.Push("uy");
    m_var_names.Push("sxx");
    m_var_names.Push("syy");
    m_var_names.Push("szz");
    m_var_names.Push("exx");
    m_var_names.Push("eyy");
    m_var_names.Push("ezz");
    m_var_names.Push("epxx");
    m_var_names.Push("epyy");
    m_var_names.Push("epzz");
    
//    if (m_simulation_data->Dimension() == 3) {
//        m_var_names.Push("uz");
//    }
    
    m_post_processor->SetPostProcessVariables(post_mat_id, m_var_names);
    
    TPZFStructMatrix structmatrix(m_post_processor->Mesh());
    structmatrix.SetNumThreads(n_threads);
    m_post_processor->SetStructuralMatrix(structmatrix);
    
}

void TPZMatElastoPlasticAnalysis::ExecuteNewtonInteration(){
    this->Assemble();
    this->Rhs() *= -1.0;
    this->Solve();
}

void TPZMatElastoPlasticAnalysis::ExecuteOneTimeStep(bool must_accept_solution_Q){
    
    if (m_simulation_data->IsInitialStateQ()) {
        m_X = Solution();
    }
    
    m_simulation_data->SetCurrentStateQ(true);
    
    //    // Reset du to zero
    //    Solution().Zero();
    //    LoadSolution(Solution());
    
    TPZFMatrix<STATE> dx(Solution());
    bool residual_stop_criterion_Q = false;
    bool correction_stop_criterion_Q = false;
    REAL norm_res, norm_dx;
    REAL r_norm = m_simulation_data->Get_epsilon_res();
    REAL dx_norm = m_simulation_data->Get_epsilon_cor();
    int n_it = m_simulation_data->Get_n_iterations();
    
    for (int i = 1; i <= n_it; i++) {
        this->ExecuteNewtonInteration();
        dx += Solution();
        norm_dx  = Norm(Solution());
        LoadSolution(dx);
        AssembleResidual();
        norm_res = Norm(this->Rhs());
        residual_stop_criterion_Q   = norm_res < r_norm;
        correction_stop_criterion_Q = norm_dx  < dx_norm;
        
        m_k_iterations = i;
        m_error = norm_res;
        m_dx_norm = norm_dx;
        
        if (residual_stop_criterion_Q ||  correction_stop_criterion_Q) {
#ifdef PZDEBUG
            std::cout << "TPMRSGeomechanicAnalysis:: Nonlinear process converged with residue norm = " << norm_res << std::endl;
            std::cout << "TPMRSGeomechanicAnalysis:: Number of iterations = " << i << std::endl;
            std::cout << "TPMRSGeomechanicAnalysis:: Correction norm = " << norm_dx << std::endl;
#endif
            LoadSolution(dx);
            this->AcceptPseudoTimeStepSolution();
            break;
        }
    }
    
    if (residual_stop_criterion_Q == false) {
        std::cout << "TPMRSGeomechanicAnalysis:: Nonlinear process not converged with residue norm = " << norm_res << std::endl;
    }
    
}

void TPZMatElastoPlasticAnalysis::UpdateState(){
    m_simulation_data->SetTransferCurrentToLastQ(true);
    AcceptPseudoTimeStepSolution();
    m_simulation_data->SetTransferCurrentToLastQ(false);
}


void TPZMatElastoPlasticAnalysis::PostProcessTimeStep(std::string & file){
    
    int dim = Mesh()->Dimension();
    int div = 0;
    TPZStack< std::string> vecnames;
    m_post_processor->TransferSolution();
    m_post_processor->DefineGraphMesh(dim,m_var_names,vecnames,file);
    m_post_processor->PostProcess(div,dim);
}

void TPZMatElastoPlasticAnalysis::AcceptPseudoTimeStepSolution(){

    SetUpdateMemmory(true);
    AssembleResidual();
    SetUpdateMemmory(false);
    
    m_simulation_data->Set_must_accept_solution_Q(false);
    
//    bool state = m_simulation_data->IsCurrentStateQ();
//    if (state) {
//        m_simulation_data->Set_must_accept_solution_Q(true);
//        AssembleResidual();
//        m_simulation_data->Set_must_accept_solution_Q(false);
//    }else{
//        m_simulation_data->Set_must_accept_solution_Q(true);
//        AssembleResidual();
//        m_simulation_data->Set_must_accept_solution_Q(false);
//    }
}


void TPZMatElastoPlasticAnalysis::LoadCurrentState(){
    LoadSolution(m_X_n);
    DebugStop();
}

void TPZMatElastoPlasticAnalysis::LoadLastState(){
    LoadSolution(m_X);
    DebugStop();
}


void TPZMatElastoPlasticAnalysis::SetUpdateMemmory(bool accept_solution_Q){

    std::map<int, TPZMaterial * >::iterator mit;
    std::map<int, TPZMaterial *> & refMatVec = Mesh()->MaterialVec();

    TPZMatWithMem<TPZMemoryDFN> * material_with_memory;
    for(mit=refMatVec.begin(); mit!= refMatVec.end(); mit++)
    {
        material_with_memory = dynamic_cast<TPZMatWithMem<TPZMemoryDFN> *>( mit->second );
        if(material_with_memory)
        {
            material_with_memory->SetUpdateMem(accept_solution_Q);
        }
    }
}


