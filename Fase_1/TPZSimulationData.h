//
//  TPZSimulationData.h
//  PZ
//
//  Created by Omar on 8/28/18.
//
//

#ifndef TPZSimulationData_h
#define TPZSimulationData_h

#include <stdio.h>
#include "pzvec.h"
#include "pzfmatrix.h"
#include "pzstack.h"
#include <iostream>
#include <stdio.h>
#include <string>
#include "TPZGmshReader.h"
#include "pzgmesh.h"
#include "TPZVTKGeoMesh.h"
#include "pzcheckgeom.h"


/** @brief Object conatining several kind of informations being used anytime and anywhere */
class TPZSimulationData
{
    
protected:
    
    /** @brief Spatial refinemenet level */
    int m_h_level;
    
    /** @brief Polynomial order for elasticity component */
    int m_elasticity_order;
    
    /** @brief Material ID related to Elasticity material */
    int m_elasticity_ID;
    
    /** @brief Polynomial order for diffusion component */
    int m_darcy_order;
    
    /** @brief Material ID related to Darcy material */
    int m_darcy_ID;
    
    /** @brief Physical dimension of the domain */
    int m_dimesion;
    
    /** @brief Number of iteration */
    int m_n_iterations;
    
    /** @brief Residue overal tolerance */
    REAL m_epsilon_res;
    
    /** @brief Correction overal tolerance */
    REAL m_epsilon_cor;
    
    /** @brief Number of thread */
    int m_n_threads;
    
    bool m_is_initial_state_Q;
    
    bool m_is_current_state_Q;
    
    /** @brief Directive that states if the last memory solution is being transferred to the current memory solution */
    bool m_transfer_current_to_last_solution_Q;
    
    /** @brief Directive that states if the current solution must be accepted inside the memory  */
    bool m_must_accept_solution_Q;
    
    /** @brief Directive that states the use of dual (true) or pirmal (false) formulation for monophacic flow  */
    bool m_is_dual_formulation_Q;
    
    /** @brief Name for the Gmsh geometry file being used */
    std::string m_geometry_file;
    
    /** @brief Name for the vtk files being postprocessed */
    std::string m_vtk_file;
    
    /** @brief Number of vtk resolution during postprocessing */
    int m_vtk_resolution;
    
    /** @brief Vector that storages only volumetric material identifiers (higher dimension elements) */
    std::vector<int> m_volumetric_material_id;
    
    /** @brief Material and boundaries identifiers sorted per region */
    TPZManVector<std::pair<int, TPZManVector<int,12>>,12> m_mat_ids;
    
public:
    
    
    /** @brief default constructor */
    TPZSimulationData();
    
    /** @brief default constructor */
    TPZSimulationData(const TPZSimulationData & other);
    
    /** @brief default constructor */
    TPZSimulationData &operator=(const TPZSimulationData &other);
    
    /** @brief destructor */
    ~TPZSimulationData();
    
    /** @brief Print object attributes */
    void Print();
    
    /** @brief Set the directive that states if the current memory solution is being transferred to the last memory solution */
    void SetTransferCurrentToLastQ(bool transfer_current_to_last_solution_Q) { m_transfer_current_to_last_solution_Q = transfer_current_to_last_solution_Q; }
    
    /** @brief Get the directive that states if the current memory solution is being transferred to the last memory solution */
    bool GetTransferCurrentToLastQ() { return m_transfer_current_to_last_solution_Q; }
    
    /// Access methods
    
    /** @brief Get the material and boundaries identifiers sorted per region */
    TPZManVector<std::pair<int, TPZManVector<int,12>>,12> & MaterialIds() { return m_mat_ids; }
    
    /** @brief Set initial state */
    void SetInitialStateQ(bool state) { m_is_initial_state_Q = state; }
    
    /** @brief Get initial state */
    bool IsInitialStateQ() {return m_is_initial_state_Q;}
    
    /** @brief Set current time state */
    void SetCurrentStateQ(bool state) { m_is_current_state_Q = state; }
    
    /** @brief Get current time state */
    bool IsCurrentStateQ() {return m_is_current_state_Q;}
    
    /** @brief Set the directive that states if the current solution must be accepted inside the memory  */
    void Set_must_accept_solution_Q(bool must_accept_solution_Q){
        m_must_accept_solution_Q = must_accept_solution_Q;
    }
    
    /** @brief Get the directive that states if the current solution must be accepted inside the memory  */
    bool Get_must_accept_solution_Q() { return m_must_accept_solution_Q; }
    
    /** @brief Get the the use of dual (true) or pirmal (false) formulation for monophacic flow  */
    bool Get_is_dual_formulation_Q() { return m_is_dual_formulation_Q; }
    
    /** @brief Set the spatial refinemenet level */
    void Set_h_level(int h_level){
        m_h_level = h_level;
    }
    
    /** @brief Get the spatial refinemenet level */
    int Get_h_level(){
        return m_h_level;
    }
    
    /** @brief Set the polynomial order for the elasticity approximation */
    void Set_elasticity_order(int elasticity_order){
        m_elasticity_order = elasticity_order;
    }
    /** @brief Set the material ID related to Elasticity material */
    void Set_elasticity_matid(int elasticity_matid){
        m_elasticity_ID = elasticity_matid;
    }
    
    /** @brief Get the material ID related to Elasticity material */
    int Get_elasticity_matid(){
        return m_elasticity_ID;
    }
    
    /** @brief Get the polynomial order for the elasticity approximation  */
    int Get_elasticity_order(){
        return m_elasticity_order;
    }
    
    /** @brief Set the polynomial order for the Darcy's approximation */
    void Set_darcy_order(int darcy_order){
        m_darcy_order = darcy_order;
    }
    
    /** @brief Set the material ID related to Darcy's material */
    void Set_darcy_matid(int darcy_matid){
        m_darcy_ID = darcy_matid;
    }
    
    /** @brief Get the material ID related to Darcy's material */
    int Get_darcy_matid(){
        return m_darcy_ID;
    }
    
    /** @brief Get the polynomial order for the Darcy's approximation  */
    int Get_darcy_order(){
        return m_darcy_order;
    }
    
    /** @brief Set the problem dimension */
    void Set_dimesion(int dimesion){
        m_dimesion = dimesion;
    }
    
    /** @brief Get the problem dimension  */
    int Get_dimesion(){
        return m_dimesion;
    }
    
    /** @brief Set Newton iterations */
    void Set_n_iterations(int n_iterations){
        m_n_iterations = n_iterations;
    }
    
    /** @brief Get Newton iterations  */
    REAL Get_n_iterations(){
        return m_n_iterations;
    }
    
    /** @brief Set residue tolerance */
    void Set_epsilon_res(REAL epsilon_res){
        m_epsilon_res = epsilon_res;
    }
    
    /** @brief Get residue tolerance  */
    REAL Get_epsilon_res(){
        return m_epsilon_res;
    }
    
    /** @brief Set correction tolerance */
    void Set_epsilon_cor(REAL epsilon_cor){
        m_epsilon_cor = epsilon_cor;
    }
    
    /** @brief Get correction tolerance  */
    REAL Get_epsilon_cor(){
        return m_epsilon_cor;
    }
    
    /** @brief Get number of threads being used  */
    int Get_n_threads(){
        return m_n_threads;
    }
    
    /** @brief Set number of threads being used */
    void Set_n_threads(int n_threads){
        m_n_threads = n_threads;
    }
    
    /** @brief Get name for the Gmsh geometry file being used  */
    std::string Get_geometry_file(){
        return m_geometry_file;
    }
    
    /** @brief Set name for the Gmsh geometry file being used */
    void Set_n_threads(std::string geometry_file){
        m_geometry_file = geometry_file;
    }
    
    /** @brief Get name for the vtk files being postprocessed  */
    std::string Get_vtk_file(){
        return m_vtk_file;
    }
    
    /** @brief Set name for the vtk files being postprocessed */
    void Set_vtk_file(std::string vtk_file){
        m_vtk_file = vtk_file;
    }
    
    /** @brief Get number of vtk resolution during postprocessing  */
    int Get_vtk_resolution(){
        return m_vtk_resolution;
    }
    
    /** @brief Set number of vtk resolution during postprocessing */
    void Set_vtk_resolution(int vtk_resolution){
        m_vtk_resolution = vtk_resolution;
    }
    
    /** @brief Get the vector that storages only volumetric material identifiers (higher dimension elements)  */
    std::vector<int> & Get_volumetric_material_id(){
        return m_volumetric_material_id;
    }
    
    /** @brief Set the vector that storages only volumetric material identifiers (higher dimension elements) */
    void Set_volumetric_material_id(std::vector<int> & volumetric_material_id){
        m_volumetric_material_id = volumetric_material_id;
    }

};

#endif /* TPZSimulationData_h */
