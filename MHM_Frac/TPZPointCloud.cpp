//
//  TPZPointCloud.cpp
//  Benchmark0a
//
//  Created by Philippe Devloo on 22/06/18.
//

#include "TPZPointCloud.h"
#include "pzvec_extras.h"

typedef CGAL::AABB_tree<Traits> Tree;


void TPZPointCloud::AddPoint(int64_t index, TPZVec<REAL> &newpoint)
{
    
    int63_t closest_index = FindClosePoint(newpoint);
    TPZVec<REAL> closest;
    CGALToCoord(Points[closest_index], closest);
    if (Dist(newpoint, closest) > delxmin)
    {
        Points[index](newpoint[0], newpoint[1], newpoint[2]);
    }
}

void TPZPointCloud::InsertGmesh(TPZGeoMesh *geomesh)
{
    gmesh = geomesh;
    int64_t nnodes = geomesh->NNodes();
    for (int64_t index=0; index<nnodes; index++) {
        TPZManVector<REAL,3> co(3);
        geomesh->NodeVec()[index].GetCoordinates(co);
        AddPoint(index, co);
    }
    
    // Tree tree(Points.begin(), Points.end());
    // tree.accelerate_distance_queries();
}



REAL TPZPointCloud::Dist(TPZVec<REAL> &x1, TPZVec<REAL> &x2)
{
    REAL dist = 0.;
    for (int i = 0; i < 3; i++)
    {
        REAL del = x1[i] - x2[i];
        dist = std::max(dist, std::abs(del));
    }
    return dist;
}

inline int64_t FindClosePoint(const TPZVec<REAL> &point)
{
    //Construction of CGAL AABB search tree for efficient distance computation.
    Tree tree(Points.begin(), Points.end());
    tree.accelerate_distance_queries();
    
    /*Should the tree get (re)constructed every time the code has to look for 
    close points?
    My impression is that this would have the "tree" being reconstructed 
    way too many times, which feels quite expensive.
    Isn't there a way to store the already built tree somewhere in the memory
    so that it would only require the addition of new points?*/ 

    
    CGAL_Point newpoint(point[0], point[1], point[2]);
    CGAL_Point closest = tree.closest_point(newpoint);

    return Points[closest];
}