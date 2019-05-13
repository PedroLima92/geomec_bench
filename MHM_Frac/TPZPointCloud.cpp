//
//  TPZPointCloud.cpp
//  Benchmark0a
//
//  Created by Philippe Devloo on 22/06/18.
//

#include "TPZPointCloud.h"
#include "pzvec_extras.h"



void TPZPointCloud::AddPoint(int64_t index, TPZManVector<REAL,3> &newpoint)
{
    
    int64_t closest_index = FindClosePoint(newpoint);
    TPZManVector<REAL, 3> closest;
    CGALToCoord(point_cloud[closest_index], closest);
    if (Dist(newpoint, closest) > delxmin)
    {
        point_cloud[index](newpoint[0], newpoint[1], newpoint[2]);
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

inline int64_t FindClosePoint(const TPZManVector<REAL,3> &point)
{
    //Construction of CGAL search tree for efficient distance computation.
    Tree tree(
        boost::counting_iterator<int64_t>(0),
        boost::counting_iterator<int64_t>(point_cloud.size()),
        Splitter(),
        Traits(point_cloud));
    
    /*Should the tree get (re)constructed every time the code has to look for 
    close points?
    My impression is that this would have the "tree" being reconstructed 
    way too many times, which feels quite expensive.
    Isn't there a way to store the already built tree somewhere in the memory
    so that it would only require the addition of new points?*/ 

    Distance Dist_CGAL(point_cloud);
    CGAL_Point newpoint(point[0], point[1], point[2]);
    
    K_neighbor_search closest(tree, newpoint, 1, 0, true, Dist_CGAL);
    int64_t closest_index = closest.begin()->first;
    //K_neighbor_search::iterator it = closest.begin();
    //return it->first;
    return closest_index;
}