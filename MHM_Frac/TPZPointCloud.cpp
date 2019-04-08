//
//  TPZPointCloud.cpp
//  Benchmark0a
//
//  Created by Philippe Devloo on 22/06/18.
//

#include "TPZPointCloud.h"
#include "pzvec_extras.h"

typedef CGAL::AABB_tree<Traits> Tree;

void TPZPointCloud::SurroundingBox(const TPZVec<REAL> &x, TPZStack<pointloc> &box)
{
    TPZManVector<REAL,3> xloc(x);
    xloc -= x0;
    TPZManVector<REAL,3> xlocreconstructed(3), xlocreconstructedplus(3);
    TPZManVector<int64_t,3> ixlocmin(3), ixlocmax(3);
    for (int i=0; i<3; i++) {
        ixlocmin[i] = (int64_t)(xloc[i]/delxmin);
        xlocreconstructed[i] = ixlocmin[i]*delxmin;
        xlocreconstructedplus[i] = (ixlocmin[i]+1)*delxmin;
    }
#ifdef PZDEBUG
    {
        for(int i=0; i<3; i++)
        {
            if(xlocreconstructed[i] > xloc[i] || xlocreconstructedplus[i] < xloc[i])
            {
                DebugStop();
            }
        }
    }
#endif

    for (int i=0; i<3; i++) {
        if (fabs(xloc[i]-xlocreconstructed[i]) < tol) {
            ixlocmax[i] = ixlocmin[i];
        }
        else if(fabs(xloc[i]-xlocreconstructedplus[i]) < tol)
        {
            ixlocmin[i]++;
            ixlocmax[i] = ixlocmin[i];
        }
        else
        {
            ixlocmax[i] = ixlocmin[i]+1;
        }
    }
    
    for (int64_t i=ixlocmin[0]; i<= ixlocmax[0]; i++) {
        for (int64_t j=ixlocmin[1]; j<= ixlocmax[1]; j++) {
            for (int64_t k=ixlocmin[2]; k<= ixlocmax[2]; k++) {
                pointloc a(i,j,k);
                box.push_back(a);
            }
        }
    }
}

void TPZPointCloud::AddPoint(int64_t index, TPZVec<REAL> &point)
{

    TPZStack<pointloc> box;
    SurroundingBox(point, box);
    int nb = box.size();
    int numinserted = 0;
    for (int i=0; i<nb; i++) {
        if (Points.find(box[i]) != Points.end()) {
            int64_t index = Points[box[i]];
            TPZManVector<REAL,3> xref(3), xbox(3);
            gmesh->NodeVec()[index].GetCoordinates(xref);
            PointlocToCo(box[i], xbox);
            if (Dist(point, xbox) < Dist(xref, xbox)) {
                Points[box[i]] = index;
                numinserted++;
            }
        }
        else
        {
            Points[box[i]] = index;
            numinserted++;
        }
    }
    if (numinserted == 0) {
        DebugStop();
    }
}

void TPZPointCloud::InsertGmesh(TPZGeoMesh *geomesh)
{
    gmesh = geomesh;
    int64_t nnodes = geomesh->NNodes();
    for (int64_t in=0; in<nnodes; in++) {
        TPZManVector<REAL,3> co(3);
        geomesh->NodeVec()[in].GetCoordinates(co);
        AddPoint(in, co);
    }
    //Construction of CGAL AABB search tree for efficient distance computation.
    Tree tree(Points.begin(), Points.end());
    tree.accelerate_distance_queries();

    /*The idea here is that the inserted mesh should be efficiently accessible for 
    when inserting new points during fracture simulation.
    Does that mean that the new points created during fracture insertion/propagation 
    would not be accounted for within the CGAL "AABB tree"?
    If that's the case, than the construction of the tree should be processed inside
    the FindClosePoint method. My impression is that this would have the "tree" being
    reconstructed multiple times and lowering the code's efficiency*/ 
}



REAL TPZPointCloud::Dist(CGAL_Point &x1, CGAL_Point &x2)
{
    REAL dist = 0.;
    for (int i=0; i<3; i++) {
        REAL del = x1[i]-x2[i];
        dist = std::max(dist,std::abs(del));
    }
    return dist;
}

inline CGAL_Point FindClosePoint(const TPZVec<REAL> &point)
{
    // Tree tree(Points.begin(), Points.end());
    // tree.accelerate_distance_queries();

    CGAL_Point newpoint(point[0], point[1], point[2]);
    CGAL_Point closest = tree.closest_point(newpoint)
    if (Dist(newpoint, closest) > delxmin)
    {
        return newpoint;
    }
    return closest;
}