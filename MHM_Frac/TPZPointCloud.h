//
//  TPZPointCloud.hpp
//  Benchmark0a
//
//  Created by Philippe Devloo on 22/06/18.
//

#ifndef TPZPointCloud_hpp
#define TPZPointCloud_hpp

#include <stdio.h>
#include <tuple>
#include <map>

#include "pzreal.h"
#include "pzgmesh.h"
#include "pzvec_extras.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_segment_primitive.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3 CGAL_Point;

typedef std::list<CGAL_Point>::iterator Iterator;
typedef CGAL::AABB_segment_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> Traits;


class TPZPointCloud
{
    
public:
    
    std::map<CGAL_Point, int64_t> Points;
    //std::map<CGAL_Point, int64_t> CGALPoints;

    REAL delxmin = 0.1;
    
    REAL tol = 1.e-3;
    
    TPZManVector<REAL,3> x0;
    
    TPZGeoMesh *gmesh = 0;
    
    //Uses CGAL library to find closest point
    inline int64_t FindClosePoint(const TPZVec<REAL> &point);

    //Why are there two different methods of inserting/adding new points?
    void AddPoint(int64_t index, TPZVec<REAL> &point);
    
    REAL Dist(TPZVec<REAL> &x1, TPZVec<REAL> &x2);

    void InsertGmesh(TPZGeoMesh *gmesh);
    
    void CGALToCoord(const CGAL_Point &a, TPZVec<REAL> &x)
    {
        x[0] = x0[0] + a[0];
        x[1] = x0[1] + a[1];
        x[2] = x0[2] + a[2];
    }

public:
    
    TPZPointCloud() : x0(3,0.) {}
    
    TPZPointCloud(const TPZPointCloud &copy) 
    :   Points(copy.Points),
        delxmin(copy.delxmin), 
        tol(copy.tol), 
        x0(3,0.), 
        gmesh(copy.gmesh)
    {}
    
    
    TPZPointCloud &operator=(const TPZPointCloud &copy)
    {
        Points = copy.Points;
        delxmin = copy.delxmin;
        tol = copy.tol;
        x0 = copy.x0;
        gmesh = copy.gmesh;
        return *this;
    }
    
    ~TPZPointCloud()
    {
        
    }
    
    
    //Why are there two different methods of inserting/adding new points?
    inline int64_t InsertPoint(TPZVec<REAL> &newpoint)
    {
        int63_t closest_index = FindClosePoint(newpoint);
        TPZVec<REAL> closest;
        CGALToCoord(Points[closest_index], closest);
        if (Dist(newpoint, closest) <= delxmin)
        {
            return closest_index 
        }
        else
        {
            int64_t index = gmesh->NodeVec().AllocateNewElement();
            gmesh->NodeVec()[index].Initialize(newpoint, *gmesh);
            return index;
        }
    }
    
};
#endif /* TPZPointCloud_hpp */
