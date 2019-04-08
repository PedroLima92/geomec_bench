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

typedef std::tuple<int,int,int> pointloc;

inline bool operator<(const pointloc &a, const pointloc &b)
{
    return
    std::get<0>(a) < std::get<0>(b) ||
    (std::get<0>(a) == std::get<0>(b) && std::get<1>(a) < std::get<1>(b)) ||
    (std::get<0>(a) == std::get<0>(b) && std::get<1>(a) == std::get<1>(b) && std::get<2>(a) < std::get<2>(b));
}

class TPZPointCloud
{
    
public:
    
    std::list<CGAL_Points> Points;
    
    REAL delxmin = 0.1;
    
    REAL tol = 1.e-3;
    
    TPZManVector<REAL,3> x0;
    
    TPZGeoMesh *gmesh = 0;
    
    void SurroundingBox(const TPZVec<REAL> &x, TPZStack<pointloc> &box);
    
    inline int64_t FindClosePoint(const TPZVec<REAL> &point);

    void AddPoint(int64_t index, TPZVec<REAL> &point);
    
    REAL Dist(TPZVec<REAL> &x1, TPZVec<REAL> &x2);
    
    void PointlocToCo(const pointloc &a, TPZVec<REAL> &x)
    {
        x[0] = x0[0]+delxmin*std::get<0>(a);
        x[1] = x0[1]+delxmin*std::get<1>(a);
        x[2] = x0[2]+delxmin*std::get<2>(a);
    }
    
    void PointlocToCGAL(const CGAL_Point &a, TPZVec<REAL> &x)
    {
        x[0] = x0[0] + a[0];
        x[1] = x0[1] + a[1];
        x[2] = x0[2] + a[2];
    }

public:
    
    TPZPointCloud() : x0(3,0.) {}
    
    TPZPointCloud(const TPZPointCloud &copy) : Points(copy.Points), delxmin(copy.delxmin), tol(copy.tol), x0(3,0.), gmesh(copy.gmesh)
    {
        
    }
    
    
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
    
    void InsertGmesh(TPZGeoMesh *gmesh);
    
    inline int64_t InsertPoint(TPZVec<REAL> &newpoint)
    {
        int64_t existing = FindClosePoint(newpoint);
        if (existing >= 0) {
            return existing;
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
