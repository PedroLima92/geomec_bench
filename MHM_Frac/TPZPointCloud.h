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

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/boost/iterator/counting_iterator.hpp>
#include <utility>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3                                     CGAL_Point;
typedef CGAL::Search_traits_3<Kernel>                       Traits_base;
typedef boost::const_associative_property_map
            <std::map<std::int64_t,CGAL_Point> >            point_map;

typedef CGAL::Search_traits_adapter
            <int64_t, point_map, Traits_base>               Traits;
typedef CGAL::Orthogonal_k_neighbor_search<Traits>          K_neighbor_search;
typedef K_neighbor_search::Tree                             Tree;
typedef Tree::Splitter                                      Splitter;
typedef K_neighbor_search::Distance                         Distance;


class TPZPointCloud
{
    
public:
    
    std::map<std::int64_t,CGAL_Point> point_cloud;

    REAL delxmin = 0.1;
    
    TPZManVector<REAL,3> x0;
    
    TPZGeoMesh *gmesh = 0;
    
    //Uses CGAL library to find closest point
    inline int64_t FindClosePoint(const TPZManVector<REAL,3> &point);

    //Why are there two different methods of inserting/adding new points?
    void AddPoint(int64_t index, TPZManVector<REAL,3> &point);
    
    REAL Dist(TPZVec<REAL> &x1, TPZVec<REAL> &x2);

    void InsertGmesh(TPZGeoMesh *gmesh);
    
    void CGALToCoord(const CGAL_Point &a, TPZManVector<REAL, 3> &x)
    {
        x[0] = x0[0] + a[0];
        x[1] = x0[1] + a[1];
        x[2] = x0[2] + a[2];
    }
    void CoordToCGAL(const TPZManVector<REAL, 3> &a, CGAL_Point &x)
    {
        x[0] = x0[0] + a[0];
        x[1] = x0[1] + a[1];
        x[2] = x0[2] + a[2];
    }
public:
    
    TPZPointCloud() : x0(3,0.) {}
    
    TPZPointCloud(const TPZPointCloud &copy) 
    :   point_cloud(copy.point_cloud),
        delxmin(copy.delxmin), 
        x0(3,0.), 
        gmesh(copy.gmesh)
    {}
    
    
    TPZPointCloud &operator=(const TPZPointCloud &copy)
    {
        point_cloud = copy.point_cloud;
        delxmin = copy.delxmin;
        x0 = copy.x0;
        gmesh = copy.gmesh;
        return *this;
    }
    
    ~TPZPointCloud()
    {
        //Erase search tree?
    }
    
    
    //Why are there two different methods of inserting/adding new points?
    inline int64_t InsertPoint(TPZVec<REAL> &newpoint)
    {
        int64_t closest_index = FindClosePoint(newpoint);
        TPZManVector<REAL, 3> closest;
        CGALToCoord(point_cloud[closest_index], closest);
        if (Dist(newpoint, closest) <= delxmin)
        {
            return closest_index;
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
