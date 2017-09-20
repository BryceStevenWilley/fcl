/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include "fcl/narrowphase/distance.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "test_fcl_utility.h"
#include "fcl_resources/config.h"

using namespace fcl;

bool verbose = false;

//==============================================================================
template <typename S>
DistanceResult<S> test_distance_general(GJKSolverType solver_type,
        CollisionGeometry<S> &s1,
        CollisionGeometry<S> &s2,
        S expected_dist,
        Vector3<S> translation)
{
  Transform3<S> tf1{Transform3<S>::Identity()};
  Transform3<S> tf2{Transform3<S>::Identity()};

  DistanceRequest<S> request;
  request.enable_signed_distance = true;
  request.enable_nearest_points = true;
  request.gjk_solver_type = solver_type;

  DistanceResult<S> result;
  result.min_distance = -1;

  double res{0.0};

  // Expecting distance to be 10
  result.clear();
  tf2.translation() = translation;
  res = distance(&s1, tf1, &s2, tf2, request, result);
  EXPECT_NE(res, 0.0);
  EXPECT_NEAR(result.min_distance, expected_dist, 1.5e-1);
  // TODO(JS): The negative distance computation using libccd requires
  // unnecessarily high error tolerance.
  return result;
}

template <typename S>
void test_distance_spheresphere(GJKSolverType solver_type)
{
    Sphere<S> s1{20};
    Sphere<S> s2{10};
    auto result = test_distance_general<S>(solver_type, s1, s2, 10, Vector3<S>(40, 0, 0));
    EXPECT_TRUE(result.nearest_points[0].isApprox(Vector3<S>(20, 0, 0)));
    EXPECT_TRUE(result.nearest_points[1].isApprox(Vector3<S>(-10, 0, 0)));
    result = test_distance_general<S>(solver_type, s1, s2, -5, Vector3<S>(25, 0, 0));
    // TODO(JS): Only GST_LIBCCD can compute the pair of nearest points on the
    // surface of the spheres.
    if (solver_type == GST_LIBCCD)
    {
      EXPECT_TRUE(result.nearest_points[0].isApprox(Vector3<S>(20, 0, 0)));
      EXPECT_TRUE(result.nearest_points[1].isApprox(Vector3<S>(-10, 0, 0)));
    }
}

template <typename S>
BVHModel<OBBRSS<S>> *mesh_cube(S length)
{
    //vertices.push_back(Vector3<S>(static_cast<S>(0), static_cast<S>(0), static_cast<S>(0)));
    //vertices.push_back(Vector3<S>(static_cast<S>(length), static_cast<S>(0), static_cast<S>(0)));
    //vertices.push_back(Vector3<S>(static_cast<S>(0), static_cast<S>(length), static_cast<S>(0)));
    //vertices.push_back(Vector3<S>(static_cast<S>(0), static_cast<S>(0), static_cast<S>(length)));
    //triangles.push_back(Triangle(0, 1, 2));
    //triangles.push_back(Triangle(0, 2, 3));
    //triangles.push_back(Triangle(0, 3, 1));
    //triangles.push_back(Triangle(2, 1, 3));

    // Make the first cube.
    std::vector<Vector3<S>> vertices;
    std::vector<Triangle> triangles;
    vertices.push_back(Vector3<S>(static_cast<S>(0), static_cast<S>(0), static_cast<S>(0)));
    vertices.push_back(Vector3<S>(static_cast<S>(length), static_cast<S>(0), static_cast<S>(0)));
    vertices.push_back(Vector3<S>(static_cast<S>(0), static_cast<S>(length), static_cast<S>(0)));
    vertices.push_back(Vector3<S>(static_cast<S>(0), static_cast<S>(0), static_cast<S>(length)));
    vertices.push_back(Vector3<S>(static_cast<S>(length), static_cast<S>(length), static_cast<S>(0)));
    vertices.push_back(Vector3<S>(static_cast<S>(0), static_cast<S>(length), static_cast<S>(length)));
    vertices.push_back(Vector3<S>(static_cast<S>(length), static_cast<S>(0), static_cast<S>(length)));
    vertices.push_back(Vector3<S>(static_cast<S>(length), static_cast<S>(length), static_cast<S>(length)));
    // Bottom
    triangles.push_back(Triangle(0, 1, 2));
    triangles.push_back(Triangle(1, 4, 2));
    // Front
    triangles.push_back(Triangle(0, 2, 3));
    triangles.push_back(Triangle(3, 2, 5));
    // Left
    triangles.push_back(Triangle(0, 3, 1));
    triangles.push_back(Triangle(3, 6, 1));
    // Top
    triangles.push_back(Triangle(6, 3, 5));
    triangles.push_back(Triangle(6, 5, 7));
    // Right
    triangles.push_back(Triangle(5, 2, 4));
    triangles.push_back(Triangle(7, 5, 4));
    // Back
    triangles.push_back(Triangle(6, 7, 4));
    triangles.push_back(Triangle(6, 4, 1));

    typedef BVHModel<OBBRSS<S>> Model;
    Model *model = new Model();
    //std::shared_ptr<BVHModel<OBBRSS<S> > > model(new BVHModel<OBBRSS<S> >);
    model->beginModel();
    model->addSubModel(vertices, triangles);
    model->endModel();
    return model;
}

template <typename S>
void test_distance_cubecube(GJKSolverType solver_type)
{
    typedef BVHModel<OBBRSS<S>> Model;
    Model *c1 = mesh_cube<double>(20);
    Model *c2 = mesh_cube<double>(10);
    test_distance_general<S>(solver_type, *c1, *c2, 20, Vector3<S>(40, 0, 0));
    test_distance_general<S>(solver_type, *c1, *c2, 5, Vector3<S>(25, 0, 0));
    test_distance_general<S>(solver_type, *c1, *c2, -5, Vector3<S>(15, 0, 0));
}

//==============================================================================
GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_sphere)
{
  test_distance_spheresphere<double>(GST_LIBCCD);
  test_distance_spheresphere<double>(GST_INDEP);
}

GTEST_TEST(FCL_NEGATIVEDISTANCE, cube_cube)
{
    test_distance_cubecube<double>(GST_LIBCCD);
    //test_distance_cubecube<double>(GST_INDEP);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
