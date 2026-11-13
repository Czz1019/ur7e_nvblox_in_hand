// Offline collision oracle for scan design. No ROS or Gazebo server connection.
// Uses the same Gazebo mesh loader and FCL as check_mount_meshes.cpp.
#include <gazebo/common/Mesh.hh>
#include <gazebo/common/MeshManager.hh>
#include <fcl/fcl.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <limits>
#include <array>

struct Body {
  int owner;
  std::shared_ptr<fcl::CollisionObjectd> object;
  std::vector<fcl::Vector3d> vertices;
  std::array<double,6> bounds;
  bool payload;
};
fcl::Transform3d readPose() {
  auto pose = fcl::Transform3d::Identity();
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 4; ++c) std::cin >> pose.matrix()(r, c);
  if (!std::cin) throw std::runtime_error("Incomplete pose");
  return pose;
}
Body readBody() {
  int owner; std::string kind; std::cin >> owner >> kind;
  std::shared_ptr<fcl::CollisionGeometryd> geometry;
  std::vector<fcl::Vector3d> floor_vertices;
  if (kind == "mesh") {
    std::string path; double sx, sy, sz;
    std::cin >> std::quoted(path) >> sx >> sy >> sz;
    const auto *mesh = gazebo::common::MeshManager::Instance()->Load(path);
    if (!mesh) throw std::runtime_error("Cannot load mesh: " + path);
    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> triangles;
    for (unsigned i = 0; i < mesh->GetSubMeshCount(); ++i) {
      auto sub = mesh->GetSubMesh(i); auto offset = vertices.size();
      for (unsigned j = 0; j < sub->GetVertexCount(); ++j) {
        auto p = sub->Vertex(j); vertices.emplace_back(p.X()*sx, p.Y()*sy, p.Z()*sz);
      }
      for (unsigned j = 0; j+2 < sub->GetIndexCount(); j+=3)
        triangles.emplace_back(offset+sub->GetIndex(j), offset+sub->GetIndex(j+1), offset+sub->GetIndex(j+2));
    }
    if (triangles.empty()) throw std::runtime_error("Empty collision mesh");
    auto bvh = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
    bvh->beginModel(); bvh->addSubModel(vertices, triangles); bvh->endModel(); geometry = bvh;
    floor_vertices = std::move(vertices);
  } else if (kind == "box") {
    double x,y,z; std::cin >> x >> y >> z; geometry = std::make_shared<fcl::Boxd>(x,y,z);
    for (int i : {-1,1}) for (int j : {-1,1}) for (int k : {-1,1})
      floor_vertices.emplace_back(i*x/2,j*y/2,k*z/2);
  } else if (kind == "sphere") {
    double radius; std::cin >> radius; geometry = std::make_shared<fcl::Sphered>(radius);
  } else if (kind == "floor") {
    geometry = std::make_shared<fcl::Halfspaced>(0.,0.,1.,0.);
  } else throw std::runtime_error("Unsupported collision kind");
  auto pose = readPose();
  std::array<double,6> bounds;
  for (auto &value : bounds) std::cin >> value;
  bool payload; std::cin >> payload;
  return {owner, std::make_shared<fcl::CollisionObjectd>(geometry, pose), floor_vertices, bounds, payload};
}
double gap(const Body &a, const Body &b) {
  fcl::CollisionResultd contact;
  fcl::collide(a.object.get(), b.object.get(), fcl::CollisionRequestd(), contact);
  if (contact.isCollision()) return -1.;
  fcl::DistanceResultd result;
  fcl::distance(a.object.get(), b.object.get(), fcl::DistanceRequestd(), result);
  return result.min_distance;
}
int main() {
  try {
    int nr, ne; double radius_bound; std::cin >> nr >> ne >> radius_bound;
    if (nr < 1 || nr > 100 || ne < 1 || ne > 100) throw std::runtime_error("Invalid body counts");
    std::vector<Body> robot, world;
    for (int i=0;i<nr;++i) robot.push_back(readBody());
    for (int i=0;i<ne;++i) world.push_back(readBody());
    std::cout << std::setprecision(12) << "READY\n" << std::flush;
    int count;
    while (std::cin >> count) {
      if (count < 1 || count > 128) throw std::runtime_error("Invalid batch size");
      for (int k=0;k<count;++k) {
        std::array<double,6> delta;
        for (auto &value : delta) std::cin >> value;
        for (auto &body : robot) { body.object->setTransform(readPose()); body.object->computeAABB(); }
        double environment = std::numeric_limits<double>::infinity(), self = environment;
        double env_slack = environment, self_slack = environment;
        for (int i=0;i<nr;++i) {
          double movement = 0;
          for (int joint=0;joint<robot[i].owner;++joint) movement += robot[i].bounds[joint]*std::abs(delta[joint]);
          for (const auto &obstacle : world) {
            if (obstacle.owner == -2) {
              if (robot[i].owner != 0) // Fixed base/floor contact is intentional.
                for (const auto &v : robot[i].vertices) {
                  double distance = (robot[i].object->getTransform()*v).z();
                  environment = std::min(environment, distance);
                  env_slack = std::min(env_slack, distance-movement);
                }
              continue;
            }
            double distance = gap(robot[i], obstacle);
            environment = std::min(environment, distance);
            env_slack = std::min(env_slack, distance-movement);
          }
          for (int j=0;j<i;++j)
            if (std::abs(robot[i].owner-robot[j].owner)>1 ||
                (robot[i].payload && robot[j].owner < robot[i].owner) ||
                (robot[j].payload && robot[i].owner < robot[j].owner)) {
              double distance = gap(robot[i], robot[j]), relative = 0;
              // Common-ancestor joint motion is a rigid transform of both
              // bodies and cancels in their relative distance.
              for (int joint=std::min(robot[i].owner,robot[j].owner);
                   joint<std::max(robot[i].owner,robot[j].owner);++joint)
                relative += (robot[i].owner>robot[j].owner ? robot[i].bounds[joint] : robot[j].bounds[joint])*std::abs(delta[joint]);
              self = std::min(self, distance);
              self_slack = std::min(self_slack, distance-relative);
            }
        }
        std::cout << environment << ' ' << self << ' ' << env_slack << ' ' << self_slack << '\n';
      }
      std::cout << std::flush;
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
