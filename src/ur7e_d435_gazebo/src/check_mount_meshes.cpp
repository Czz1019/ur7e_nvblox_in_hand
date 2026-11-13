// Read transformed camera/arm geometry from stdin and check it with Gazebo + FCL.
#include <gazebo/common/Mesh.hh>
#include <gazebo/common/MeshManager.hh>
#include <fcl/fcl.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

struct Shape {
  std::string group, name;
  std::shared_ptr<fcl::CollisionObjectd> object;
};

int main() {
  try {
    std::vector<Shape> shapes;
    std::string group, name, kind;
    while (std::cin >> group >> name >> kind) {
      std::shared_ptr<fcl::CollisionGeometryd> geometry;
      if (kind == "mesh") {
        std::string path;
        double sx, sy, sz;
        std::cin >> std::quoted(path) >> sx >> sy >> sz;
        const auto *mesh = gazebo::common::MeshManager::Instance()->Load(path);
        if (!mesh) throw std::runtime_error("Cannot load " + path);
        std::vector<fcl::Vector3d> vertices;
        std::vector<fcl::Triangle> triangles;
        for (unsigned i = 0; i < mesh->GetSubMeshCount(); ++i) {
          const auto *sub = mesh->GetSubMesh(i);
          const auto offset = vertices.size();
          for (unsigned j = 0; j < sub->GetVertexCount(); ++j) {
            const auto p = sub->Vertex(j);
            vertices.emplace_back(p.X()*sx, p.Y()*sy, p.Z()*sz);
          }
          for (unsigned j = 0; j + 2 < sub->GetIndexCount(); j += 3)
            triangles.emplace_back(offset + sub->GetIndex(j),
                                   offset + sub->GetIndex(j+1), offset + sub->GetIndex(j+2));
        }
        if (triangles.empty()) throw std::runtime_error("No triangles in " + path);
        auto model = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
        model->beginModel();
        model->addSubModel(vertices, triangles);
        model->endModel();
        geometry = model;
      } else if (kind == "box") {
        double x, y, z;
        std::cin >> x >> y >> z;
        geometry = std::make_shared<fcl::Boxd>(x, y, z);
      } else {
        throw std::runtime_error("Unsupported geometry: " + kind);
      }
      fcl::Transform3d pose = fcl::Transform3d::Identity();
      for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 4; ++col) std::cin >> pose.matrix()(row, col);
      if (!std::cin) throw std::runtime_error("Incomplete geometry input");
      shapes.push_back({group, name, std::make_shared<fcl::CollisionObjectd>(geometry, pose)});
    }
    std::cout << std::setprecision(10) << "{\"pairs\":[";
    bool first = true, collision = false;
    for (const auto &camera : shapes) {
      if (camera.group != "camera") continue;
      for (const auto &arm : shapes) {
        if (arm.group != "arm") continue;
        fcl::CollisionResultd contact;
        fcl::collide(camera.object.get(), arm.object.get(), fcl::CollisionRequestd(), contact);
        fcl::DistanceResultd distance;
        fcl::distance(camera.object.get(), arm.object.get(), fcl::DistanceRequestd(), distance);
        collision = collision || contact.isCollision();
        if (!first) std::cout << ',';
        first = false;
        std::cout << "{\"camera\":\"" << camera.name << "\",\"arm\":\"" << arm.name
                  << "\",\"collision\":" << (contact.isCollision() ? "true" : "false")
                  << ",\"distance_m\":" << std::max(0.0, distance.min_distance) << '}';
      }
    }
    if (first) throw std::runtime_error("No camera/arm pairs provided");
    std::cout << "],\"collision\":" << (collision ? "true" : "false") << "}\n";
    return 0;  // A collision is a diagnostic result, not a failed invocation.
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
