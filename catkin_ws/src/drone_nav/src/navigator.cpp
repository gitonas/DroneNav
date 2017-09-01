#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/LinearMath/Vector3.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class treeRRT {
    void add_edge(size_t,size_t);
  public:
    int noNodes, noEdges;
    vector<Vector3d> nodes;
    vector<pair<size_t,size_t>> edges;
    void add_node(Vector3d);
    bool connect_to(treeRRT);
    // missing: path_to object which is a list of index sequences
    // could also be only the parent in the tree. actually that makes more sense
}

void treeRRT::add_node(Vector3d newNode)
{
  // Add a new node to the objects nodes vector and update noNodes
  size_t is_visible = -1;
  for (size_t i = 0; i < noNodes; i++) {
    if (!is_blocked(newNode,nodes[i])) {
      is_visible = i;
      break;
    }
  }
  if (is_visible >= 0) {
    nodes.push_back(newNode);
    vector<size_t> visibleNodes;
    pair<size_t, double> minEl;
    minEl = make_pair(is_visible,(newNode - nodes[is_visible]).norm());
    for (size_t i = is_visible; i < noNodes; i++) {
      if (!is_blocked(nodes[i])) {
        double dist = (newNode - nodes[i]).norm();
        if (dist < minEl.second) {
          minEl.first = i;
          minEl.second = dist;
        }
      }
    }
    add_edge(minEl.first,(size_t)(++noNodes));
    noEdges++;
  }

}


bool treeRRT::connect_to(treeRRT t)
{
  // Check if t is connectable to this.
  // Either update this to incorporate t or return Path object instead of bool
}

void treeRRT::add_edge(int i, int j)
{
  // Add an edge between nodes i and j, by appending to the edges vector.
  // Update path_to list
}

bool is_blocked(Vector3f<float> A, Vector3f<float> B)
{
  float distance = Vector3f::abs(B-A);
  Vector3f<float> dir = (B-A)/distance;
  Vector3f<float> center = A + distance/2*dir;
  
  // Select points from point cloud between A and B
  
  PointXYZ searchPoint;
  searchPoint->x = center[1];
  searchPoint->y = center[2];
  searchPoint->z = center[3];

  KdTreeFLANN<PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);

  Vector3f<int> pointIdxRadius;
  Vector3f<float> pointsSquaredDistRadiusM;
  float radius = distance/sqrt(2);

  int count = kdtree.radiusSearch(searchPoint, radius,
             pointIdxRadius, pointsSquaredDistRadius);
  

  // Get orthogonal plane vectors
  Vector3f dir_x = dir.cross(Vector3f z(0,0,1));
  //dir_x[0] =  dir[1];
  //dir_x[1] = -dir[0];
  //dir_x[2] =  0;

  Vector3f dir_y = dir.cross(dir_x);
  //dir_y[0] = dir[1]*dir_x[2] - dir_x[1]*dir[2];
  //dir_y[1] = dir[2]*dir_x[0] - dir_x[2]*dir[0];
  //dir_y[2] = dir[0]*dir_x[1] - dir_x[0]*dir[1];

  
  for (size_t i = 0; i < nb->points.size (); ++i)
  {
    PointXYZ nbXYZ = cloud->points[pointIdxRadius[i]];
    Vector3f nb(nbXYZ.x - A[0],nbXYZ.y - A[1],nbXYZ.z - A[2]);

    
    // Calculate longitudinal projection onto line(A,B)
    float longDist = nb.dot(dir)/distance - 0.5;
    float bufferDist;
    if (abs(longDist) > 0.5)
    {
      // If the projection onto line(A,B) lies between A and B...
      // Calculate lateral distance to line(A,B)
      float latDistSq = pow(nb.dot(dir_x),2) + pow(nb.dot(dir_y),2);
      if (latDistSq < ros::param::get("/pars/drone/bufferDist",bufferDist))
      {
        // If lateral distance is closer than the buffer distance...
        // Return that the path is bocked.
        return true;
      }
    }
  }

  return false;
}



int main(int argc, char ** argv)
{
  /* Provide replanning service:
   * - [In]: Obstacle PointCloud
   * - [In]: Waypoint list
   * - [Out]: Waypoint list
   *
   * Functions:
   * - is_blocked(A,B): returns true if there is an obstacle between A and B
   * - 
   */

  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

}
