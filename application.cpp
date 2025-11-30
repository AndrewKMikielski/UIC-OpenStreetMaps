#include "application.h"

#include <iostream>
#include <limits>
#include <map>
#include <queue> // priority_queue
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>

#include "dist.h"
#include "graph.h"
#include "json.hpp"

using json = nlohmann::json;
using namespace std;

double INF = numeric_limits<double>::max();

void buildGraph(istream &input, graph<long long, double> &g,
                vector<BuildingInfo> &buildings,
                unordered_map<long long, Coordinates> &coords) {
  json j;
  input >> j;

    // 1. Parse Nodes
    for (const auto& node : j["waypoints"]) {
        long long id = node["id"];
        double lat = node["lat"];
        double lon = node["lon"];
        Coordinates c(lat, lon);
        coords[id] = c;
        g.addVertex(id);
    }

    // 2. Parse Footways and add edges
    for (const auto& nodes : j["footways"]) {
        for (size_t i = 1; i < nodes.size(); ++i) {
            long long from = nodes[i - 1];
            long long to = nodes[i];
            double distance = distBetween2Points(coords[from], coords[to]);
            g.addEdge(from, to, distance);
            g.addEdge(to, from, distance); // bidirectional
        }
    }

    // 3. Parse Buildings and find closest footway node
    for (const auto& bldg : j["buildings"]) {
        long long id = bldg["id"];
        string name = bldg.value("name", "");
        string abbr = bldg.value("abbr", "");
        double lat = bldg["lat"];
        double lon = bldg["lon"];

        Coordinates loc(lat, lon);
        BuildingInfo info(id, loc, name, abbr);
        buildings.push_back(info);

        // Add to graph
        g.addVertex(id);

        // Find closest footway node(s)
        double minDist = INF;
        vector<long long> closest;

        for (const auto &[nodeId, nodeCoords] : coords) {
          double d = distBetween2Points(loc, nodeCoords);
          if (abs(d - minDist) < 1e-6) {
            closest.push_back(nodeId);
          } else if (d < minDist) {
            minDist = d;
            closest.clear();
            closest.push_back(nodeId);
          }
        }

        for (const auto &[nodeId, nodeCoords] : coords) {
          double d = distBetween2Points(loc, nodeCoords);
          if (d <= 0.036) {
              g.addEdge(id, nodeId, d);
              g.addEdge(nodeId, id, d);
          }
      }
    }
}

BuildingInfo getBuildingInfo(const vector<BuildingInfo> &buildings,
                             const string &query) {
  for (const BuildingInfo &building : buildings) {
    if (building.abbr == query) {
      return building;
    } else if (building.name.find(query) != string::npos) {
      return building;
    }
  }
  BuildingInfo fail;
  fail.id = -1;
  return fail;
}

BuildingInfo getClosestBuilding(const vector<BuildingInfo> &buildings,
                                Coordinates c) {
  double minDestDist = INF;
  BuildingInfo ret = buildings.at(0);
  for (const BuildingInfo &building : buildings) {
    double dist = distBetween2Points(building.location, c);
    if (dist < minDestDist) {
      minDestDist = dist;
      ret = building;
    }
  }
  return ret;
}

// Comparison operator for std::priority_queue
class prioritize {
  public:
   bool operator()(const pair<long long, double>& p1,
                   const pair<long long, double>& p2) const {
     return p1.second > p2.second;
   }
};

// Builds the Path from start to target with a predecessors map
// Returns a vector containing the steps for the path
template <typename T>
vector<T> buildPath(const T &start, const T &target, const unordered_map<T, T> &predecessors){
  vector<T> path;

  path.push_back(target);
  T predecessor = predecessors.at(target);
  while (predecessor != start) {
    path.push_back(predecessor);
    predecessor = predecessors.at(predecessor);
  }
  path.push_back(predecessor);
  // path is saved as target-->start so it needs to be reversed to start-->target
  reverse(path.begin(), path.end());

  return path;
}

vector<long long> dijkstra(const graph<long long, double> &G, long long start,
                           long long target,
                           const set<long long> &ignoreNodes) {
  if (start == target) {
    vector<long long> vec(1, start);
    return vec;
  }

  priority_queue<pair<long long, double>, vector<pair<long long, double>>, prioritize> worklist;
  unordered_map<long long, double> distances;
  unordered_map<long long, long long> predecessors;
  set<long long> seen;
  const double START_DIST = 0.0;  // sort of not needed, only used twice

  // initialize data before starting the worklist algo
  worklist.push({start, START_DIST});
  seen.insert(start);
  distances[start] = START_DIST;

  // Start work
  while (!worklist.empty()) {
    // get the top, save its distance as the current distance so far
    pair<long long, double> curr = worklist.top();
    double currDist = distances[curr.first];
    worklist.pop(); // remove this item from worklist

    // Iterate through the neighbors to add more work to worklist
    for (long long w : G.neighbors(curr.first)) {
      // calculate the total distance through curr to w
      double distTo = 0.0;
      G.getWeight(curr.first, w, distTo);
      double newDist = currDist + distTo;
      if (!seen.contains(w) && (!ignoreNodes.contains(w) || w == start || w == target)) {
        // never before seen node, create data for it
        seen.insert(w);
        distances[w] = newDist;
        predecessors[w] = curr.first;
        worklist.push({w, newDist});
      } else if ((!ignoreNodes.contains(w) || w == start || w == target) && newDist < distances[w]) {
        // only update data for a node when a shorter path is found
        distances[w] = newDist;
        predecessors[w] = curr.first;
        worklist.push({w, newDist});
      }
    }
  }

  // return {} if the target is unreachable
  if (!seen.contains(target)) {
    return {};
  }

  // final steps, building a path from the results of the algo
  vector<long long> path = buildPath(start, target, predecessors);
  return path;
}

double pathLength(const graph<long long, double> &G,
                  const vector<long long> &path) {
  double length = 0.0;
  double weight;
  for (size_t i = 0; i + 1 < path.size(); i++) {
    bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
    if (!res) {
      return -1;
    }
    length += weight;
  }
  return length;
}

void outputPath(const vector<long long> &path) {
  for (size_t i = 0; i < path.size(); i++) {
    cout << path.at(i);
    if (i != path.size() - 1) {
      cout << "->";
    }
  }
  cout << endl;
}

// Honestly this function is just a holdover from an old version of the project
void application(const vector<BuildingInfo> &buildings,
                 const graph<long long, double> &G) {
  string person1Building, person2Building;

  set<long long> buildingNodes;
  for (const auto &building : buildings) {
    buildingNodes.insert(building.id);
  }

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    // Look up buildings by query
    BuildingInfo p1 = getBuildingInfo(buildings, person1Building);
    BuildingInfo p2 = getBuildingInfo(buildings, person2Building);
    Coordinates P1Coords, P2Coords;
    string P1Name, P2Name;

    if (p1.id == -1) {
      cout << "Person 1's building not found" << endl;
    } else if (p2.id == -1) {
      cout << "Person 2's building not found" << endl;
    } else {
      cout << endl;
      cout << "Person 1's point:" << endl;
      cout << " " << p1.name << endl;
      cout << " " << p1.id << endl;
      cout << " (" << p1.location.lat << ", " << p1.location.lon << ")" << endl;
      cout << "Person 2's point:" << endl;
      cout << " " << p2.name << endl;
      cout << " " << p2.id << endl;
      cout << " (" << p2.location.lon << ", " << p2.location.lon << ")" << endl;

      Coordinates centerCoords = centerBetween2Points(p1.location, p2.location);
      BuildingInfo dest = getClosestBuilding(buildings, centerCoords);

      cout << "Destination Building:" << endl;
      cout << " " << dest.name << endl;
      cout << " " << dest.id << endl;
      cout << " (" << dest.location.lat << ", " << dest.location.lon << ")"
           << endl;

      vector<long long> P1Path = dijkstra(G, p1.id, dest.id, buildingNodes);
      vector<long long> P2Path = dijkstra(G, p2.id, dest.id, buildingNodes);

      // This should NEVER happen with how the graph is built
      if (P1Path.empty() || P2Path.empty()) {
        cout << endl;
        cout << "At least one person was unable to reach the destination "
                "building. Is an edge missing?"
             << endl;
        cout << endl;
      } else {
        cout << endl;
        cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P1Path);
        cout << endl;
        cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P2Path);
      }
    }

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}
