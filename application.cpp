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

#include "dist.h"
#include "graph.h"
#include "json.hpp"
using json = nlohmann::json;
using namespace std;

double INF = numeric_limits<double>::max();

struct DistanceCompare {
    bool operator()(const pair<double, long long>& lhs,
                    const pair<double, long long>& rhs) const {
        return lhs.first > rhs.first;
    }
};


void buildGraph(istream &input, graph<long long, double> &g,
                vector<BuildingInfo> &buildings,
                unordered_map<long long, Coordinates> &coords) {
  json jsonData;
  input >> jsonData;

  unordered_map<long long, Coordinates> allPoints;

  
  if (jsonData.contains("waypoints") && jsonData["waypoints"].is_array()) {
    for (const auto& entry : jsonData["waypoints"]) {
      long long nid;
      double latitude, longitude;

      if (entry.is_array() && entry.size() == 3) {
        nid = entry[0];
        latitude = entry[1];
        longitude = entry[2];
      } else if (entry.is_object()) {
        nid = entry["id"];
        latitude = entry["lat"];
        longitude = entry["lon"];
      } else {
        continue;
      }

      allPoints[nid] = Coordinates(latitude, longitude);
      g.addVertex(nid);
    }
  }

  
  if (jsonData.contains("buildings") && jsonData["buildings"].is_array()) {
    for (const auto& bldg : jsonData["buildings"]) {
      if (!bldg.is_object() || !bldg.contains("id") ||
          !bldg.contains("name") || !bldg.contains("abbr") ||
          !bldg.contains("lat") || !bldg.contains("lon")) {
        continue;
      }

      long long bid = bldg["id"];
      string name = bldg["name"];
      string shortname = bldg["abbr"];
      double lat = bldg["lat"];
      double lon = bldg["lon"];
      Coordinates bCoord(lat, lon);

      buildings.emplace_back(bid, bCoord, name, shortname);
      allPoints[bid] = bCoord;
      g.addVertex(bid);
    }
  }

  
  if (jsonData.contains("footways") && jsonData["footways"].is_array()) {
    for (const auto& fw : jsonData["footways"]) {
      vector<long long> path;

      if (fw.is_array()) {
        for (const auto& pid : fw) {
          path.push_back(pid);
        }
      } else if (fw.is_object() && fw.contains("nodes")) {
        for (const auto& pid : fw["nodes"]) {
          path.push_back(pid);
        }
      } else {
        continue;
      }

      for (size_t i = 0; i + 1 < path.size(); ++i) {
        long long from = path[i];
        long long to = path[i + 1];

        if (allPoints.count(from) && allPoints.count(to)) {
          double d = distBetween2Points(allPoints[from], allPoints[to]);
          g.addEdge(from, to, d);
          g.addEdge(to, from, d);
        }
      }
    }
  }

  
  const double LINK_RANGE = 0.036;
  set<long long> linkablePoints;

  for (const auto& [pid, loc] : allPoints) {
    linkablePoints.insert(pid);
  }

  for (const auto& b : buildings) {
    linkablePoints.erase(b.id);
  }

  for (const auto& b : buildings) {
    for (long long otherId : linkablePoints) {
      double d = distBetween2Points(b.location, allPoints[otherId]);
      if (d <= LINK_RANGE) {
        g.addEdge(b.id, otherId, d);
        g.addEdge(otherId, b.id, d);
      }
    }
  }

  coords.clear();
  for (const auto& [id, coord] : allPoints) {
    coords[id] = coord;
  }
  for (const auto& b : buildings) {
    coords.erase(b.id);
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

vector<long long> dijkstra(const graph<long long, double>& G, long long start,
                           long long target,
                           const set<long long>& ignoreNodes) {
  if (start == target) {
    return {start};
  }

  priority_queue<pair<double, long long>,
                 vector<pair<double, long long>>,
                 DistanceCompare> openSet;

  unordered_map<long long, double> bestDist;
  unordered_map<long long, long long> parent;

  vector<long long> vertices = G.getVertices();
  for (const auto& node : vertices) {
    bestDist[node] = INF;
  }
  bestDist[start] = 0.0;

  openSet.push({0.0, start});

  while (!openSet.empty()) {
    auto current = openSet.top();
    openSet.pop();

    double curDist = current.first;
    long long curNode = current.second;

    if (curDist > bestDist[curNode]) {
      continue;
    }

    if (curNode == target) {
      break;
    }

    set<long long> adjacents = G.neighbors(curNode);
    for (auto neighborIt = adjacents.begin(); neighborIt != adjacents.end(); ++neighborIt) {
      long long neighbor = *neighborIt;

      if (ignoreNodes.count(neighbor) && neighbor != start && neighbor != target) {
        continue;
      }

      double edgeWeight = 0.0;
      if (!G.getWeight(curNode, neighbor, edgeWeight)) {
        continue;
      }

      double newDist = bestDist[curNode] + edgeWeight;
      if (newDist < bestDist[neighbor]) {
        bestDist[neighbor] = newDist;
        parent[neighbor] = curNode;
        openSet.push({newDist, neighbor});
      }
    }
  }

  if (bestDist[target] == INF) {
    return {};  // No path
  }

  vector<long long> finalPath;
  long long crawl = target;
  while (crawl != start) {
    finalPath.push_back(crawl);
    crawl = parent[crawl];
  }
  finalPath.push_back(start);

  reverse(finalPath.begin(), finalPath.end());
  return finalPath;
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
