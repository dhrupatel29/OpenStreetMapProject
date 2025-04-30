#pragma once

#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

using namespace std;

/// @brief Simple directed graph using an adjacency list.
/// @tparam VertexT vertex type
/// @tparam WeightT edge weight type
template <typename VertexT, typename WeightT>
class graph {
 private:
  // TODO_STUDENT
  unordered_map<VertexT, unordered_map<VertexT, WeightT>> adjacencyList;
  size_t edgeSz = 0;


 public:
  /// Default constructor
  
  graph() : edgeSz(0) {
  }

  /// @brief Add the vertex `v` to the graph, must typically be O(1).
  /// @param v
  /// @return true if successfully added; false if it existed already
  bool addVertex(VertexT v) {
    // TODO_STUDENT
    if (adjacencyList.find(v) != adjacencyList.end()) {
    return false;
    } else {
    adjacencyList[v] = unordered_map<VertexT, WeightT>();
    return true;
    }
  }

  /// @brief Add or overwrite directed edge in the graph, must typically be
  /// O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight edge weight / label
  /// @return true if successfully added or overwritten;
  ///         false if either vertices isn't in graph
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    // TODO_STUDENT
    if (adjacencyList.find(from) == adjacencyList.end() || adjacencyList.find(to) == adjacencyList.end()) {
    return false;
    } else {
    auto& neighbor = adjacencyList[from];
    auto i = neighbor.find(to);
    if (i == neighbor.end()) {
      neighbor[to] = weight;
      edgeSz++;
    } else {
      neighbor[to] = weight;
    }
    return true;
    }
  
  }

  /// @brief Maybe get the weight associated with a given edge, must typically
  /// be O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight output parameter
  /// @return true if the edge exists, and `weight` is set;
  ///         false if the edge does not exist
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    // TODO_STUDENT
    auto fromMap = adjacencyList.find(from);
    if (fromMap == adjacencyList.end()) return false;

    auto toMap = fromMap->second.find(to);
    if (toMap == fromMap->second.end()) return false;

    weight = toMap->second;
    return true;
  }

  /// @brief Get the out-neighbors of `v`. Must run in at most O(|V|).
  /// @param v
  /// @return vertices that v has an edge to
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT> S;
    // TODO_STUDENT
    auto node = adjacencyList.find(v);
    if (node == adjacencyList.end()) return S;
    auto path = node->second.begin();
    
    while (path != node->second.end()) {
      S.insert(path->first);
      ++path;
    }
  
    
    return S;
  }

  /// @brief Return a vector containing all vertices in the graph
  vector<VertexT> getVertices() const {
    // TODO_STUDENT
    vector<VertexT> res;
    auto i = adjacencyList.begin();
    while (i != adjacencyList.end()) {
      res.push_back(i->first);
      ++i;
    }
    return res;
    
  }

  /// @brief Get the number of vertices in the graph. Runs in O(1).
  size_t numVertices() const {
    // TODO_STUDENT
    return adjacencyList.size();
    
  }

  /// @brief Get the number of directed edges in the graph. Runs in at most
  /// O(|V|), but should be O(1).
  size_t numEdges() const {
    // TODO_STUDENT
    size_t total = 0;
    for (const auto& i : adjacencyList) {
      total += i.second.size();
    }
    return total;
    
  }
};
