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
  size_t sz_edges;
  size_t sz_vertices;
  unordered_map<VertexT, unordered_map<VertexT, WeightT>> graphAdjacencyList;

 public:
  /// Default constructor
  graph() {
    sz_edges = 0;
    sz_vertices = 0;
  }

  /// @brief Add the vertex `v` to the graph, must typically be O(1).
  /// @param v
  /// @return true if successfully added; false if it existed already
  bool addVertex(VertexT v) {
    if (graphAdjacencyList.contains(v)) {
      return false;
    }
    graphAdjacencyList[v];
    sz_vertices++;
    return true;
  }

  /// @brief Add or overwrite directed edge in the graph, must typically be
  /// O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight edge weight / label
  /// @return true if successfully added or overwritten;
  ///         false if either vertices isn't in graph
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    if (!graphAdjacencyList.contains(from) || !graphAdjacencyList.contains(to)) {
      return false;
    }
    if (graphAdjacencyList.at(from).find(to) == graphAdjacencyList.at(from).end()) {
      sz_edges++;
    }
    graphAdjacencyList.at(from)[to] = weight;
    return true;
  }

  /// @brief Maybe get the weight associated with a given edge, must typically
  /// be O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight output parameter
  /// @return true if the edge exists, and `weight` is set;
  ///         false if the edge does not exist
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    if (!graphAdjacencyList.contains(from) || !graphAdjacencyList.contains(to) || graphAdjacencyList.at(from).find(to) == graphAdjacencyList.at(from).end()) {
      return false;
    }
    weight = graphAdjacencyList.at(from).at(to);
    return true;
  }

  /// @brief Get the out-neighbors of `v`. Must run in at most O(|V|).
  /// @param v
  /// @return vertices that v has an edge to
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT> S;
    for (auto [vertex, value] : graphAdjacencyList.at(v)) {
      S.emplace(vertex);
    }
    return S;
  }

  /// @brief Return a vector containing all vertices in the graph
  vector<VertexT> getVertices() const {
    vector<VertexT> vertices;
    for (auto [vertex, innerMap] : graphAdjacencyList) {
      vertices.push_back(vertex);
    }
    return vertices;
  }

  /// @brief Get the number of vertices in the graph. Runs in O(1).
  size_t numVertices() const {
    return sz_vertices;
  }

  /// @brief Get the number of directed edges in the graph. Runs in at most
  /// O(|V|), but should be O(1).
  size_t numEdges() const {
    return sz_edges;
  }
};
