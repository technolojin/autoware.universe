// Copyright 2023 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "graph/graph.hpp"
#include "graph/nodes.hpp"

#include <iostream>
#include <string>

namespace autoware::diagnostic_graph_aggregator
{

void dump_unit(const NodeUnit * unit, const std::string & indent = "", bool root = true)
{
  const auto path = unit->path().empty() ? "" : unit->path() + " ";
  const auto type = "(" + unit->type() + ")";
  std::cout << indent << "- " << path << type << std::endl;

  if (root || unit->parent_units().size() == 1) {
    for (const auto child : unit->child_nodes()) {
      dump_unit(child, indent + "    ", false);
    }
  }
}

void dump_root(const std::string & path)
{
  Graph graph(path);

  std::cout << "===== Top-level trees ============================" << std::endl;
  for (const auto & unit : graph.nodes()) {
    if (unit->parent_units().size() == 0 && unit->child_nodes().size() != 0) {
      dump_unit(unit);
    }
  }
  std::cout << "===== Subtrees ===================================" << std::endl;
  for (const auto & unit : graph.nodes()) {
    if (unit->parent_units().size() >= 2 && unit->child_nodes().size() != 0) {
      dump_unit(unit);
    }
  }

  std::cout << "===== Isolated units =============================" << std::endl;
  for (const auto & unit : graph.nodes()) {
    if (unit->parent_units().size() == 0 && unit->child_nodes().size() == 0) {
      dump_unit(unit);
    }
  }
}

}  // namespace autoware::diagnostic_graph_aggregator

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: tree <path>" << std::endl;
    return 1;
  }
  autoware::diagnostic_graph_aggregator::dump_root(argv[1]);
}
