#include "solver/GraphColoring.hpp"
#include <iostream>
#include <algorithm>
#include <limits>

namespace Solver {

// ============================================================================
// GraphColoring Base Class
// ============================================================================

GraphColoring::GraphColoring(const Graph& graph) {
    _graph.resize(graph.numNodes);
    
    // Build adjacency list from edges
    for (const auto& edge : graph.edges) {
        _graph[edge[0]].push_back(edge[1]);
        _graph[edge[1]].push_back(edge[0]);
    }
    
    _colors.resize(_graph.size(), -1);
}

GraphColoring::GraphColoring(const std::vector<std::vector<int>>& adjacencyList) {
    _graph = adjacencyList;
    _colors.resize(_graph.size(), -1);
}

int GraphColoring::getNumColors() const {
    int maxColor = -1;
    for (int color : _colors) {
        if (color > maxColor) {
            maxColor = color;
        }
    }
    return maxColor + 1;
}

bool GraphColoring::isValid() const {
    if (_colors.empty() || _graph.size() != _colors.size()) {
        return false;
    }
    
    // Check that no adjacent nodes have the same color
    for (size_t i = 0; i < _graph.size(); i++) {
        if (_colors[i] == -1) {
            return false;  // Uncolored node
        }
        
        for (int neighbor : _graph[i]) {
            if (_colors[i] == _colors[neighbor]) {
                return false;  // Adjacent nodes with same color
            }
        }
    }
    
    return true;
}

void GraphColoring::convertToCategories() {
    _categories.clear();
    int numColors = getNumColors();
    _categories.resize(numColors);
    
    for (size_t i = 0; i < _colors.size(); i++) {
        int color = _colors[i];
        if (color >= 0) {
            _categories[color].push_back(i);
        }
    }
}

void GraphColoring::printStats() const {
    std::cout << "Graph Coloring Statistics:\n";
    std::cout << "  Nodes: " << _graph.size() << "\n";
    std::cout << "  Colors: " << getNumColors() << "\n";
    std::cout << "  Valid: " << (isValid() ? "Yes" : "No") << "\n";
    
    if (!_categories.empty()) {
        std::cout << "  Category sizes:\n";
        for (size_t i = 0; i < _categories.size(); i++) {
            std::cout << "    Color " << i << ": " << _categories[i].size() << " nodes\n";
        }
    }
}

// ============================================================================
// Color Balancing Implementation (from Gaia)
// ============================================================================

float GraphColoring::findLargestSmallestCategories(int& biggestCategory, int& smallestCategory) {
    if (_categories.empty()) {
        convertToCategories();
    }
    
    biggestCategory = 0;
    smallestCategory = 0;
    size_t maxSize = _categories[0].size();
    size_t minSize = _categories[0].size();
    
    for (size_t iColor = 1; iColor < _categories.size(); iColor++) {
        if (maxSize < _categories[iColor].size()) {
            biggestCategory = iColor;
            maxSize = _categories[iColor].size();
        }
        
        if (minSize > _categories[iColor].size()) {
            smallestCategory = iColor;
            minSize = _categories[iColor].size();
        }
    }
    
    return float(_categories[biggestCategory].size()) / float(_categories[smallestCategory].size());
}

int GraphColoring::findChangableNodeInCategory(int sourceColor, int destinationColor) {
    auto& sourceCategory = _categories[sourceColor];
    for (size_t iNode = 0; iNode < sourceCategory.size(); iNode++) {
        if (changable(sourceCategory[iNode], destinationColor)) {
            return iNode;
        }
    }
    return -1;
}

void GraphColoring::changeColor(int sourceColor, int categoryId, int destinationColor) {
    int nodeId = _categories[sourceColor][categoryId];
    _colors[nodeId] = destinationColor;
    
    if (!_categories.empty()) {
        _categories[sourceColor].erase(_categories[sourceColor].begin() + categoryId);
        _categories[destinationColor].push_back(nodeId);
    }
}

bool GraphColoring::changable(int node, int destinationColor) {
    // Check if node can be changed to destinationColor
    for (int neiId : _graph[node]) {
        if (_colors[neiId] == destinationColor) {
            return false;
        }
    }
    return true;
}

void GraphColoring::balanceColoredCategories(float goalMaxMinRatio) {
    if (_categories.empty()) {
        convertToCategories();
    }
    
    float maxMinRatio = -1.f;
    int iterations = 0;
    const int maxIterations = 1000; // Prevent infinite loops
    
    std::cout << "[Color Balancing] Starting with goal ratio: " << goalMaxMinRatio << std::endl;
    
    do {
        int biggestCategory = -1, smallestCategory = -1;
        maxMinRatio = findLargestSmallestCategories(biggestCategory, smallestCategory);
        
        if (maxMinRatio <= goalMaxMinRatio) break;
        
        // Find a changeable vertex from the biggest category to move to the smallest
        int changableId = findChangableNodeInCategory(biggestCategory, smallestCategory);
        if (changableId == -1) {
            // Try other categories
            for (size_t iColor = 0; iColor < _categories.size(); iColor++) {
                if (iColor == biggestCategory || iColor == smallestCategory) {
                    continue;
                }
                
                changableId = findChangableNodeInCategory(iColor, smallestCategory);
                if (changableId != -1) {
                    biggestCategory = iColor;
                    break;
                }
            }
        }
        
        if (changableId == -1) {
            std::cout << "[Color Balancing] Cannot optimize further. Final ratio: " << maxMinRatio << std::endl;
            return;
        }
        
        changeColor(biggestCategory, changableId, smallestCategory);
        iterations++;
        
    } while (maxMinRatio > goalMaxMinRatio && iterations < maxIterations);
    
    std::cout << "[Color Balancing] Finished after " << iterations << " iterations. Final ratio: " << maxMinRatio << std::endl;
}

// ============================================================================
// OrderedGreedy Implementation
// ============================================================================

int OrderedGreedy::nextNode() {
    int minDegreeNode = -1;
    int minDegree = std::numeric_limits<int>::max();
    
    for (size_t i = 0; i < _degrees.size(); i++) {
        if (_degrees[i] == -1) {
            continue;  // Already colored
        }
        
        if (_degrees[i] < minDegree) {
            minDegree = _degrees[i];
            minDegreeNode = i;
        }
    }
    
    return minDegreeNode;
}

void OrderedGreedy::reduceDegree(int node) {
    _degrees[node] = -1;  // Mark as colored
    
    // Reduce degree of all neighbors
    for (int neighbor : _graph[node]) {
        if (_degrees[neighbor] != -1) {
            _degrees[neighbor]--;
        }
    }
}

std::vector<int>& OrderedGreedy::color() {
    // Initialize degrees
    _degrees.clear();
    _degrees.resize(_graph.size(), 0);
    
    for (size_t i = 0; i < _graph.size(); i++) {
        _degrees[i] = _graph[i].size();
        _colors[i] = -1;
    }
    
    // Greedy coloring
    int maxColor = -1;
    int numColored = 0;
    std::vector<bool> colorUsed;
    colorUsed.reserve(128);  // Pre-allocate for common case
    
    while (numColored < static_cast<int>(_graph.size())) {
        int node = nextNode();
        
        if (node == -1) {
            std::cerr << "Error: Could not find next node to color\n";
            break;
        }
        
        // First node - assign color 0
        if (maxColor == -1) {
            maxColor = 0;
            _colors[node] = 0;
        } else {
            // Resize color usage tracker
            colorUsed.resize(maxColor + 1, false);
            std::fill(colorUsed.begin(), colorUsed.end(), false);
            
            // Mark colors used by neighbors
            for (int neighbor : _graph[node]) {
                if (_colors[neighbor] >= 0) {
                    colorUsed[_colors[neighbor]] = true;
                }
            }
            
            // Find smallest available color
            int minAvailableColor = -1;
            for (size_t i = 0; i < colorUsed.size(); i++) {
                if (!colorUsed[i]) {
                    minAvailableColor = i;
                    break;
                }
            }
            
            // Assign color
            if (minAvailableColor == -1) {
                // Need new color
                maxColor++;
                _colors[node] = maxColor;
            } else {
                _colors[node] = minAvailableColor;
            }
        }
        
        reduceDegree(node);
        numColored++;
    }
    
    return _colors;
}

} // namespace Solver
