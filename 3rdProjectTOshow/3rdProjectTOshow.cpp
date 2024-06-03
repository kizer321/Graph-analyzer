#define _CRT_SECURE_NO_WARNINGS

#include <cstdio>
#include "Vector.h"
#include "Queue.h"

void merge(int* degreeS, int left, int mid, int right) {
    int n1 = mid - left + 1;
    int n2 = right - mid;

    int* L = new int[n1];
    int* R = new int[n2];

    for (int i = 0; i < n1; i++)
        L[i] = degreeS[left + i];
    for (int i = 0; i < n2; i++)
        R[i] = degreeS[mid + 1 + i];
    int i = 0, j = 0, k = left;
    while (i < n1 && j < n2) {
        if (L[i] >= R[j]) {
            degreeS[k] = L[i];
            i++;
        }
        else {
            degreeS[k] = R[j];
            j++;
        }
        k++;
    }
    while (i < n1) {
        degreeS[k] = L[i];
        i++;
        k++;
    }
    while (j < n2) {
        degreeS[k] = R[j];
        j++;
        k++;
    }
    delete[] L;
    delete[] R;
}

void mergeSort(int* degreeS, int left, int right) {
    if (left < right) {
        int mid = left + (right - left) / 2;
        mergeSort(degreeS, left, mid); // sorting left site of data
        mergeSort(degreeS, mid + 1, right); // sorting right site of data

        merge(degreeS, left, mid, right); // merging both site of data
    }
}

void DFS(const Vector<Vector<int>>& graph, Vector<int>& visited, int vertex) {
    visited[vertex] = 1;
    int tempSize = graph[vertex].size();
    for (int i = 0; i <tempSize ; i++) {
        int neighbor = graph[vertex].get(i);
        if (!visited[neighbor - 1])
            DFS(graph, visited, neighbor - 1);
    }
}

int countComponents(const Vector<Vector<int>>& graph, int numberOfVertices) {
    Vector<int> visited;
    visited.fill(0, numberOfVertices);
    int componentCount = 0;

    for (int i = 0; i < numberOfVertices; i++) {
        if (!visited.get(i)) {
            DFS(graph, visited, i);
            componentCount++; // counting how many time we have to start DFS algorithm to visit every vertex;
        }
    }
    return componentCount;
}

bool isBipartite(const Vector<Vector<int>>& graph, int startingVertex, Vector<int>& color) {
    Queue<int> q;
    q.push(startingVertex);
    color.push(1, startingVertex);
    while (!q.empty()) {
        int vertex = q.frontElement();
        q.pop();
        int currentColor = color[vertex];
        int nextColor = (currentColor == 1) ? 2 : 1;
        for (int i = 0; i < graph[vertex].size(); i++) {
            int neighbor = graph[vertex][i] - 1;
            if (color[neighbor] == 0) {
                color.push(nextColor, neighbor);
                q.push(neighbor);
            }
            else if (color[neighbor] == currentColor) { // if neighbor of current vertex has been colored by the same color graph is not bipartite
                return false;
            }
        }
    }
    return true;
}

bool isGraphBipartite(const Vector<Vector<int>>& graph, int numberOfVertices) {
    Vector<int> color;
    color.fill(0, numberOfVertices);
    for (int i = 0; i < numberOfVertices; i++) {
        if (color[i] == 0) {
            if (!isBipartite(graph, i, color))
                return false;
        }
    }
    return true; // if we manage to colore graph usigng 2 colors graph is bipartite
}

void BFS(const Vector<Vector<int>>& graph, Vector<int>& distance, int vertex, int numberOfVertices) {
    Queue<int> q;
    Vector<int> visited;
    visited.fill(0, numberOfVertices);
    q.push(vertex);
    visited[vertex] = 1;
    distance[vertex] = 0;

    while (!q.empty()) {
        int v = q.frontElement();
        q.pop();
        for (int i = 0; i < graph[v].size(); i++) {
            int neighbour = graph[v][i] - 1;
            if (!visited[neighbour]) {
                visited[neighbour] = 1;
                distance[neighbour] = distance[v] + 1;
                q.push(neighbour);
            }
        }
    }
}

void eccentricities(const Vector<Vector<int>>& graph, int numberOfVertices) {
    Vector<Vector<int>> distance;
    Vector<int> temp;
    distance.reserve(numberOfVertices);
    temp.fill(-1, numberOfVertices);
    for (int i = 0; i < numberOfVertices; i++) {
        distance.push(temp);
    }
    for (int i = 0; i < numberOfVertices; i++) {
        int eccentricity = 0;
        if(graph[i].size() > 0){
            if(distance[i][0] == -1)
                BFS(graph, distance[i], i, numberOfVertices);
            for (int k = 0; k < numberOfVertices; k++) {
                if (distance[i][k] != -1) {
                    if (eccentricity < distance[i][k])
                        eccentricity = distance[i][k];
                }
            }
        }
        printf("%d ", eccentricity);
    }
}

bool isGraphPlanarBasic(int numberOfVertices, int numberOfEdges) {
    return numberOfEdges <= 3 * numberOfVertices - 6;
}


//tarjan's algorithm for planarity check
bool isPlanarAlgorithm(Vector<Vector<int>>& graph, int numberOfVertices, int vertex, int parent, Vector<int>& visited, Vector<int>& entry, Vector<int> low, int& time, Vector<int> stack, Vector<bool> inStack) {
    visited[vertex] = 1;
    entry[vertex] = low[vertex] = ++time;
    stack.push(vertex);
    inStack[vertex] = 1;

    for (int i = 0; i < graph[vertex].size(); i++) {
        int neighbor = graph[vertex][i] - 1;
        if (neighbor == parent)
            continue;
        if (!visited[neighbor]) {
            if (!isPlanarAlgorithm(graph, numberOfVertices, neighbor, vertex, visited, entry, low, time,stack, inStack)) {
                return false;
            }
            if (low[vertex] > low[neighbor])
                low[vertex] = low[neighbor];
        }
        else if(inStack[neighbor]) {
            if (low[vertex] > entry[neighbor])
                low[vertex] = entry[neighbor];
        }
    }

    if (low[vertex] == entry[vertex]) {
        int  w = 0, componentSize = 0;
        do {
            w = stack[w];
            stack.pop();
            inStack[w] = 0;
            componentSize++;
        } while (w != vertex);
        if (componentSize > 4) return false;
    }
    return true;
}

bool isGraphPlanarAlgorithm(Vector<Vector<int>>& graph, int numberOfVertices) {
    Vector<int> visited;
    Vector<int> entry;
    Vector<int> low;
    Vector<int> stack;
    Vector<bool> inStack;
    visited.fill(0, numberOfVertices);
    entry.fill(-1, numberOfVertices);
    low.fill(-1, numberOfVertices);
    inStack.fill(false, numberOfVertices);
    int time = 0;

    for (int i = 0; i < numberOfVertices; i++) {
        if (!visited[i]) {
            if (!isPlanarAlgorithm(graph, numberOfVertices, i, -1, visited, entry, low, time, stack, inStack)) {
                return false;
            }
        }
    }
    return true;
}

void greedyColoring(Vector<Vector<int>>& graph, int numberOfVertices) {
    Vector<int> result;
    result.fill(-1, numberOfVertices);
    result[0] = 1; // marking 1st vertex with color 0

    Vector<int> available;
    available.fill(0, numberOfVertices);

    for (int i = 1; i < numberOfVertices; i++) {
        for (int j = 0; j < graph[i].size(); j++) {
            int neighbor = graph[i][j];
            if (result[neighbor - 1] != -1) {
                available.push(1, result[neighbor - 1]);
            }
        }

        int color;
        for (color = 1; color < numberOfVertices; color++) { // color = 1 because we have started from color = 0
            if (!available[color]) { // checking which color is available
                break;
            }
        }
        result.push(color, i);

        for (int j = 0; j < graph[i].size(); j++) {
            int neighbor = graph[i][j];
            if (result[neighbor - 1] != -1)
                available.push(0, result.get(neighbor - 1));
        }
    }

    //printing result:
    for (int i = 0; i < numberOfVertices; i++)
        printf("%d ", result.get(i));
}

void mergeForLF(int* degreeS, int* index, int left, int mid, int right) {
    int n1 = mid - left + 1;
    int n2 = right - mid;

    int* L = new int[n1];
    int* R = new int[n2];
    int* L_index = new int[n1];
    int* R_index = new int[n2];

    for (int i = 0; i < n1; i++) {
        L[i] = degreeS[left + i];
        L_index[i] = index[left + i];
    }
    for (int i = 0; i < n2; i++) {
        R[i] = degreeS[mid + 1 + i];
        R_index[i] = index[mid + 1 + i];
    }
    int i = 0, j = 0, k = left;
    while (i < n1 && j < n2) {
        if (L[i] > R[j] || (L[i] == R[j] && L_index[i] < R_index[j])) { // if component in left is bigger than component on right or left's index is lower 
            degreeS[k] = L[i];                                                                  // than we are putting left component to result
            index[k] = L_index[i];
            i++;
        }
        else {
            degreeS[k] = R[j];                                              // else we put right's site component
            index[k] = R_index[j];
            j++;
        }
        k++;
    }
    while (i < n1) {  // putting left components from left site to result
        degreeS[k] = L[i]; 
        index[k] = L_index[i];
        i++;
        k++;
    }
    while (j < n2) { // putting left components from right site to result
        degreeS[k] = R[j];
        index[k] = R_index[j];
        j++;
        k++;
    }
    delete[] L;
    delete[] R;
    delete[] L_index;
    delete[] R_index;
}

void mergeSortForLF(int* degreeS, int* index, int left, int right) {
    if (left < right) {
        int mid = left + (right - left) / 2;
        mergeSortForLF(degreeS, index, left, mid); // sorting left site of data this time we have tgo sort also ids of vertices
        mergeSortForLF(degreeS, index, mid + 1, right); //sorting right site of data (ids and deegrees)

        mergeForLF(degreeS, index, left, mid, right); //merging both sites
    }
}

void LFColoring(const Vector<Vector<int>>& graph, int numberOfVertices, int* deegrees) {
    int* index = new int[numberOfVertices];
    for (int i = 0; i < numberOfVertices; i++)
        index[i] = i;
    mergeSortForLF(deegrees, index, 0, numberOfVertices - 1);

    Vector<int> result;
    result.fill(-1, numberOfVertices);
    Vector<int> available;
    available.fill(0, numberOfVertices);

    for (int i = 0; i < numberOfVertices; i++) {
        int ind = index[i];
        for (int j = 0; j < graph[ind].size(); j++) {
            int neighbor = graph[ind][j];
            if (result[neighbor - 1] != -1)
                available.push(1, result[neighbor - 1]);
        }

        int color;
        for (color = 1; color < numberOfVertices; color++) { // same algorithm as greedy coloring but we are starting from vertex with biggest deegree
            if (!available[color])
                break;
        }

        result.push(color, ind);

        for (int j = 0; j < graph[ind].size(); j++) {
            int neighbor = graph[ind][j];
            if (result.get(neighbor - 1) != -1)
                available.push(0, result[neighbor - 1]);
        }
    }
    for (int i = 0; i < numberOfVertices; i++)
        printf("%d ", result[i]);
    delete[] index;
}

int* SLFOrder(const Vector<Vector<int>>& graph, int numberOfVertices, int* deegrees) {
    Vector<int> included;
    included.fill(0, numberOfVertices);
    Vector<int> coloredNeighbours;
    coloredNeighbours.fill(0, numberOfVertices);
    int* order = new int[numberOfVertices];
    int* index = new int[numberOfVertices];
    for (int i = 0; i < numberOfVertices; i++)
        index[i] = i;
    mergeSortForLF(deegrees, index, 0, numberOfVertices - 1); // also sorting deegries as in LF algorithm
    for (int i = 0; i < numberOfVertices; i++) {
        int minColoredNeighbour = INT_MAX;
        int choosenVertex = -1;
        for (int j = 0; j < numberOfVertices; j++) {
            if (!included[index[j]]) {
                int numberOfColoredNeighbour = 0;
                for (int k = 0; k < graph[index[j]].size(); k++) {
                    int neighbour = graph[index[j]][k]-1;
                    if (included[neighbour]) { // if neigbor of current vertex are included they are colored so we incrase number of colored neighbors
                        numberOfColoredNeighbour++;
                    }
                }
                if ( // looking for vertex with smallest number of neighbors which are colored
                    numberOfColoredNeighbour < minColoredNeighbour || 
                    (numberOfColoredNeighbour == minColoredNeighbour && deegrees[index[j]] > deegrees[choosenVertex]) || // in case of draw 1st we check degree number of vertices
                     (deegrees[index[j]] == deegrees[choosenVertex] && index[j] < index[choosenVertex]))
                {   // than we check index of vertices
                    minColoredNeighbour = numberOfColoredNeighbour;
                    choosenVertex = index[j];
                }
            }
        }
        included[choosenVertex] = 1;
        order[i] = choosenVertex;
    }
    delete[] index;
    return order;
}

void SLFColoring(const Vector<Vector<int>>& graph, int numberOfVertices, int* deegrees) {

    Vector<int> result;
    result.fill(-1, numberOfVertices);
    Vector<int> available;
    available.fill(0, numberOfVertices);
    int* order = SLFOrder(graph, numberOfVertices, deegrees); // putting vertices in order by number of colored neighbrs


    for (int i = 0; i < numberOfVertices; i++) {
        int u = order[i];
        for (int j = 0; j < graph[u].size(); j++) {
            int current = graph[u][j] - 1;
            if (result[current] != -1) {
                available[result[current]] = 1;
            }
        }

        int color;
        for (color = 1; color <= numberOfVertices; color++) {
            if (!available[color]) {
                break;
            }
        }

        result[u] = color;

        for (int j = 0; j < graph[u].size();  j++) {
            int current = graph[u][j] - 1;
            if (result[current] != -1) {
                available[result[current]] = 0;
            }
        }
    }

    for (int i = 0; i < numberOfVertices; i++) {
        printf("%d ", result[i]);
    }
    printf("\n");
    delete[] order;
}

bool isInPath(const Vector<int>& path, int vertex) {
    for (int v = 0; v < path.size(); v++) {

        if (path[v] == vertex) {
            return true;
        }
    }
    return false;
}

void DFS_C4(const Vector<Vector<int>>& graph, int startingVertex, int currentVertex, int depth, int& count, Vector<int>& path) {
    path.push(currentVertex);
    if (depth == 4) {
        for (int k = 0; k < graph[currentVertex].size(); k++) {
            int current = graph[currentVertex][k];
            if (current - 1 == startingVertex) {
                count++;
            }
        }
    }
    else {
        for (int k = 0; k < graph[currentVertex].size();k++) {
            int neighbour = graph[currentVertex][k];
            if (!isInPath(path, neighbour - 1)) {  // also we have to check if we didnt go like starting vertex->neigbor-> starting->neighbor->starting
                DFS_C4(graph, startingVertex, neighbour - 1, depth + 1, count, path);
            }
        }
    }
    path.pop();
}

int countC4Subgraphs(const Vector<Vector<int>>& graph, int numberOfVertices) {
    int count = 0;
    for (int i = 0; i < numberOfVertices; i++) {       // counting C4 - starting DFS from each vertex and ending it after depth is equal 4 
        Vector<int> path;
        DFS_C4(graph, i, i, 1, count, path); // optimalization idea - breaking in each iteration if vertex has been visited
    }
    return count / 8;       //each cycle is counted 8 times - from every vertex and in both directions
}



int main() {
    int numberOfGraphs = 0;
    scanf("%d", &numberOfGraphs);
    for (int i = 0; i < numberOfGraphs; i++) {
        int numberOfVertices = 0;
        long long numberOfEdges = 0;
        scanf("%d", &numberOfVertices);
        Vector<Vector<int>> graph;
        graph.reserve(numberOfVertices);
        int* deegrees1 = new int[numberOfVertices];
        int* deegrees = new int[numberOfVertices];
        for (int k = 0; k < numberOfVertices; k++) {
            int numberOfNeighbours = 0;
            scanf("%d", &numberOfNeighbours);
            numberOfEdges += numberOfNeighbours;
            deegrees1[k] = numberOfNeighbours;
            deegrees[k] = numberOfNeighbours;
            Vector<int> neighbors;
            neighbors.reserve(numberOfNeighbours);
            for (int j = 0; j < numberOfNeighbours; j++) {
                int neighbour;
                scanf("%d", &neighbour);
                neighbors.push(neighbour);
            }
            graph.push(neighbors);
        }
        numberOfEdges /= 2;

        mergeSort(deegrees1, 0, numberOfVertices - 1);
        for (int j = 0; j < numberOfVertices; j++) {
            printf("%d ", deegrees1[j]);
        }
        delete[] deegrees1;
        printf("\n");
        int ans2 = countComponents(graph, numberOfVertices);   // 2nd answer
        printf("%d\n", ans2);
        if (isGraphBipartite(graph, numberOfVertices))
            printf("T\n");
        else                                                  // 3rd answer
            printf("F\n");                                 
        //eccentricities(graph, numberOfVertices);            // 4th
        //printf("\n");
        printf("?\n");
        printf("?\n");                                  
        //if (!isGraphPlanarBasic(numberOfVertices, numberOfEdges))    //5th
        //    printf("F\n");
        //else
        //    if (isGraphPlanarAlgorithm(graph, numberOfVertices))
        //        printf("T\n");
        //    else
        //        printf("F\n");
        greedyColoring(graph, numberOfVertices);            // 6th
        printf("\n");
        LFColoring(graph, numberOfVertices, deegrees);                //7
        printf("\n");
        //printf("?\n");                     
        printf("?\n");  //8
        //SLFColoring(graph, numberOfVertices, deegrees);                  //8
        //int ans9 = countC4Subgraphs(graph, numberOfVertices); //9
        //printf("%d\n", ans9);
        printf("?\n");                                      
        long long temp2 = numberOfVertices - 1;    // number of complements edges is maximum possible number of edges ( n*(n-1) / 2 ) substracted by number of edges in current graph
        long long ans10 = numberOfVertices * temp2 / 2 - numberOfEdges; 
        printf("%lld\n", ans10);                                      // 10th answer
        delete[] deegrees;
    }
    return 0;
}
