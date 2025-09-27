# D-QEM-Simplification
This repository contains the datasets, experimental results, and source code associated with our study:

**Title:** Detail-Preserving Simplification of Textured Mesh Models for Natural Objects  
**Authors:** Yuangang Liu et al.  
**Status:** Manuscript under review

## Project structure:

### Data
*All raw data and experimental results, including the original 3D models and the test results of the simplification algorithm reported in this study.*
**Origin models:** The data files of the three test models—— Meerkat,Oryx, and Cow.
**Simplified models (results):** Resulting models at various levels of simplification generated using different simplification algorithms.

### Source
| File                                      | Description                                                                                                                                                                          |
| ----------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **Pair.cpp / Pair.h**                     | Defines the `Pair` class, representing an edge in the mesh for simplification, including attributes and operations related to the edge collapse cost calculation.               |
| **PairHeap.cpp / PairHeap.h**             | Implements a heap structure (`PairHeap`) to efficiently manage and retrieve edge pairs with the lowest collapse cost during mesh simplification.                                     |
| **Point.cpp / Point.h**                   | Defines the `Point` class representing a 3D vertex, including coordinates, normal vectors, and related operations.                                                                   |
| **PointSet.cpp / PointSet.h**             | Implements a `PointSet` class to manage a collection of points in the mesh, supporting operations such as adjacency queries and update of mesh connectivity.                         |
| **Simplification.cpp / Simplification.h** | Core implementation of the D-QEM mesh simplification algorithm, integrating point, pair, and pair heap structures to progressively simplify the 3D mesh while preserving geometric detail. |

 
 ### Error Metric
*Window and input handler.*
  

## Data structures
- m_edgeVector - vector of all edges (no order);
-	m_vertexNeighbor - multimap from vertex index to all it's neighbors;
- m_errors - vector of mat4 errors for every vertex index;
-	m_OBJIndices - list of OBJIndex, the final structure includes every normal, uv and vertex index in the vertices.
- m_vertices - vector of the positions of the vertices;
- At runtime create a min heap for the edges and update it on every edge removed.

**Use Arrows to move around the scene and zoom in/out.**

##  Images:
> 4K trianlges on the right, 2k on the left.
<img  src="Images/meshSimplification_2000_4.png" width="400" >
<img  src="Images/meshSimplification_2000_5.png" width="400" >

> 4K trianlges on the right, 500 on the left.
<img  src="Images/meshSimplification_500_6.png" width="400" >
<img  src="Images/meshSimplification_500_7.png" width="400" >

> 298 trianlges on the right, 75 on the left.
<img  src="Images/meshSimplification_75_1.png" width="400" >


