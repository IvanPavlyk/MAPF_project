# Improved Conflict Based Search With Heuristics 
This project offers solving  Multi-Agent-Path-Finding(MAPF) problem optimally using Conflict-Based Search(CBS). 

To improve the performance of CBS, the project constains implementations of the following heuristics to guide the high-level CBS algorithm: 

* Prioritize cardinal conflict over non-cardinal and semi-cardinal conflicts 
* Minimum Vertex Cover of Cardinal Conflict Graph
* Minimum Vertex Cover of Dependency Graph
* Minimum Weighted Vertex Cover Weighted Dependency Graph

The search algorithm also contains two implementations of resolving collisions
* Standard Splitting 
* Disjoint Splitting 


## Usage
    python run_experiments.py --instance instances/test_47.txt --solver ICBS --h 2  --r 1 --disjoint

- instance: input map
- disjoint: specifies if to use disjoint splitting 
- solver: MAPF solver, either "CBS" or "ICBS"
- h: 
  - --h 0: Priorotizing Cardinal Conflicts
  - --h 1: Using Cardinal Graph(CG) heuristics
  - --h 2: Using Dependency Graph(DG) heuristic
  - --h 3: Using Weighted Dependency Graph Heuristic(WDG) heuristic
- r: The number of times to run the program on the instance 



## Resources 

This project is an implementation of the following paper: 
https://www2.cs.sfu.ca/~hangma/pub/ijcai19.pdf
    

  

