# SJTU-MGTS3601 Graph Data Analysis & Business Applications

![Language](https://img.shields.io/badge/Language-C++-blue)
![Build](https://img.shields.io/badge/Build-CMake-green)

This repository contains the course projects for **MGTS3601 Graph Data Analysis and Business Applications** at Antai College of Economics and Management, Shanghai Jiao Tong University.

The projects focus on reproducing algorithms from top-tier conference papers in the fields of graph databases and graph computing.

##  Project Structure

The repository consists of two independent sub-projects:

1.  **Project 1:** Reproducing Span-Reachability queries on Temporal Graphs.
2.  **Project 2:** Reproducing Truss-based Structural Diversity search on Large Graphs.

---

##  Project 1: Span-Reachability Queries

###  Reference Paper
* **Title:** Efficiently Answering Span-Reachability Queries in Large Temporal Graphs
* **Authors:** Dong Wen, Yilun Huang, Ying Zhang, Lu Qin, Wenjie Zhang, Xuemin Lin
* **Conference:** IEEE International Conference on Data Engineering (ICDE), 2020
### Implementation Status
The `main.cpp` file in this directory contains the full implementation of the algorithms, including the baseline online search and the TILL-REACH index construction. It has been verified to successfully reproduce the results presented in the original paper.

---

##  Project 2: Truss-Based Structural Diversity Search

###  Reference Paper
* **Title:** Truss-Based Structural Diversity Search in Large Graphs
* **Authors:** Jinbin Huang, Xin Huang, Jianliang Xu
* **Conference:** IEEE International Conference on Data Engineering (ICDE), 2021
### Implementation Status
The `main.cpp` file in this directory implements the **GCT Index (Global Connectivity based Truss Index)** and its associated algorithms. The code successfully reproduces the structural diversity calculation results as described in the paper.

---

##  Notes
* Dataset files are located within the respective project directories (`*.txt`).
* For detailed analysis and experimental results, please refer to the `report.pdf` included in each project folder.