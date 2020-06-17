# Dynamic Programming and Markov Processes

<img src="https://github.com/coldhenry/Dynamic-Programming-and-Markov-Processes/blob/master/pic/env.png" width="120">

## Introduction

- In this paper, we aims to design an algorithm that generate **an optimal path for a given Key and Door environment.** 
- There are five objects on a map: the agent (the start point), the key, the door, the treasure (the goal), and walls. The agent has three regular actions, move forward (MF), turn left (TL), and turn right (TR), and two special actions, pick up key (PK) and unlock door (UD). Taking any of these ﬁve actions costs energy (positive cost).
- This type of problems could be considered as a Markov Decision Process, which could be applied to other applications in robotics. To solve this problem, the high level approach is to assign different cost to each actions given various scenarios. Same movement could result in different cost while encountering a wall or a key. We start from the very end of the path, calculate its cost, and propagate the cost to the beginning. By doing so, the dynamic programming algorithm ensures us a path that get the agent to the treasure location with a minimal cost. 



## Markov Processes

​	We formulate this problem as a Markov Decision Process, which is a Markov Reward Process with controlled transitions defined by a tuple of elements as following

1. State space X
2. Control space U
3. Motion Model f
4. Initial State x0
5. Planning Horizon T
6. The state cost l(x,u) and terminal cost q(x)



## Dynamic Programming

​	It is an algorithm that can compute an optimal closed-loop policy given a known Markov Decision Process model of the environment. The main idea is to use the value functions to structure the search for good polices. The advantages it has are that it is much more efficient than the brute-force approach of evaluating all possible strategies. 

<img src="https://github.com/coldhenry/Dynamic-Programming-and-Markov-Processes/blob/master/pic/DP_alg.png" width="360">

## Test cases
There are 7 test cases, which are built with the module gym-minigrid.

<img src="https://github.com/coldhenry/Dynamic-Programming-and-Markov-Processes/blob/master/pic/testcase.png" width="480">



## Results

All results are demonstrated with GIFs.

|                    **doorkey-5x5-normal**                    |                    **doorkey-6x6-direct**                    | **doorkey-6x6-normal**                                       | **doorkey-6x6-shortcut**                                     |
| :----------------------------------------------------------: | :----------------------------------------------------------: | ------------------------------------------------------------ | ------------------------------------------------------------ |
| <img src="https://github.com/coldhenry/Dynamic-Programming-and-Markov-Processes/blob/master/gif/doorkey-5x5-normal.gif" width="80"> | <img src="https://github.com/coldhenry/Dynamic-Programming-and-Markov-Processes/blob/master/gif/doorkey-6x6-direct.gif" width="80"> | <img src="https://github.com/coldhenry/Dynamic-Programming-and-Markov-Processes/blob/master/gif/doorkey-6x6-normal.gif" width="80"> | <img src="https://github.com/coldhenry/Dynamic-Programming-and-Markov-Processes/blob/master/gif/doorkey-6x6-shortcut.gif" width="80"> |
|                    **doorkey-8x8-normal**                    |                    **doorkey-8x8-direct**                    | **doorkey-8x8-shortcut**                                     |                                                              |
| <img src="https://github.com/coldhenry/Dynamic-Programming-and-Markov-Processes/blob/master/gif/doorkey-8x8-normal.gif" width="80"> | <img src="https://github.com/coldhenry/Dynamic-Programming-and-Markov-Processes/blob/master/gif/doorkey-8x8-direct.gif" width="80"> | <img src="https://github.com/coldhenry/Dynamic-Programming-and-Markov-Processes/blob/master/gif/doorkey-8x8-shortcut.gif" width="80"> |                                                              |

