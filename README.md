# DPOC_2022_PE_Python
## Winners of Raffaello D'Andrea's Dynamic Programming and Optimal Control ETH competition 2022/2023

The problem we solved is to program a robot to navigate through 4 squared maps (with triangular) in Mars to find gems inside a mine and bring them back to the lab. 
The main challenges are the presence of noise in each map that can move randomly the robot and the presence of obstacles and aliens who can steal the gems.

We solved this Dynamic Programming Infinite Horizon problem by using the Linear Program approach, after having computed SIMULTANEOUSLY the transition probability 
matrix P and the cost matrix G. The incredibly fast result was obtained mainly due to the eigenvalue's method ('highs') chosen to compute the linear program.

The output is the optimal cost vector and the optimal input vector, which describe the optimal cost and optimal input for each tile in each map.

![Image](results.png)
