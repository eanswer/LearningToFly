# Learning to Fly: Computational Controller Design for Hybrid UAVs with Reinforcement Learning
**Jie Xu, Tao Du, Michael Foshey, Beichen Li, Bo Zhu, Adriana Schulz, Wojciech Matusik**

**ACM Transactions on Graphics, 38(4) 42:1-42:12 (SIGGRAPH), 2019**



#### Code Structure

- `SimulationUI`: C++ rigid body simulation to visualize the controller's performance. (C++)
- `Training`: Code to train the controller with our method with Reinforcement Learning. (Python)
- `ardupilot`: Modified Ardupilot firmware code to implement our controller on real hardware. (C++)



#### Training

- The code relies on a modified version of OpenAI baselines in `git@github.com:eanswer/openai_baselines.git`. To run the code, please first clone the repository 

  ```
  git clone git@github.com:eanswer/openai_baselines.git
  ```

- 

