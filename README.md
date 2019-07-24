# Learning to Fly: Computational Controller Design for Hybrid UAVs with Reinforcement Learning
**Jie Xu, Tao Du, Michael Foshey, Beichen Li, Bo Zhu, Adriana Schulz, Wojciech Matusik**

**ACM Transactions on Graphics, 38(4) 42:1-42:12 (SIGGRAPH), 2019**



#### Code Structure

- `SimulationUI`: C++ rigid body simulation to visualize the controller's performance. (C++)
- `Training`: Code to train the controller with our method with Reinforcement Learning. (Python)
- `ardupilot`: Modified Ardupilot firmware code to implement our controller on real hardware. (C++)



#### Training

- The code has been tested on `Ubuntu 16.04`.

- The code relies on a modified version of OpenAI baselines in `git@github.com:eanswer/openai_baselines.git`. To run the code, please first clone the repository 

  ```
  git clone git@github.com:eanswer/openai_baselines.git
  ```

  and then follow the `README.md` to install baselines.

- We provide two models in the data folder.

- To start training a controller for a model, for example QuadPlane, in one thread,

  ```
  python run.py --model QuadPlane
  ```

  or in multiple threads (e.g. 4),

  ```
  mpirun -np 4 python run.py --model QuadPlane
  ```

  You can also add a `--visualize` to visualize the training by `Mujoco`. (If you don't install `Mujoco`, this option does't work).

- The trained models are saved in `results` folder.

- To test a trained mode, for example model_afterIter_400.ckpt, you can run

  ```
  python run.py --model QuadPlane --controller model_afterIter_400 --play
  ```

- To save a trained mode to a `c++` format to be plugged into the `SimulationUI` code or `ardupilot` firmware code