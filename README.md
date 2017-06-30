# Extended Kalman Filter Project Solution
Self-Driving Car Engineer Nanodegree Program

This is my (Anton Varfolomeev) solution for EKF project (P1 of the Term 2 of Self-Driving Car Engineer Nanodegree Program)



This project utilizes fusion of two kalman filters to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

The project was edited using MS Visual studio editor, compiled and executed in `Bash on Ubuntu on Windows` against
the Windows version of the Udacity simulator.

## Project Implemetation details

This project's implementation was simple and straightforward, given the lectures
and tutorials we have.

Besides the main code (marked with TODO in project template files), I made several
smaller changes:

- As Tools class contains no data members, I replaced it with the Tools namespace. It removed
 the need for the **tools** object (and the need to pass this object to the lambda function)

- To gain a better understanding of the RMSE behavior, I added RMSE logging to the `main` fuction:

```

  std::ofstream logRMSE;
  logRMSE.open("RMSE.log");
  ....
  logRMSE << RMSE(0) << ", " << RMSE(1) << ", " << RMSE(2) << ", " << RMSE(3) << std::endl;

```
- To reduce the amount of computations, I check for the current `dt` value, and re-calculate **Q** and **F** 
matrices only when it is necessary. I know, it may have a little value in real situation,
but in current project (with constant time step of 50 ms) I compute these matrices only once.


## Results

Resulted RMSE:
 
 X: 0.0945

 Y: 0.0847

VX: 0.3310

VY: 0.4087



