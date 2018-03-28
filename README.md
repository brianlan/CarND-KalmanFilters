# CarND-KalmanFilters
CarND-KalmanFilters


- Prediction is not necessarily to happen when there's a observation, because the observation could miss for some 
  time steps in the real world actually, but the prediction should always triggered when the time elapsed.
- KF always assumes there's already a prediction for current time step t (it could be a initial guess if we know
  nothing about the state). If we got a measurement at time step t, we'll update the state x and P according to the 
  newly observed data (at this moment, x should be close to the measurement). At the same time, the error of current
  measurement and the prediction we were done at time step t - 1 is taken into account to measure the uncertainty
  and the hidden state (e.g. velocity). OK, the next thing we're going to do is to give a prediction for 
  time step t + 1. And this rule goes over and over.
- In the course example, we have a state vector containing position and velocity. We do observe position but nothing 
  about the velocity. But the interesting thing is we can still maintain a quite accurate velocity value using 
  position observations and our transition matrix F. The velocity here is so called hidden state, which could be the
  observable variables' driven power.
- Question: why the uncertainty P is quite different before and after the prediction-step?