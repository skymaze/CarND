# Reflection

## Files

### main.cpp

+ Handle communication with the simulator and pass sensor fusion data to Road.
+ According to the calculated cost deside whether to change lanes
+ Manage state
+ Generate route

### road.h

+ Classify the traffic on each road
+ Calculate lane cost

## Process flow

1. Determine whether to change lanes based on the distance from the vehicle ahead and current state
2. If too close, pass sensor fusion to Road object to calculate cost.
3. Choose the best action and state based on cost
4. Processe action

## Cost

+ The traffic distence ahead - farther is better
+ The traffic distence behind - farther is better to avoid rear-end
+ The traffic speed ahead - faster is better
+ The traffic speed behind - slower is better to avoid rear-end