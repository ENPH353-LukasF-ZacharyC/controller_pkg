# Robot Controller Package

## Example Video

[![IMAGE ALT TEXT](https://user-images.githubusercontent.com/69919668/144112976-e98061cb-8d28-4ee2-b4ff-7f2158017a89.png)](https://drive.google.com/file/d/1GXuWEhUg67pxoYZ4PCpu2vFkGM3neSk3/view?usp=sharing "Competition Example")

## Systems Diagram and processes overview

<a href="url"><img src="https://user-images.githubusercontent.com/69919668/143199697-a70d4264-5c53-43b7-84b3-4210c07b53ae.png" align="left" height="750" ></a>

### Driving system

Binary Filters out the road the follows its center of mass of the road using PID controls published to the robot's twist node.  
Biases added for intersections and parked cars, center of mass tends to lean towards them so when they are detected the robot veers away from them

#### Pedestrian System

Dectects the red of the crosswalk stop line  

Stops and waits for lots of movement to be detected (the pedestrian walking). Once pedestrian has stopped crossing (Little or no movement) the robot crosses the crosswalk with an forward Bias and left bias (will always move forward and left) for 2 sim seconds 

#### Inner Loop and Truck Avoidance

Once done with the outter loop the robot will turn in at the next intersection (Detected by number of road pixels). The robot turns 90 degrees and waits to see lots of movement (the truck moving in front of it) then waits to see no/very little movement then enters the inner loop and drives as normal. Since the robot's speed is roughly the same as the trucks they will not encounter eachother in the single lap the robot needs to complete

---

### 
