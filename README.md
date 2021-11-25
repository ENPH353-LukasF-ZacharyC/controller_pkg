# Robot Controller Package

## Systems Diagram and processes overview

<a href="url"><img src="https://user-images.githubusercontent.com/69919668/143199697-a70d4264-5c53-43b7-84b3-4210c07b53ae.png" align="left" height="750" ></a>

### Driving system

Follows the Center of mass of the road using PID. 

Dectects the red of the crosswalk stop line

  Stops and waits for the pedestrian to walk. Once pedestrian has stopped crossing car crosses the crosswalk with an forward Bias (will always move forward) for 2 sim seconds
