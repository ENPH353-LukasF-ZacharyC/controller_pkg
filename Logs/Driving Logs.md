# Driving logs

## Nov 8, 2021

- Filtering the images Green channel within a range of 80 - 90 isolates the road with some noise that can be erroded away. 
- From this isolated road image we can find the moment of the road and use PID controls to go towards the centroid. 
- PID controls need to be tuned. 
- Crosswalks don't seem to interfer with driving and moment selection
- Issues seen at intersection where the car will drive to the center of the intersection and then either select the right or left path

Completes a lap around the outside with 50% consistency (No car or pedestrian) 

## Nov 10-14, 2021
### Navigation

- Add intersection detection, when at an intersection there are significantly more road pixels which can be easily counted and compared to a constant
- When we detect an intersection we can add a twist bias to go in the direction we want
- New Issue seen: when starting script from starting position car will either run off road or generally go right not left
- Solution: Hard code start to turn left for a second or two

### Crosswalk

- Methodogy: check number of red pixels right infront of the car, if there are enough stop the car and then check for movement
- Movement checking is done by subtracting the current image from the previous image because the car is still
  - Issue the car isn't always perfectly still so we need to erode the subtracted image and blur
- When we have seen movement of the pedestrian and then no movement for 25 frames allow the car to continue (Note: we won't check for cross time for the next 5 seconds of sim time so the car doesn't stop at the stop line on the other side of the crosswalk. 

Completes a lap around the outside of the circle with 99% accuracy (No pedestrian)
With pedestrian sometimes stops in the middle of the crosswalk or tries to cross right as the pedestrian starts to cross; crosswalk timing needs to be improved

## Nov 16, 2021
Time Trials Tomorrow
### Center Loop ideas
- Use a NN to detect if the truck is too close (I think this is the most interesting choice and will be looking into it) 
  - Use imitation learning: have someone drive around the center loop and record whether they are moving forward or not, if not moving forward then the car is infront of them
    - Might have issues with corners where you are turning
- Cant use Motion detection because the car will also be using. Unless we can find a way to track 2 images to overlay each other
- SIFT to recognize the back and side of the truck.
- 
