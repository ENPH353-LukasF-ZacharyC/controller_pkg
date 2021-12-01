/These logs were originally kept elsewhere/

# November 18th

- Started the code for the license plate recognition neural network using lab 5 as a basis. Lukas used a license plate object to store the plate images, the plate letters and numbers, and the text, which I think would be really useful for generating training data.
  

# November 19th

- Changed the way the license plate object gets the characters, originally it was by cropping, now the code finds the 4 largest continous areas in a binary filtered image. This replicates the way our robot will find characters
- Added functions to distort data
  - blur: adds a Guassian blur
  - rotate_image: rotates an image
  - shear: shears the image, similar to stretching the image by pulling on the bottom right corner
- Currently dealing with an issue where the data is the wrong size

# November 20th
- Fixed the size issue and trained the NN, it works on normal data
- Tried to add it to Main.py but it didn't mesh, had to charge the version of tensorflow for the NN
- The new tensorflow version caused a new size issue

# November 22nd
- With Miti's help fixed the size issue
- Gathered data from the robot and found that it was more distored then previously thought and the NN wasn't very good at identifying characters
- Added two more data distortion functions
  - raze: erodes then dilates the image
  - sp_noise: adds random 1s and 0s noise
- With the two new functions my training data looks much more similar to the robot data

# November 24th
- Added a new NN to Main.py, works pretty well, makes occasional mistakes
- Altered the license_Plate_Handler code so that the robot will take multiple images of a license plate allowing the NN to make multiple predictions and then chose the most common prediction

# November 26th
- Trained NN model 1 that was trained on much more data
- Added it to Main.py and it was very sucessful
- Trained NN model 2 that was given additional letter specific data
- Added it to Main.py and it rarely makes mistakes

# November 29th
- Trained NN model 3 that was trained with additional V data
