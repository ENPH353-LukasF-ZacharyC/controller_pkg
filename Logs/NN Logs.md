#November 18th

- Started the code for the license plate recognition neural network using lab 5 as a basis. Lukas used a license plate object to store the plate images, the plate letters and numbers, 
  and the text, which I think would be really useful for generating training data.

#November 19th

- Changed the way the license plate object gets the characters, originally it was by cropping, now the code finds the 4 largest continous areas in a binary filtered image. This
  replicates the way our robot will find characters
