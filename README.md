# Robonomy
[YouTube Link](https://youtu.be/HAGtMbE7lBk)

For artists, the traditional art of Zen can be a source of inspiration and tranquillity. This project aims to bring the essence of Zen art to life by designing and developing a robot that can create unique and beautiful Zen art pieces on a Buddha board. Robot art - specifically painting, poses an interesting challenge. Robots are not typically associated with creativity, prompting us to delve deeper into the elements of art and explore how
this can be replicated by a robot.

With this project, we wanted to get a deeper understanding of the algorithms that would enable robots to do creative tasks like painting‘ and enable a robot to paint sketches on a flat surface using a paintbrush. We employed a Franka Panda Emika 7 DOF arm, a Buddha board, a brush, and water to test our ideas. The brush dynamics offered an avenue for certain imperfections in the robot’s artwork, aligning with the conventional perception of creativity and art.
The Buddha Board set up allows for the ”Zen” aspect of our project, where the painting is made with just water. The water evaporates after a while, resetting the board to its original state, making the robot’s output fleeting.

We have 4 different methods implemented:

### 1. QuickDraw

We use the timestaped trajectories available in the quick draw dataset and transform it to the robot frame to get a drawing.

![Quick draw apple](/gifs/quick_draw_apple.gif?raw=true "Quick draw apple")

The source code for this example can be found in
`/scripts/plan_trajectory.py`

### 2. Point Engineering

Nothing fancy just using transforms and logic to make good paintings.
Here in true zen fashion we draw a clock every 15 minutes, which then evaportes and the robot keeps on drawing it again and again.

![Quick draw apple](/gifs/clock.gif?raw=true "Quick draw apple")

The source code for this example can be found in
`/scripts/image2drawing.py` in the `getClockXY` function.

### 3. Fourier Series

Drawing inspiration from the 3Blue1Brown video on using [fourier series](https://www.youtube.com/watch?v=r6sGWTCMz2k) to approximate images we decided to use the approach to make drawings!
Trebel Cleft| Batman
:-------------------------:|:-------------------------:
| ![Trebel Cleft](/gifs/trebel_cleft.gif?raw=true "Trebel Cleft") | ![batman](/gifs/batman.gif?raw=true "batman") |

### 4. Graph Traversal:

Given any image we convert it to a set of X-Y coordinates which are timesequenced. We detect the key features in the image by passing it through X and Y sobel filters and detecting corners in the image. These features are then added as nodes to a graph. We then find the cyclic and acyclic sub graphs. We then run a travelling salesman problem on these subgraphs. Here we want to visit all the nodes but want to minimize the cost of doing so. We model the cost the distance moved by the robot to visit all the nodes. We then make parabolic jumps in z axis to connect all the cyclic and acyclic nodes for the arm to move without disturbing the painting.

This results in a trajectories that looks like this:
CMU RI Logo| Hut | Flower
:-------------------------:|:-------------------------:|:-------------------------:
| ![RI_logo](/gifs/RI_logo.gif?raw=true "RI_logo") | ![hut](/gifs/hut.gif?raw=true "hut") | ![flower](/gifs/flower.gif?raw=true "flower") |

The souorce code to convert the image points to a set of x-y trajectories can be found in: `/planning/gen_path.py`
This generates a `.dot` file which can be then passed to `/scripts/image2drawing.py` to draw it.

To get this to run it requires some interpolation for which we use cubic interrpolation which results in something like this:

![ri logo traj](/images/RI_logo_traj.png?raw=true "ri logo traj")

### Brush dipping

In addition to the above we had to develope a brush dipping strategy to keep the brush moist enough but not have water accumulate at the tip:

![brush_dipping](/gifs/brush_dipping.gif?raw=true "brush_dipping")

The source code for this example can be found in
`/scripts/brush_dipping.py`


### 

Note: All of the examples described below in the README have been implemented for the PickNik Robotics Hackathon to integrate move-it!
The code for the move it demo can be found in `scripts/moveit_code.py`
To run it simply set up move it and launch the `demo.launch` with `rviz:=true` in ROS1 Noetic

Variours patters can be drawn by commenting out line `339` in `moveit_code.py`

Here are the video's for various patterns drawn on a panda arm using MoveIt! 
1. [circle](https://drive.google.com/file/d/1_gVt7hPjdmWhkvGdYQ7WcMPs5dOxOjzD/view?usp=drive_link)
2. [clock](https://drive.google.com/file/d/1yN-19tqKWBul4tfey2qVZp_4WvXW_RB-/view?usp=drive_link)
3. [flower](https://drive.google.com/file/d/1F-AUqSK5NnuKxd4t8yo02xPHn4hxrPzY/view?usp=drive_link)
4. [RI_logo](https://drive.google.com/file/d/1PAhdPotZndl59CMaxbfJkay_zMkyUUVX/view?usp=drive_link)


Authors:
1. [Vineet Tambe](https://www.linkedin.com/in/vineet-tambe/)\*
2. [Ronit Hire](https://www.linkedin.com/in/ronit-hire/)\*
3. [Praveen Venkatesh](https://www.linkedin.com/in/praveenvnktsh/)\*

[*Equal contribution]