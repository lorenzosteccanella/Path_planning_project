# Path-Planning-Project


### Path generation.
The project has been accomplished in two steps:

1° step: Path generation in order to drive smoothly in the street. The car is controlled thanks to point path that the planner generates, the car moves from point to point perfectly, so we don't have to worry about building a controller for this project. 
The first attempt in creating a feasible trajectory with the goal of driving at a velocity of 50 MPH, this means that since the car moves 50 times a second, a distance of 0.5m per move will create a velocity of 25 m/s, has been made just using Frenet coordinate and space the point 0.5m. This approach seems to work well within the average velocity, but has the issue that sometimes exceed the maximum velocity. In fact, with this approach we are not considering that when we turn, we are traversing more distance.

Since a linear trajectory doesn't work we take advantage of the spline c++ library that implements a cubic interpolation to solve this issue:
```
// create spline object
tk::spline s;
// set x,y points to the spline
s.set_points(ptsx,ptsy);

// calculate how to break up spline points so we trave at the target velocity
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

// number of points 
double N = (target_dist/(.02*ref_vel/2.24));
```

2° step: Avoid collision and change lane when a lane is free. The goal of this step is to find a policy to follow in order to drive at highway, this include decide when change lane and decide when you can't change lane and you have to slow down.
The sensor_fusion vector list all the vehicle in the street with velocity information, d Frenet coordinate and s Frenet coordinate: 
```
float d = sensor_fusion[i][6];
double vx= sensor_fusion[i][3];
double vy= sensor_fusion[i][4];
double check_speed = sqrt(vx*vx + vy*vy);
double check_car_s = sensor_fusion[i][5];

check_car_s += ((double) prev_size*.02*check_speed);
```
For detecting if a car is too close we can simply detect if the car is in my lane and if the s distance is under a determined threshold (i.e. 30): 
```
// car is in my lane
		  if(d < (2+4*lane+2) && d > (2+4*lane-2)){
		    // car is too close
		    if((check_car_s > car_s) && ((check_car_s - car_s) < 30))
		    {
		      too_close= true; 
		    }
}
```
This allows to reduce the speed when we are too close to another car.
The second step is to detect whether there is an empty lane where we can switch in order to overtake the car in our lane.
```
// calculate if exist a free lane
// lane 1 
if(d < (2+4*0+2) && d > (2+4*0-2)){
 //cout<<" you are in lane 0 -->"<<abs(check_car_s- car_s)<<endl;
 if(min_s_line_0 > abs(check_car_s - car_s)){
min_s_line_0 = abs(check_car_s - car_s);
 }
}
// lane 2
if(d < (2+4*1+2) && d > (2+4*1-2)){
 //cout<<" you are in lane 1 -->"<<abs(check_car_s - car_s)<<endl;
 if(min_s_line_1 > abs(check_car_s - car_s)){
min_s_line_1 = abs(check_car_s - car_s);
 }
}
// lane 3
if(d < (2+4*2+2) && d > (2+4*2-2)){
 //cout<<" you are in lane 2 -->"<<abs(check_car_s - car_s)<<endl;
 if(min_s_line_2 > abs(check_car_s - car_s)){
min_s_line_2 = abs(check_car_s - car_s);
 }
}
```
This can be done calculating the minimum s distance for each lane and then we can simple with some if statement decides what behavior we want:

```

if( too_close==true)
		{
        // decrease velocity
		  ref_vel -= .100; 
        
        // change line
		  if(lane == 0){
		  	if((min_s_line_0 > min_s_line_1) && (min_s_line_0 >min_s_line_2) && (min_s_line_0>20)) { lane= 0;}
		  	if((min_s_line_1 > min_s_line_0) && (min_s_line_1 >min_s_line_2) && (min_s_line_1>20)) { lane= 1;}
		  }
		  if(lane == 1){
			if((min_s_line_0 > min_s_line_1) && (min_s_line_0 >min_s_line_2) && (min_s_line_0>20)) { lane= 0;}
		  	if((min_s_line_1 > min_s_line_0) && (min_s_line_1 >min_s_line_2) && (min_s_line_1>20)) { lane= 1;}
			if((min_s_line_2 > min_s_line_1) && (min_s_line_2 >min_s_line_0) && (min_s_line_2>20)) { lane= 2;}
		  }
		  if(lane == 2){
		  	if((min_s_line_1 > min_s_line_0) && (min_s_line_1 >min_s_line_2) && (min_s_line_1>20)) { lane= 1;}
			if((min_s_line_2 > min_s_line_1) && (min_s_line_2 >min_s_line_0) && (min_s_line_2>20)) { lane= 2;}
		  }
		  
}

```

### Simulator.
You can download the Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
Safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 
The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```


