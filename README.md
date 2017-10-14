# TERM 3 - Path planning project from Self Driving Car Engineer Nanodegree

The main objective of the project is to develop a path planning algorithm which will drive the car autonomously in a highway. 

The algorithm can be divided in three parts:

1. Prediction. 
2. Behavior Planning.
3. Trajectory Generation.

## Prediction
In the Prsediction module, it is necessary to decode the information from the "Sensor fusion" and then generate a set of predictions and parameters which will be used later for the logic of the Behavior Planner.

In the following lines it's shown the implementation of such module:

        for(int i=0; i < sensor_fusion.size(); i++){
   			
   			double tv_vx = sensor_fusion[i][3];
			double tv_vy = sensor_fusion[i][4];
			double tv_vel = sqrt(tv_vx*tv_vx+tv_vy*tv_vy);
			double tv_s = sensor_fusion[i][5];

			Vehicle target_vehicle = Vehicle(0,tv_s, tv_vel,0,0);
			target_vehicle.lane = target_vehicle.find_lane(sensor_fusion[i][6]);
			

			int tv_id = sensor_fusion[i][0];
			int prediction_duration = 1;
			vector<vector<int>> preds = target_vehicle.generate_predictions(prediction_duration);
			predictions[tv_id] = preds;
			double tv_ttc;
			if (car_speed > 0)
			{
			tv_ttc = ((tv_s - car_s)/(tv_vel - car_speed*0.000277778));	
			}
			else
			{
			tv_ttc = 999999;
			}
			target_vehicle.TTC = tv_ttc;
			target_vehicles.push_back(target_vehicle);

			#ifdef DEBUG_PREDICTIONS
			cout << "ID " << tv_id  << " vel(MPH) " << tv_vel/0.44704 << " s " << tv_s <<  " Lane  " << target_vehicle.lane << " TTC " << tv_ttc << endl;
			#endif
		}
    
It is important to remark, that I've created a parameter for each target vehicle called TTC (Time To Collision). This parameter provides us a very useful information for determining the risk of collisions and also deciding when to turn right or left in the highway.

## BEHAVIOR PLANNING
The main purpose of this module is to decide when the car shall behave to react to the different situations which can be found during the highway driving. The output of this module are the velocity and the lane where the host vehicle shall be driving.

In the following lines it is shown the implementation of such module:


		bool too_close = false;
		bool right_warning = false;
		bool left_warning = false;
		bool front_warning = false;
		double HV_target_velocity = 49;
		double TV_TTC_velocity = 30;
		lane = Host_Vehicle.lane;

		for (int i = 0; i < target_vehicles.size();i++)
		{
		// Check if the cars are in our lane
		
		

		double TTC_WARNING = 2;
		double TV_Distance = target_vehicles[i].s - Host_Vehicle.s;

		if (abs(target_vehicles[i].TTC) < TTC_WARNING && TV_Distance > 0)
		{
			if (target_vehicles[i].lane == Host_Vehicle.lane)
			{
				front_warning = true;
				TV_TTC_velocity = target_vehicles[i].v;
			}
			if (target_vehicles[i].lane > Host_Vehicle.lane)
			{
				right_warning = true;
			}
			if (target_vehicles[i].lane < Host_Vehicle.lane)
			{
				left_warning = true;
			}

		} 
		}

		if (front_warning == false)// && right_warning == false && left_warning == false )
		{
			//ref_vel = HV_target_velocity;
			if (ref_vel < 49)
			{
				ref_vel+=.224;
			}
			
		} 
		else if (front_warning == true)
		{
			if (right_warning == true && left_warning == true)
			{
				//if (ref_vel < 49)
			//{
				ref_vel -=.224;// TV_TTC_velocity/0.44704 - 5;
			//}
				#ifdef DEBUG_BEHAVIOR
				cout << "Front Warning decrease velocity " << " Lane " << lane <<  endl;
				#endif
			}
			else if (right_warning == true && Host_Vehicle.lane > 0)
			{
				lane -= 1; 
				#ifdef DEBUG_BEHAVIOR
				cout << "Front Warning turn left " << " Lane " << lane <<  endl;
				#endif
			}
			else if (left_warning == true && Host_Vehicle.lane < 2)
			{
				lane += 1;
				#ifdef DEBUG_BEHAVIOR
				cout << "Front Warning turn right " << " Lane " << lane << endl;
				#endif
			}

			else
			{
				//if (ref_vel < 49)
			//{
				if (lane > 0)
				{
					lane -= 1;
				}
				else if (lane < 2)
				{
					lane += 1;
				}
				ref_vel-=.224;
				
It is important to remark that the key of such module is to verify is there is any risk of collsion around the host vehicle. If there are no collission risks we have to decide if we want to drive at target speed, turn right, turn left or decrease the speed.

Once this has been decided, the commands of velocity and lane are sent to the trajectory planner module.

## TRAJECTORY GENERATION
The main objective of the TRAJECTORY GENERATION module is to generate a path which will be sent to the Simulator. Such module is based in the one which has been explained and implemented by Udacity professors in the "Walkthrough".

## CONCLUSIONS 
I've succesfully implemented and algorithm which is able to take decisions and drive autonomously in a highway. At the beginning   my objective was to generate a set of predictions for each target vehicle and also implement a set of trajectories for each state of the host vehicle and implement some cost funcions in order to do it in a more robust way. However, my computer was not able to compute all the calculation in real-time and I finally decided to implement a simpler implementation which was also able to work for the Highway scenario.

My conclusion is that to implement a good path planning module it is necessary a powerful computer to implement all the diferent states that can be found in the reality. It is also really difficult to define all the states that can be found in the road and that's why I think this is one of the most difficult parts in the development of an Automated Vehicle.

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

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

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
