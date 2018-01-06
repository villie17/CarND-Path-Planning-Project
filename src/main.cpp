#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;


// Constant to define max speed of car in mph
const double MAX_SPEED = 49.392;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}


/*
 * Helper function to calculate car speed
 */
double getCarSpeed(double vx, double vy)
{
	return sqrt(vx*vx+vy*vy);
}
/*
 * Check if the lane is safe. It checks if there is a car in my lane b/w fwd_dist and back_dist of car_s
 * Returns:
 * A vector containing 3 values.
 * First is a flag. 1 if lane is safe, 0 otherwise
 * Second value is the id of car in lane in front of me
 * Third is id of car in lane behind me
 */

vector<int> checkLaneSafe(vector<vector<double>>& sensor_fusion, int prev_size, double car_s, int lane, int fwd_dist, int back_dist){

	// Sanity check
	if (lane<0 || lane > 2) return { 0, 999999, -999999};

	int safe = 1; // Assume its safe, until proven otherwise
	double front_min_s = 999999; // Big number
	double back_min_s = -999999; // Big number
	double diff_s = 0.0;
	int front_car_id = -1;
	int back_car_id = -1;

	for(int i=0; i<sensor_fusion.size(); i++){
		// The d of the car
		float d = sensor_fusion[i][6];
		// My lane number
		int my_d = 2+4*lane;
		// If the car is anywhere in target lane
		if(d < (my_d+2) &&  d > (my_d-2)){
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double check_speed = sqrt(vx*vx+vy*vy);

			double check_car_s = sensor_fusion[i][5];

			// Extrapolate car to my location
			check_car_s += ((double)prev_size*0.02*check_speed);
			diff_s = check_car_s - car_s;
			// check if car is within certain given distance
			if((diff_s > - back_dist) && (diff_s < fwd_dist)){
				safe = 0;

				if (diff_s < 0 && diff_s > back_min_s){
					back_min_s = diff_s;
					back_car_id = i;
					//cout << "Car id:" << i << " Back min_s" << back_min_s << endl;
				}
				if (diff_s > 0 && diff_s < front_min_s){
					front_min_s = diff_s;
					front_car_id = i;
					//cout << "Car id:" << i << " Front min_s" << front_min_s << endl;
				}
			}
		}
	}
	return {safe, front_car_id, back_car_id};
}

/*
 * The States I am using for Finite State Machines
 */
enum State  {KEEP_LANE=0, PREPARE_LEFT_CHANGE, CHANGE_LEFT, PREPARE_RIGHT_CHANGE, CHANGE_RIGHT};

/*
 * Class to keep track of cars status
 * It has current
 */
class CarStatus{
public:
	int lane = 1; 								// Current lane
	double ref_vel = 0.0;						// Current speed
	double target_speed = MAX_SPEED;			// Target speed
	State cur_state = KEEP_LANE;				// Current status
	/*
	 * Set new target speed but only if its lower than current target speed
	 */
	void setMinTargetSpeed(double target_speed){
		if (this->target_speed < target_speed) return;
		this->target_speed = target_speed;
	}
	/*
	 * Print the state
	 */
	void printState(){
		cout << "State: ";
		if (this->cur_state == KEEP_LANE) cout << "KEEP_LANE";
		else if (this->cur_state == PREPARE_LEFT_CHANGE) cout << "PREPARE_LEFT_CHANGE";
		else if (this->cur_state == PREPARE_RIGHT_CHANGE) cout << "PREPARE_RIGHT_CHANGE";
		else if (this->cur_state == CHANGE_LEFT) cout << "CHANGE_LEFT";
		else if (this->cur_state == CHANGE_RIGHT) cout << "CHANGE_RIGHT";
		cout << " Lane: " << this->lane << " Ref Vel: " << this->ref_vel << " Target speed: "<< this->target_speed << endl;

	}
};

/*
 * This is where next state in state diagram is calculated
 *       __________________________________________________________________________
 *      |                                                                          |
 *   KEEP_LANE ________________________________________________-> CHANGE_LEFT____->|
 *      |      |____________________________________________|__-> CHANGE_RIGHT___->|
 *      |      |
 *      |      |______-> PREPARE_LEFT_CHANGE______________->|
 *      |      |______-> PREPARE_RIGHT_CHANGE_____________->|
 *      |                                                   |
 *      |<-_________________________________________________|
 *
 */
void NextAction(CarStatus& car, vector<vector<double>>& sensor_fusion,  int prev_size, double car_s)
{
	vector<int> safety_front, safety_left, safety_right;
	safety_front = checkLaneSafe(sensor_fusion, prev_size, car_s, car.lane , 30, 0);
	safety_right = checkLaneSafe(sensor_fusion, prev_size, car_s, car.lane + 1, 20, 10);
	safety_left = checkLaneSafe(sensor_fusion, prev_size, car_s, car.lane - 1, 20, 10);

	if (safety_front[0] == 1){ // No car in front of me
		if (car.cur_state == PREPARE_LEFT_CHANGE || car.cur_state == PREPARE_RIGHT_CHANGE){
			// Ignore for now
			cout << " In Prepare lane change state" << endl;
		}
		else{
			car.cur_state = KEEP_LANE;
			car.target_speed == MAX_SPEED;
			return;
		}
	}
	else{
		int target_id = safety_front[1];
		car.target_speed = getCarSpeed(sensor_fusion[target_id][3], sensor_fusion[target_id][4])*2.24; // Match speed of front car

	}
	// First priority is for left lane change
	if (safety_left[0] == 1){
		car.cur_state = CHANGE_LEFT;
		car.target_speed = MAX_SPEED; // Reset
		car.lane--;
		return;
	}
	if (safety_right[0] == 1){
		car.cur_state = CHANGE_RIGHT;
		car.target_speed = MAX_SPEED; // Reset
		car.lane++;
		return;
	}

	// No car.lane change from down below
	//cout << "I am in lane: " << car.lane << endl;
	//cout << "Left:" << safety_left[0] << "\t" << safety_left[1] << "\t" << safety_left[2] << endl;
	//cout << "Right:" << safety_right[0] << "\t" << safety_right[1] << "\t" << safety_right[2] << endl;

	// If left or right both not safe, then it gets interesting
	// Check lanes that can turn left
	if (car.lane == 1 || car.lane == 2){
			car.cur_state = PREPARE_LEFT_CHANGE; // Prepare to change left

			if (safety_left[1] >= 0){ // There is a car in front in left lane
				int target_id = safety_left[1];
				double front_left_target_speed = getCarSpeed(sensor_fusion[target_id][3], sensor_fusion[target_id][4])*2.24 ;
				//cout << "Front left : " << front_left_target_speed << endl;
				if (front_left_target_speed < car.target_speed){
					car.cur_state = KEEP_LANE;
				}else{
					//car.target_speed = front_left_target_speed-5; // go bit slower than front car
				}
			}
			else{
				int target_id = safety_left[2];
				double behind_left_target_speed = getCarSpeed(sensor_fusion[target_id][3], sensor_fusion[target_id][4])*2.24;
				//cout << "Behind left : " << behind_left_target_speed << endl;
				if (behind_left_target_speed < car.target_speed){
					car.cur_state = KEEP_LANE;
				}else{
					//car.target_speed = behind_left_target_speed+5; // Match the target speed of behind,
					// No need to change speed, the car is faster let it pass by
				}
			}
			//cout << " State: " << "PREPARE_LEFT_CHANGE to "<< car.lane << " with target speed " << car.target_speed << endl;
			return;
	}

	if (car.lane == 0 || car.lane == 1){
		car.cur_state = PREPARE_RIGHT_CHANGE; // Prepare to change right

		if (safety_right[1] >= 0){ // There is a car in front in right lane
			int target_id = safety_right[1];
			double front_right_target_speed = getCarSpeed(sensor_fusion[target_id][3], sensor_fusion[target_id][4])*2.24;
			//cout << "Front right : " << front_right_target_speed << endl;
			if (front_right_target_speed < car.target_speed){
				car.cur_state = KEEP_LANE;
			}else{
				//car.target_speed = front_right_target_speed-5; // Match the target speed of front, but go slower
			}
		}
		else{
			int target_id = safety_right[2];
			double behind_right_target_speed = getCarSpeed(sensor_fusion[target_id][3], sensor_fusion[target_id][4])*2.24;
			//cout << "Behind right : " << behind_right_target_speed << endl;
			if (behind_right_target_speed < car.target_speed){
				car.cur_state = KEEP_LANE;
			}else{
				//car.target_speed = behind_right_target_speed+5; // Match the target speed of behind, but go faster
				// No need to change speed, the car is faster let it pass by
			}
		}

		//cout << " State: " << "PREPARE_RIGHT_CHANGE to "<< car.lane << " with target speed " << car.target_speed << endl;
		return;
	}

	return;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // Declare car object
  CarStatus car = CarStatus();

  // Add car object to onMessage function amount other things
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&car](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];


          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          	// Remaining previous path, which could not be played out ( the pacman dots that couldn't been eaten )
          	int prev_size = previous_path_x.size();

          	json msgJson;

          	// Assume car is at end of previous path, we are going to use the previous path
          	if (prev_size > 0){
          		car_s = end_path_s;
          	}

          	/*
          	 * Decide next car state, target speed and next car lane
          	 */


          	NextAction(car, sensor_fusion, prev_size, car_s);
          	car.printState();


          	// The path that I will make for the car
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// Points for spline
			vector<double> ptsx;
			vector<double> ptsy;

			// Where am I?
			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);

			// If no previous points, get the first two points

			if(prev_size < 2){
				// This happens if car is restarted in middle
				if (car_yaw > 180) car_yaw = 360-car_yaw;
				double prev_car_x = car_x - cos(car_yaw);
				double prev_car_y = car_y - sin(car_yaw);

				ptsx.push_back(prev_car_x);
				ptsx.push_back(car_x);

				ptsy.push_back(prev_car_y);
				ptsy.push_back(car_y);
			}
			else // Start from last 2 previous points
			{
				ref_x = previous_path_x[prev_size-1];
				ref_y = previous_path_y[prev_size-1];

				double ref_x_prev = previous_path_x[prev_size-2];
				double ref_y_prev = previous_path_y[prev_size-2];
				ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
				ptsx.push_back(ref_x_prev);
				ptsx.push_back(ref_x);

				ptsy.push_back(ref_y_prev);
				ptsy.push_back(ref_y);

			}

			// Get 3 more points, 30 meters apart
			vector<double> next_wp0 = getXY(car_s+30,(2+4*car.lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp1 = getXY(car_s+60,(2+4*car.lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp2 = getXY(car_s+90,(2+4*car.lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);

			// Push x coords
			ptsx.push_back(next_wp0[0]);
			ptsx.push_back(next_wp1[0]);
			ptsx.push_back(next_wp2[0]);

			// Push y coords
			ptsy.push_back(next_wp0[1]);
			ptsy.push_back(next_wp1[1]);
			ptsy.push_back(next_wp2[1]);

			// Transform them into local car coordinates
			for (int i=0; i<ptsx.size(); i++){
				double shift_x = ptsx[i]-ref_x;
				double shift_y = ptsy[i]-ref_y;

				ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
				ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
			}

			// Create a spline
			tk::spline s;
			s.set_points(ptsx, ptsy);

			//vector<double> next_x_vals;
			//vector<double> next_y_vals;

			// Push back the previous points
			for(int i=0; i<previous_path_x.size(); i++){
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}

			double target_x = 30.0;
			double target_y = s(target_x);
			double target_dist = sqrt((target_x*target_x) + (target_y*target_y));

			double x_add_on = 0;


			for(int i=1; i<= 50-previous_path_x.size(); i++){
				// Calculate next point with .02 second * m/second /2.24 = meters
				double N = (target_dist)/(.02*car.ref_vel/2.24);
				double x_point = x_add_on+(target_x)/N;
				double y_point = s(x_point);

				x_add_on = x_point;
				double x_ref = x_point;
				double y_ref = y_point;

				// Convert from car to global coords
				x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
				y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

				x_point += ref_x;
				y_point += ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);

				if (car.ref_vel - car.target_speed > 0.2){
					car.ref_vel -= 0.224/2; // This works
				}
				else if (car.ref_vel - car.target_speed < -0.2)
				{
					car.ref_vel += 0.224/2; // This works
				}
			}



          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
