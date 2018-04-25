#ifndef ROAD_H
#define ROAD_H

#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

class Road {
    public:
        int lanes_available;
        vector<float> vehicles_ahead;
        vector<float> vehicles_behind;
        vector<int> vehicles_parallel;
        vector<float> traffic_speed_ahead;
        vector<float> traffic_speed_behind;

        Road() {
            lanes_available = 3;
        }
        void update_road_status(double car_s, double car_speed, int prev_size, vector<vector<double>> sensor_fusion) {
        // init status
            vehicles_ahead.clear();
            vehicles_behind.clear();
            vehicles_parallel.clear();
            traffic_speed_ahead.clear();
            traffic_speed_behind.clear();
            for (int i = 0; i < lanes_available; i++) {
                vehicles_ahead.push_back(999);
                vehicles_behind.push_back(999);
                vehicles_parallel.push_back(0);
                traffic_speed_ahead.push_back(999);
                traffic_speed_behind.push_back(0);
            }
            for (int i = 0; i < sensor_fusion.size(); i++) {
                float d = sensor_fusion[i][6];
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx * vx + vy * vy);
                double check_car_s = sensor_fusion[i][5];

                check_car_s += ((double)prev_size * .02 * check_speed);
                double relative_distance = check_car_s - car_s;

                for (int j = 0; j < lanes_available; j++) {
                // vehicles in lane j
                    if (d < (2 + 4 * j + 2) && d > (2 + 4 * j -2)) {
                        // vehicles ahead
                        if (relative_distance >= 0) {
                            // update min distance
                            if (relative_distance < vehicles_ahead[j]) {
                                vehicles_ahead[j] = relative_distance;
                            }
                            // update min speed
                            if (check_speed < traffic_speed_ahead[j]) {
                                traffic_speed_ahead[j] = check_speed;
                            }
                        // vehicles behind
                        } else if (relative_distance < 0){
                            // update min distance
                            if (abs(relative_distance) < vehicles_behind[j]) {
                                vehicles_behind[j] = abs(relative_distance);
                            }
                            // update max speed
                            if (check_speed > traffic_speed_behind[j]) {
                                traffic_speed_behind[j] = check_speed;
                            }
                        }
                        if (0 <= relative_distance && relative_distance <= 20) {
                            vehicles_parallel[j] = 1;
                        } else if (relative_distance > -15 && 0 > relative_distance) {
                            vehicles_parallel[j] = 1;
                        } else if (relative_distance > -30 && -15 >= relative_distance) {
                            if (check_speed > car_speed) {
                                vehicles_parallel[j] = 1;
                            }
                        }
                        break;
                    }
                }
            }
            // debug
            // for (int i = 0; i < lanes_available; i++) {
            //     cout << "line" << i << "status updated:" <<  vehicles_ahead[i] << " " << vehicles_behind[i] << " " << traffic_speed_ahead[i] << " " << traffic_speed_behind[i] << endl;
            // }
        };

        float get_distance_cost(float distance) {
            return exp(-abs(distance));
        }

        float get_speed_ahead_cost(float speed) {
            return exp(-abs(speed));
        }

        float get_speed_behind_cost(float speed) {
            return 1-exp(-abs(speed));
        }
        
        float get_line_cost(int line) {
            return get_distance_cost(vehicles_ahead[line])
                + get_distance_cost(vehicles_behind[line])
                + 2 * get_speed_ahead_cost(traffic_speed_ahead[line])
                + get_speed_behind_cost(traffic_speed_behind[line])
                + vehicles_parallel[line] * 999;
        }
        float get_current_line_cost(int line) {
            return get_distance_cost(vehicles_ahead[line])
                + get_distance_cost(vehicles_behind[line])
                + 2 * get_speed_ahead_cost(traffic_speed_ahead[line])
                + get_speed_behind_cost(traffic_speed_behind[line])
                + vehicles_parallel[line] * 999;
        }
};

#endif