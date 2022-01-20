import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1 - x2) ** 2 + abs(y1 - y2) ** 2) ** 0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):

            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a ** 4) + 2 * (a ** 2) * (b ** 2) + 2 * (a ** 2) * (c ** 2) -
                               (b ** 4) + 2 * (b ** 2) * (c ** 2) - (c ** 4)) ** 0.5 / (2 * a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0] + heading_vector[0],
                              car_coords[1] + heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):
            if start is None:
                start = 0
            if end is None:
                end = 0
            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count - 1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time / current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[2.88739, 0.72647, 2.5, 0.10878],
                        [3.16759, 0.70479, 2.5, 0.11242],
                        [3.45517, 0.69218, 2.5, 0.11514],
                        [3.75325, 0.68581, 2.5, 0.11926],
                        [4.07281, 0.68361, 2.5, 0.12783],
                        [4.5, 0.68376, 2.5, 0.17088],
                        [4.55, 0.68378, 2.5, 0.02],
                        [5.11738, 0.6908, 2.5, 0.22697],
                        [5.44798, 0.71123, 2.42926, 0.13635],
                        [5.71127, 0.74223, 2.11451, 0.12537],
                        [5.94137, 0.78496, 1.86166, 0.12572],
                        [6.14913, 0.84078, 1.65536, 0.12995],
                        [6.33676, 0.91067, 1.46498, 0.13667],
                        [6.50352, 0.99484, 1.29912, 0.14379],
                        [6.64763, 1.09336, 1.12768, 0.1548],
                        [6.76715, 1.2064, 1.12768, 0.14588],
                        [6.8579, 1.33509, 1.14039, 0.13808],
                        [6.92194, 1.47647, 1.09878, 0.14125],
                        [6.96027, 1.62797, 1.0, 0.15628],
                        [6.9669, 1.78881, 1.0, 0.16097],
                        [6.92977, 1.95515, 1.04719, 0.16276],
                        [6.8538, 2.1191, 1.04719, 0.17255],
                        [6.72693, 2.26842, 1.29232, 0.15161],
                        [6.56583, 2.39791, 1.49971, 0.13782],
                        [6.38076, 2.50633, 1.76788, 0.12133],
                        [6.18037, 2.59603, 2.19388, 0.10007],
                        [5.97126, 2.67207, 2.5, 0.089],
                        [5.75829, 2.7411, 2.5, 0.08955],
                        [5.55841, 2.81013, 2.5, 0.08459],
                        [5.36005, 2.88361, 2.5, 0.08461],
                        [5.16333, 2.96219, 2.5, 0.08473],
                        [4.96845, 3.04683, 2.5, 0.08499],
                        [4.77552, 3.13833, 2.5, 0.08541],
                        [4.58462, 3.23745, 2.5, 0.08604],
                        [4.39562, 3.3442, 2.5, 0.08682],
                        [4.20825, 3.45789, 2.5, 0.08767],
                        [4.02217, 3.5774, 2.5, 0.08846],
                        [3.83713, 3.70184, 2.5, 0.0892],
                        [3.68186, 3.8097, 2.5, 0.07562],
                        [3.52529, 3.9118, 2.29015, 0.08162],
                        [3.36674, 4.00606, 2.08098, 0.08864],
                        [3.20532, 4.09041, 1.96633, 0.09262],
                        [3.04013, 4.16336, 1.92131, 0.09399],
                        [2.87024, 4.22393, 1.91637, 0.09411],
                        [2.69486, 4.27162, 1.91637, 0.09484],
                        [2.51319, 4.30602, 1.85235, 0.09982],
                        [2.32453, 4.32672, 1.7719, 0.10712],
                        [2.12696, 4.3308, 1.66733, 0.11852],
                        [1.91811, 4.31381, 1.51912, 0.13794],
                        [1.69472, 4.26741, 1.36611, 0.16701],
                        [1.45416, 4.17401, 1.2067, 0.21385],
                        [1.21119, 4.00653, 1.2067, 0.24455],
                        [1.01923, 3.74402, 1.23998, 0.26227],
                        [0.92221, 3.42051, 1.65752, 0.20377],
                        [0.88927, 3.10444, 1.9241, 0.16516],
                        [0.89601, 2.82076, 2.24613, 0.12633],
                        [0.92405, 2.56281, 2.34825, 0.11049],
                        [0.96605, 2.3246, 2.21437, 0.10923],
                        [1.01803, 2.11229, 2.05027, 0.10661],
                        [1.08079, 1.91513, 1.90336, 0.1087],
                        [1.15514, 1.73108, 1.74997, 0.11343],
                        [1.24162, 1.56015, 1.62718, 0.11773],
                        [1.34113, 1.40324, 1.43325, 0.12964],
                        [1.45473, 1.26109, 1.28851, 0.14122],
                        [1.58653, 1.13641, 1.28851, 0.14081],
                        [1.74473, 1.03229, 1.56352, 0.12113],
                        [1.92656, 0.94305, 1.76526, 0.11474],
                        [2.13282, 0.86779, 1.97845, 0.11098],
                        [2.36411, 0.8068, 2.28169, 0.10483],
                        [2.61751, 0.75992, 2.5, 0.10308]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0  # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist / (track_width * 0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2] - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff / (SPEED_DIFF_NO_REWARD)) ** 2) ** 2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME * (FASTEST_TIME) /
                                           (STANDARD_TIME - FASTEST_TIME)) * (
                                            steps_prediction - (STANDARD_TIME * 15 + 1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3

        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2] - speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500  # should be adapted to track length and other rewards
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                                       (15 * (STANDARD_TIME - FASTEST_TIME))) * (steps - STANDARD_TIME * 15))
        else:
            finish_reward = 0
        reward += finish_reward

        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################

        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward()  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
