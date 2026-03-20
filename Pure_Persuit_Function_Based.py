# imports all necessary libraries
from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, multitask, run_task, wait
import umath as math



# robot setup
prime_hub = PrimeHub()
Right = Motor(Port.A, Direction.CLOCKWISE)
Left = Motor(Port.B, Direction.COUNTERCLOCKWISE)
Fourfouri_Ultimate = DriveBase(Left, Right, 62.4, 134)     
wheel_circumference = math.pi* 62.4
wheel_distance = 134



# graph_vales should be in the form [[x, a, b, c, d], [x, a, b, c, d], ... , [x]]
graph_values = [[100, -4.8010973936899854e-05, 0.016255144032921807, 2.189300411522634, -233.47050754458166], [280, 7.705421727362899e-06, -0.011904876568775686, 4.854415688238631, 4.956512526126901], [750, -3.318518518518517e-06, 0.011199999999999995, -11.199999999999996, 3699.9999999999986], [1500, -9.999999999998999e-05, 0.4699999999999543, -734.9999999999301, 383399.99999996444], [1600]]



# robot values
drive_base = [62.4, 134, 500] # wheel diameter, wheel distance, base speed
wheel_diameter = drive_base[0]
wheel_circumference = math.pi* wheel_diameter
wheel_distance = drive_base[1]
base_speed = drive_base[2]







# idk just variabled. Dylan said he will deal with these later
starting_robot_heading = math.pi/2 # in radians
last_linear = 0
last_robot_heading = starting_robot_heading


# updates values related with calculation of localization
def update_localization_values():
  global starting_robot_heading, last_linear, last_robot_heading # for these ones I either: 1)just used global to avoid error 2)they are pre-set values that are used inside function
  global linear_difference, robot_heading_difference, current_robot_heading # these ones are used for localization and/or pure persuit


  # reads sensors and sets current values
  right_angle = Right.angle()
  left_angle = Left.angle()
  current_linear = (right_angle + left_angle) * wheel_circumference/720
  current_robot_heading = starting_robot_heading - math.radians(prime_hub.imu.heading()) # in radians


  # finds the difference with the values of previous check
  linear_difference = current_linear - last_linear
  robot_heading_difference = current_robot_heading - last_robot_heading


  # saves current values for future use
  last_linear = current_linear
  last_robot_heading = current_robot_heading






  
# variables setup
robot_x = graph_values[0][0] # robot starting x position
robot_y = graph_values[0][1] * graph_values[0][0]**3 + graph_values[0][2] * graph_values[0][0]**2 + graph_values[0][3] * graph_values[0][0] + graph_values[0][4] # robot starting y position
 

def localization():  
  global robot_x, robot_y, current_robot_heading, linear_difference, robot_heading_difference


  if robot_heading_difference != 0:
    trajectory_radius = linear_difference/robot_heading_difference
    overall_displacement = 2*trajectory_radius*math.sin(robot_heading_difference/2)

  else:
    overall_displacement = linear_difference

  robot_x += overall_displacement * math.cos(current_robot_heading - robot_heading_difference/2)
  robot_y += overall_displacement * math.sin(current_robot_heading - robot_heading_difference/2)







# tuning variables
lookahead = 120
target_x = 0
target_y = 0
spline_counter = 0
spline_x_endpoint = graph_values[1][0]
spline_y_endpoint = graph_values[0][1] * spline_x_endpoint**3 + graph_values[0][2] * spline_x_endpoint**2 + graph_values[0][3] * spline_x_endpoint + graph_values[0][4] 


def target_point_selector():
  global robot_x, robot_y, current_robot_heading
  global target_x, target_y, lookahead
  global graph_values
  global spline_x_endpoint, spline_y_endpoint
  
  
  final_spline_point_x_distance = spline_x_endpoint - robot_x
  final_spline_point_y_distance = spline_y_endpoint - robot_y
  final_spline_point_total_distance = math.sqrt(final_spline_point_x_distance**2 + final_spline_point_y_distance**2)


  target_x = robot_x + final_spline_point_x_distance/final_spline_point_total_distance * lookahead
  if target_x > spline_x_endpoint:
    target_x = spline_x_endpoint
  target_y = graph_values[spline_count][1] * target_x**3 + graph_values[spline_count][2] * target_x**2 + graph_values[spline_count][3] * target_x + graph_values[spline_count][4] 
  

  if final_spline_point_total_distance < 120 and spline_count != len(graph_values):
    spline_count += 1
    spline_x_endpoint = graph_values[spline_count + 1][0]
    spline_y_endpoint = graph_values[spline_count + 1][1] * spline_x_endpoint**3 + graph_values[spline_count + 1][2] * spline_x_endpoint**2 + graph_values[spline_count + 1][3] * spline_x_endpoint + graph_values[spline_count + 1][4] 







# to do: fix issue when robot is on same y as target
def pure_persuit(target_x, target_y):
  global wheel_distance, base_speed, right_speed, left_speed, left_max_speed, right_max_speed, left_acceleration, right_acceleration
  global robot_x, robot_y , current_robot_heading


  # calculates distance and angle to target
  x_dif = target_x - robot_x
  y_dif = target_y - robot_y
  total_distance = math.sqrt(x_dif**2 + y_dif**2)
  angle_dif = math.atan2(y_dif, x_dif) 


  # calculates the position of target point relative to robot view
  target_relative_angle = angle_dif - math.radians(current_robot_heading) 
  relative_target_y = math.sin(target_relative_angle) * total_distance


  # calculates speed multipliers
  if relative_target_y != 0:
    radius = -( total_distance**2 / (2*relative_target_y) )

    if abs(2*radius) != wheel_distance:
      speed_multiplier_right = 2*radius / (2*radius+wheel_distance)
      speed_multiplier_left = 2*radius / (2*radius-wheel_distance)

    else:
      speed_multiplier_left = int(2*radius == wheel_distance)
      speed_multiplier_right = int(2*radius == -wheel_distance)

  else:
    speed_multiplier_left = 1
    speed_multiplier_right = 1


  # applies new speed (this needs to be moved near value check for better accuracy)
  Right.run(base_speed*speed_multiplier_right)
  Left.run(base_speed*speed_multiplier_left)







# forever loop   
while True:
  update_localization_values()

  # to do: add if so these wont run if sensors havent updated
  localization()
  target_point_selector()
  pure_persuit(target_x, target_y)