#FILE OF PROPERTIES OF TARGETS
from numpy import array,matrix
#aspect ratio tolerances of the target
aspect_ratio_min = 1.1
aspect_ratio_max = 1.5

sensitivity = 55

white_target = {
	"color_lower": array([0,0,255-sensitivity]),
	"color_higher": array([255,sensitivity,255]),
	"aspect_ratio_min": aspect_ratio_min,
	"aspect_ratio_max": aspect_ratio_max,
	"width": 0.2794,
	"type": 'color'
}

red_target = {
	"color_lower": array([130,sensitivity,sensitivity]),
	"color_higher": array([180,255,255]),
	"aspect_ratio_min": aspect_ratio_min,
	"aspect_ratio_max": aspect_ratio_max,
	"width": 0.15875,
	"type": 'color'
}

blue_target = {
	"color_lower": array([90,sensitivity,sensitivity]),
	"color_higher": array([129,255,255]),
	"aspect_ratio_min": aspect_ratio_min,
	"aspect_ratio_max": aspect_ratio_max,
	"width": 0.15875,
	"type": 'color'
}

green_target = {
	"color_lower": array([0,sensitivity,sensitivity]),
	"color_higher": array([89,255,255]),
	"aspect_ratio_min": aspect_ratio_min,
	"aspect_ratio_max": aspect_ratio_max,
	"width": 0.15875,
	"type": 'color'
}

#targets = [white_target, red_target, blue_target, green_target]
color_targets = white_target

#coordinates are (x_platform - x_marker) measured in meters or some other consistent unit
target_2 = {
	"pos_rel": matrix([-.24765, 0, 0]),
	"l_side": .2032, #length of a side
	"ident": 2, #id of the target in the relevant aruco dictionary
	"type": 'aruco'
}

target_4 = {
	"pos_rel": matrix([.24765,0,0]),
	"l_side": 0.1857375,	
	"ident": 4,
	"type": 'aruco'
}

target_7 = {
	"pos_rel": matrix([0,-0.384175,0]),
	"l_side": 0.1254125,
	"ident": 7,
	"type": 'aruco'
}

target_15 = {
	"pos_rel": matrix([0,.23,0]),
	"l_side": 0.1508125,
	"ident": 15,
	"type": 'aruco'
}

target_100 = {
	"pos_rel": matrix([0,0,1.04775]),
	"l_side": 0.18415,
	"ident": 100,
	"type": 'aruco'
}

target_101 = {
	"pos_rel": matrix([0,0,1.04775]),
	"l_side": 0.0762,
	"ident": 101,
	"type": 'aruco',
}

aruco_targets = [target_2, target_4, target_7, target_15, target_100, target_101]


board_1 = {
	"pos_rel": matrix([-0.2128,0.16638,1.05465]),
	"num_x": 2,
	"num_y": 2,
	"size_square": 0.0715,
	"space_between": .01432,
	"marker_start": 1,
	"ident": 1
}

board_2 = {
	"pos_rel": matrix([0,-0.212725,0]),
	"num_x": 2,
	"num_y": 2,
	"size_square": 0.0715,
	"space_between": 0.01432,
	"marker_start": 20,
	"ident": 2
}

board_3 = {
	"pos_rel": matrix([0,-0.231775,0]),
	"num_x": 2,
	"num_y": 2,
	"size_square": 0.07185,
	"space_between": 0.01467,
	"marker_start": 30,
	"ident": 3
}

board_4 = {
	"pos_rel": matrix([0,0.22939375,0]),
	"num_x": 2,
	"num_y": 2,
	"size_square": 0.05115,
	"space_between": 0.0102,
	"marker_start": 40,
	"ident": 4
}

board_5 = {
	"pos_rel": matrix([0,0.1407,0]),
	"num_x": 2,
	"num_y": 2,
	"size_square": .0311,
	"space_between": .0066,
	"marker_start": 50,
	"ident": 5
}

board_6 = {
	"pos_rel": matrix([0.08944,0.53975,1.05465]),
	"num_x": 2,
	"num_y": 2,
	"size_square": .074,
	"space_between": .01544,
	"marker_start": 60,
	"ident": 6
}

board_7 = {
	"pos_rel": matrix([0.457125,0.16788,1.05465]),
	"num_x": 2,
	"num_y": 2,
	"size_square": .07422,
	"space_between": .01525,
	"marker_start": 70,
	"ident": 7
}

# board_8 = {
# 	"pos_rel": matrix([0,-0.1016,0]),
# 	"num_x": 2,
# 	"num_y": 2,
# 	"size_square": 0.05363,
# 	"space_between": 0.0109,
# 	"marker_start": 10,
# 	"ident": 8
# }

board_8 = {
	"pos_rel": matrix([.092,.16388,1.0535]),
	"num_x": 2,
	"num_y": 2,
	"size_square": 0.0715,
	"space_between": 0.01432,
	"marker_start": 10,
	"ident": 8
}


board_9 = {
	"pos_rel": matrix([0.001,0.0,0.0]),
	"num_x": 2,
	"num_y": 2,
	"size_square": .0311,
	"space_between": .00609,
	"marker_start": 80,
	"ident": 9
}

# board_2 = {
# 	"pos_rel": matrix([0,0,0]),
# 	"num_x": 1,
# 	"num_y": 2,
# 	"size_square": .11683,
# 	"space_between": .0235,
# 	"marker_start": 50,
# 	"ident": 2
# }

board_104 = {
 	"pos_rel": matrix([-0.29845,0.5826125,1.0535]),
 	"num_x": 1,
 	"num_y": 2,
 	"size_square": .11716,
 	"space_between": .0237,
 	"marker_start": 104,
 	"ident": 104
}

board_106 = {
 	"pos_rel": matrix([-0.5286375,0.2270125,1.0535]),
 	"num_x": 1,
 	"num_y": 2,
 	"size_square": .11716,
 	"space_between": .0237,
 	"marker_start": 106,
 	"ident": 106
}

# board_108 = {
# 	"pos_rel": matrix([0,0,1.0535]),
# 	"num_x": 1,
# 	"num_y": 2,
# 	"size_square": .11716,
# 	"space_between": .0237,
# 	"marker_start": 108,
# 	"ident": 108
# }
# # 108 isn't placed.

board_112 = {
	"pos_rel": matrix([0.4175125,0.5588,1.0535]),
	"num_x": 1,
	"num_y": 2,
	"size_square": .11716,
	"space_between": .0237,
	"marker_start": 112,
	"ident": 112
}

# board_104 = {
#	"pos_rel": matrix([0.3048,0,1]),
#	"num_x": 1,
#	"num_y": 2,
#	"size_square": .11716,
#	"space_between": .0237,
#	"marker_start": 104,
#	"ident": 104
# }

# board_106 = {
#	"pos_rel": matrix([0,-0.3048,1]),
#	"num_x": 1,
#	"num_y": 2,
#	"size_square": .11716,
#	"space_between": .0237,
#	"marker_start": 106,
#	"ident": 106
# }

# board_108 = {
#	"pos_rel": matrix([-0.3048,0,1]),
#	"num_x": 1,
#	"num_y": 2,
#	"size_square": .11716,
#	"space_between": .0237,
#	"marker_start": 108,
#	"ident": 108
# }

aruco_boards = [board_1, board_2, board_3, board_4, board_5, board_6, board_7, board_8, board_104, board_106, board_112 ]
