define laser ranger(
	sensor(
		range [0.0 5.6]
		fov 240.0
		samples 240
	)

	color "blue"
	size [0.06 0.06 0.06]

	block(
		points 4
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 1]
		point[3] [1 0]
		z [0 1]
	)
)

define simple_robot position(
	odom_error [0.03 0.03 0.00 0.05]
	localization "odom"
	size [1.2 1.2 0.2]
	gui_nose 1
	drive "diff"
	color "black"

	# Block 1
	block(
		points 4
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 1]
		point[3] [1 0]
		z [0 1]
	)

	# Block 2
	block(
		points 4
		point[0] [1 0]
		point[1] [1 1]
		point[2] [2 1]
		point[3] [2 0]
		z [0 1]
	)

	# Block 3
	block(
		points 4
		point[0] [0 1]
		point[1] [0 2]
		point[2] [1 2]
		point[3] [1 1]
		z [0 1]
	)

	laser(pose [0.23 0.0 -0.15 0.0])
)

