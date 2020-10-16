        uint32 id 		  # obstacle ID.
	Point3d position  # obstacle position in the world coordinate
	# system.
	float32 theta 	 # heading in the world coordinate system.
	Point3d velocity  # obstacle velocity.

	# Size of obstacle bounding box.
	float32 length  # obstacle length.
	float32 width   # obstacle width.
	float32 height  # obstacle height.

	# obstacle corner points
	Point3d[] polygon_points 
	# duration of an obstacle since detection in seconds.
	float32 tracking_time 

	uint16 obstacle_type    # obstacle type
	uint16 sub_type   # obstacle sub_type
	float32 gps_timestamp  # GPS time in seconds.
	string type_name 

	float32 confidence 
	float32 type_confidence
	bool is_static

	float32 reserved_1
