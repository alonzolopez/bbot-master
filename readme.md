This is the file structure for the gazebo launch
bbot_gazebo.launch
	bbot_world.launch
		empty_world.launch
			bbot.world - defines lighting, physics, camera focus, etc.
		bbot_upload.launch
			materials.xacro
			bbot.xacro - defines physical params, inertia, controller constraints