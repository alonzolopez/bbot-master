## BBOT File Structure
bbot_gazebo.launch
>bbot_world.launch
>>empty_world.launch
>>>bbot.world - defines lighting, physics, camera focus, etc.
>>bbot_upload.launch
>>>materials.xacro
>>>bbot.urdf.xacro
>>>bbot.xacro - defines physical params, inertia, controller constraints
>>>bbot.gazebo.xacro - defines gazebo params including sensor plugins and mu vals for motors
>>>bbot.transmission.xacro - uses the <transmission> tags to attach actuators to revolute Gazebo joints
>>>utilities.xacro
>bbot_control.launch - launches the appropriate controller, loads controller gains from iiwa_control.yaml
>>bbot_control.yaml - set the controller gains for all controllers
>>bbot_position_interface.yaml
>>bbot_velocity_interface.yaml
