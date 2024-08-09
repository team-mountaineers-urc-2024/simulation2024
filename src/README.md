# Src Folder

This folder can be used to store the files for custom plugins

Known Plugins
- [Physics](README.md#physics)
- [User Commands](README.md#user-commands)
- [Scene](README.md#scene)
- [Diff Drive](README.md#diff-drive)
- [Skid Steer](README.md#diff-drive-skid)
- [Battery](README.md#battery)
- [Collada World Exporter](README.md#collada-world-exporter)
- [Logging](README.md#logging)
- [Logical Audio Source](README.md#logical-audio-source)
- [Microphone](README.md#microphone-sensor)
- [Lift Drag](README.md#lift-drag-plugin)
- [Joint State Publisher](README.md#joint-state-publisher)
- [Joint Force](README.md#joint-force)
- [Render Engine](README.md#render-engine)
- [Sensors](README.md#sensors)
- [Acoustic Comms](README.md#acoustic-comms)
- [Breadcrumbs](README.md#breadcrumbs)
- [Camera Video Recorder](README.md#camera-video-recorder)
- [Comms Endpoint](README.md#comms-endpoint)
- [Contact](README.md#contact)
- [Detachable Joint](README.md#detachable-joint)
- [Follow Actor](README.md#follow-actor)
- [Force Torque](README.md#force-torque)
- [IMU](README.md#IMU)
- [Log Video Recorder](README.md#log-video-recorder)
- [Logical Camera](README.md#logical-camera)
- [Navsat](README.md#navsat)
  
[comment]: <> (\[Triggered Publisher\]\(README.md#triggered-publisher\))

---

### Physics

This plugin adds basic physics (like gravity) to the simulation

This plugin does not take any parameters

```
<plugin
  filename="libignition-gazebo-physics-system.so"
  name="ignition::gazebo::systems::Physics">
</plugin>
```

---

### User Commands

This plugin allows the user to execute commands (like spawning an entity) while the simulation is still running

This plugin does not take any parameters

```
<plugin
  filename="libignition-gazebo-user-commands-system.so"
  name="ignition::gazebo::systems::UserCommands">
</plugin>
```

---

### Scene

This plugin publishes the current world to the gui, otherwise the simulation would run without any visual aspect

This plugin does not take any parameters

```
<plugin
  filename="libignition-gazebo-scene-broadcaster-system.so"
  name="ignition::gazebo::systems::SceneBroadcaster">
</plugin>
```

---

### Diff Drive

This plugin allows the user to control the movement of a model with a differential drive from a specified ignition topic

This plugin takes the following parameters:
- `left_joint`: The joint linking the left wheel to the chassis
- `right_joint`: The joint linking the right wheel to the chassis
- `wheel_separation`: The horizontal distance between the two wheels in meters
- `wheel_radius`: The radius of the wheels, in meters
- `odom_publish_frequency`: The frequency at which the odometry should be published out
- `topic`: The Gazebo topic through which the command velocity will be published

```
<plugin
  filename="libignition-gazebo-diff-drive-system.so"
  name="ignition::gazebo::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>1.2</wheel_separation>
    <wheel_radius>0.4</wheel_radius>
    <odom_publish_frequency>1</odom_publish_frequency>
    <topic>cmd_vel</topic>
</plugin>
```

---

### Drive Drive Skid

This plugin allows the user to control the movement of a model with a skid steer drive from a specified ignition topic

This plugin takes the following parameters:
- `left_joint`: This is duplicated, one takes in the front joint, the other takes in the back joint for the left wheels
- `right_joint`: This is duplicated, one takes in the front joint, the other takes in the back joint for the right wheels
- `wheel_separation`: The horizontal distance between the two wheels in meters
- `wheel_radius`: The radius of the wheels, in meters
- `odom_publish_frequency`: The frequency at which the odometry should be published out
- `topic`: The Gazebo topic through which the command velocity will be published

```
<plugin
  filename="libignition-gazebo-diff-drive-system.so"
  name="ignition::gazebo::systems::DiffDrive">
    <left_joint>front_left_wheel_joint</left_joint>
    <left_joint>back_left_wheel_joint</left_joint>
    <right_joint>front_right_wheel_joint</right_joint>
    <right_joint>back_right_wheel_joint</right_joint>
    <wheel_separation>1.2</wheel_separation>
    <wheel_radius>0.4</wheel_radius>
    <odom_publish_frequency>1</odom_publish_frequency>
    <topic>cmd_vel</topic>
</plugin>
```

---

# Battery
This plugin simulates a battery on the rover. When said battery 'dies', the connected joints will become fixed

This plugin takes the following parameters:
- `<battery_name>`: The name of the battery
- `<voltage>`: Initial voltage of the battery (V)
- `<open_circuit_voltage_constant_coef>`: Voltage at full charge (V)
- `<capacity>`: Total charge that the battery can hold (Ah)
- `<power_load>`: Power load on battery (W)
- `<start_draining>`: Start draining battery from the begining of the simulation. If this is not set the battery draining can only be started through the topics set through
- `<start_power_draining_topic>`: Topic(s) that can be used to start power draining
- `<stop_power_draining_topic>`: Topic(s) that can be used to stop power draining
- `<fix_issue_225>`: As reported [here](https://github.com/gazebosim/gz-sim/issues/225), there are some issues affecting batteries in Ignition Blueprint and Citadel
- `<open_circuit_voltage_linear_coef>`: Amount of voltage decrease when no charge (V).
- `<initial_charge>`: Initial charge of the battery (Ah).
- `<resistance>`: Internal resistance (Ohm)
- `<smooth_current_tau>`: Coefficient for smoothing current.  
- `<enable_recharge>`: Should be `true` to enable recharging.
- `<charging_time>`: Hours taken to fully charge the battery. Keep in mind that this value assumes no battery load while charging. If the battery is under load, it will take a longer time to recharge.
- `<recharge_by_topic>`: If true, the start/stop signals for recharging the battery will also be available via topics. The regular Ignition services will still be available.

```
<plugin filename="ignition-gazebo-linearbatteryplugin-system"
  name="gz::sim::systems::LinearBatteryPlugin">
    <!--Li-ion battery spec from LIR18650 datasheet-->
    <battery_name>linear_battery</battery_name>
    <voltage>4.2</voltage>
    <open_circuit_voltage_constant_coef>4.2</open_circuit_voltage_constant_coef>
    <open_circuit_voltage_linear_coef>-2.0</open_circuit_voltage_linear_coef>
    <initial_charge>2.5</initial_charge>
    <capacity>2.5 </capacity>
    <resistance>0.07</resistance>
    <smooth_current_tau>2.0</smooth_current_tau>
    <enable_recharge>false</enable_recharge>
    <charging_time>3.0</charging_time>
    <soc_threshold>0.51</soc_threshold>
    <!-- Consumer-specific -->
    <power_load>2.1</power_load>
    <start_on_motion>true</start_on_motion>
</plugin>
```

---

### Collada World Exporter

This world plugin that automatically generates a Collada mesh consisting of the models within a world SDF file. This plugin can be useful if you'd like to share an SDF world without requiring an SDF loader.

This plugin does not take any parameters

Running the world with the following command will generate a directory with all the mesh and materials for the worl
```
ign gazebo -v 4 -s -r --iterations 1 WORLD_FILE_NAME
```

```
<plugin
  filename="ignition-gazebo-collada-world-exporter-system"
  name="gz::sim::systems::ColladaWorldExporter">
</plugin>
```

---

### Logging

This world plugin records Console messages and simulation state to a file

This plugin does not take any parameters

```
<plugin
  filename="ignition-gazebo-log-system"
  name="gz::sim::systems::LogRecord">
</plugin>
```

---

### Logical Audio Source

This plugin acts as an audio source from which audio can play

This plugin takes in the following parameters:
- `<id>`: A unique identifer that allows the source to be affected by services
- `<pose>` : Where the source should be placed and how it should be rotated
- `<attenuation_function>` : How the sound should be affected as it travels through space
- `<attenuation_shape>` : The shape of the sound as it propogates
- `<inner_radius>` : Where the sound will be at maximum volume
- `<falloff_distance>` : Where the sound can no longer be heard
- `<volume_level>` : At what volume the sound should be
- `<playing>` : Whether it is playing anything or not
- `<play_duration>` : How long the source is to play a sound

```
<plugin filename="ignition-gazebo-logicalaudiosensorplugin-system" name="gz::sim::systems::LogicalAudioSensorPlugin">
    <source>
        <id>1</id>
        <pose>.5 0 0 0 0 0</pose>
        <attenuation_function>linear</attenuation_function>
        <attenuation_shape>sphere</attenuation_shape>
        <inner_radius>1.0</inner_radius>
        <falloff_distance>5.0</falloff_distance>
        <volume_level>.8</volume_level>
        <playing>true</playing>
        <play_duration>10</play_duration>
    </source>
</plugin>
```

---

### Microphone Sensor

This plugin acts as an audio reciving device from which audio can be detected

This plugin takes in the following parameters:
- `<id>` : a unique identifer (unique to a given model)
- `<pose>` : Where the microphone should be placed and how it should be rotated
- `<volume_threshold>` : A value between 0.0 and 1.0 which is the minimum volume the microphone can detect

```
<plugin filename="ignition-gazebo-logicalaudiosensorplugin-system" name="gz::sim::systems::LogicalAudioSensorPlugin">
    <microphone>
        <id>1</id>
        <pose>0 .5 0 0 0 0</pose>
        <volume_threshold>.4</volume_threshold>
    </microphone>
</plugin>
```

---

### Lift Drag Plugin

This plugin is meant to simulate lift and drag for a flying vehicle

This plugin takes in the following parameters:
- `<link_name>`: The link to which the lift and drag will apply

```
<plugin
  name="gz::sim::systems::LiftDrag"
  filename="ignition-gazebo-lift-drag-system">
    <!-- ...configuration goes here... -->
    <link_name>rotor_0</link_name>
</plugin>
```

---

### Joint State Plublisher

This plugin allows the user and nodes to publish direct joint values

This plugin does not take in any parameters

```
<plugin
  filename="ignition-gazebo-joint-state-publisher-system"
  name="gz::sim::systems::JointStatePublisher"
</plugin>
```

---

### Joint Force

This plugin applies a force to the selected joint

This plugin takes in the following parameters:
- `<joint_name>`: The joint that will have the force applied

```
<plugin
  filename="ignition-gazebo-apply-joint-force-system"
  name="gz::sim::systems::ApplyJointForce">
    <joint_name>rotor_0_joint</joint_name>
</plugin>
```

---

### Render Engine

This plugin is used to render an image

This plugin takes in the following parameters:
- `<render_engine>` : The engine that should be used to render the scene
- `<background_color>` : The background color of the scene

```
<plugin
  filename="ignition-gazebo-sensors-system"
  name="ignition::gazebo::systems::Sensors">
    <render_engine>ogre2</render_engine>
    <background_color>1, 1, 1</background_color>
</plugin>
```

---

### Sensors

This plugin allows users to add many different sensors to their models all at once

This plugin supports the following sensors, implementations vary:
- `camera`: Camera
- `depth_camera`: Depth Camera
- `gpu_lidar`: lidar
- `rgbd_camera`: RGBD Camera
- `thermal_camera`: Thermal Camera
- `bounding_box`: Bounding Box Camera Sensor
- `segmentation`: Segmentation Camera

This plugin always takes in the following parameters:
- `<pose>` : What is the xyz, rpy of the sensor
- `<topic>` : Over what topic should the sensor put data
- `<update_rate>` : At what rate should the sensor give data feedback
- `<visualize>` : Do you want the sensor data to be visualized

# The sensor implementation below may be incorrect, needs to be checked

```
<sensor name="${name}" type='gpu_lidar'>
    <pose>0 0 ${body_height*0.6} 0 0 0</pose>
    <!-- <pose>0 0 1.0 0 0 0</pose> -->
    <topic>${name}/lidar</topic>
    <update_rate>10</update_rate>

    ... sensor specific implementation...

    <visualize>true</visualize>
    
    <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
        <render_engine>ogre</render_engine>
    </plugin>
</sensor>
```
### Acoustic Comms
A comms model that simulates communication using acoustic devices. The model uses simple distance based acoustics model.
This system can be configured with the following SDF parameters:
Optional parameters:
- <max_range>: Hard limit on range (meters). No communication will happen beyond this range. Default is 1000.
- <speed_of_sound>: Speed of sound in the medium (meters/sec). Default is 343.0
- <collision_time_per_byte> : If a subscriber receives a message 'b' bytes long at time 't0', it won't receiveand other message till time : 't0 + b * collision_time_per_byte'. Defaults to zero.
- <collision_time_packet_drop> : If a packet is dropped at time `t0`, the next packet won't be received until time `t0 + collision_time_packet_drop`. Defaults to zero.
- <propagation_model> : Enables the use of propagation model. Disabled by default.
- <source_power> : Source power at the transmitter in Watts. Defaults to 2 kW.
- <noise_level> : Ratio of the noise intensity at the receiver to the same reference intensity used for source level. Defaults to 1
- <spectral_efficiency> : Information rate that can be transmitted over a given bandwidth in a specific communication system, in (bits/sec)/Hz. Defaults to 7 bits/sec/Hz.
- <seed> : Seed value to be used for random sampling.
  
Here's an example:
```
 <plugin
  filename="gz-sim-acoustic-comms-system"
  name="gz::sim::systems::AcousticComms">
  <max_range>500</max_range>
  <speed_of_sound>1400</speed_of_sound>
  <collision_time_per_byte>0.001</collision_time_per_byte>
  <collision_time_packet_drop>0.001</collision_time_packet_drop>
  <propagation_model>
  <source_power>2000</source_power>
  <noise_level>1</noise_level>
  <spectral_efficiency>7</spectral_efficiency>
  </propagation_model>
 </plugin>
```

---

### Breadcrumbs

 A system for creating Breadcrumbs in the form of models that can get deployed/spawned at the location of the model to which this system is attached. Each breadcrumb is a complete sdf::Model. When deployed, the pose of the breadcrumb model is offset from the containing model by the pose specified in the `<pose>` element of the breadcrumb model. A name is generated for the breadcrumb by appending the current count of deployments to the name specified in the breadcrumb `<model>` element. The model specified in the `<breadcrumb>` parameter serves as a template for deploying multiple breadcrumbs of the same type. Including models from Fuel is accomplished by creating a `<model>` that includes the Fuel model using the `<include>` tag. See the example in examples/worlds/breadcrumbs.sdf.
  
  System Parameters
   - `<topic>`: Custom topic to be used to deploy breadcrumbs. If the topic is not set, the default topic with the following pattern would be used `/model/<model_name>/breadcrumbs/<breadcrumb_name>/deploy`. The topic type is ignition.msgs.Empty
   - `<max_deployments>`: The maximum number of times this breadcrumb can be deployed. Once this many are deployed, publishing on the deploy topic will have no effect. If a negative number is set, the maximum deployment will be unbounded. If a value of zero is used, then the breadcrumb system will be disabled. A zero value is useful for situations where SDF files are programmatically created. The remaining deployment count is available on the `<topic>/remaining` topic.
   - `<disable_physics_time>`: The time in which the breadcrumb entity's dynamics remain enabled. After his specified time, the breadcrumb will be made static. If this value is <= 0 or the param is not specified, the breadcrumb model's dynamics will not be modified.
   - `<performer_volume>`: Geometry that represents the bounding volume of the performer. Only `<geometry><box>` is supported currently. When this parameter is present, the deployed models will be performers.
   - `<allow_renaming>`: If true, the deployed model will be renamed if another model with the same name already exists in the world. If false and there is another model with the same name, the breadcrumb will not be deployed. Defaults to false.
   - `<breadcrumb>`: This is the model used as a template for deploying breadcrumbs.
   - `<topic_statistics>`: If true, then topic statistics are enabled on `<topic>` and error messages will be generated when messages are dropped. Default to false.

---

### Camera Video Recorder

Record video from a camera sensor
The system takes in the following parameters:
  <service>  Name of topic for the video recorder service. If this is not specified, the topic defaults to:
               /world/<world_name/model/<model_name>/link/<link_name>/sensor/<sensor_name>/record_video
  <use_sim_time> True/false value that specifies if the video should be recorded using simulation time or real time. The default is false, which indicates the use of real time.
  <fps> Video recorder frames per second. The default value is 25, and the support type is unsigned int.
  <bitrate> Video recorder bitrate (bps). The default value is 2070000 bps, and the supported type is unsigned int.

---

### Comms Endpoint

A system that registers in the comms broker an endpoint. You're creating an address attached to the model where the plugin is running. The system will bind this address in the broker automatically for you and unbind it when the model is destroyed.
The endpoint can be configured with the following SDF parameters:
  * Required parameters:
  <address> An identifier used to receive messages (string).
  <topic> The topic name where you want to receive the messages targeted to this address.
   
  * Optional parameters:
  <broker> Element used to capture where are the broker services. This block can contain any of the next optional parameters:
  <bind_service>: Service name used to bind an address. The default value is "/broker/bind"
  <unbind_service>: Service name used to unbind from an address. The default value is "/broker/unbind"

  Here's an example:
  ```
   <plugin
     filename="ignition-gazebo-comms-endpoint-system"
     name="ignition::gazebo::systems::CommsEndpoint">
     <address>addr1</address>
     <topic>addr1/rx</topic>
     <broker>
       <bind_service>/broker/bind</bind_service>
       <unbind_service>/broker/unbind</unbind_service>
     </broker>
   </plugin>
```

---

### Contact

Contact sensor system which manages all contact sensors in simulation.

Here's an example:
```
 <?xml version="1.0" ?>
 <sdf version="1.6">
   <world name="default">
     <!-- ... other world properties ... -->
 
     <!-- Include your model with collision elements -->
     <include>
       <uri>model://your_model</uri>
     </include>
 
     <!-- Contact plugin configuration -->
     <plugin name="contact_plugin" filename="libContactPlugin.so">
       <!-- Name of the model containing the collision elements -->
       <model_name>your_model</model_name>
 
       <!-- Name of the collision element you want to monitor -->
       <collision_name>your_collision</collision_name>
     </plugin>
   </world>
 </sdf>
```

In this Example:
 <include>: Include the model that you want to monitor for collisions. Replace "your_model" with the actual name of your model.
 <plugin>: Define the ContactPlugin with specific configuration parameters.
 name: A unique name for your plugin instance.
 filename: The filename of the shared library containing your ContactPlugin code (replace "libContactPlugin.so" with the actual filename).
 <model_name>: Specify the name of the model that contains the collision element you want to monitor. Replace "your_model" with the actual name of your model.
 <collision_name>: Specify the name of the collision element you want to monitor. Replace "your_collision" with the actual name of your collision element.

### Detachable Joint
A system that initially attaches two models via a fixed joint and allows for the models to get detached during simulation via a topic. A model can be re-attached during simulation via a topic. The status of the detached state can be monitored via a topic as well.

Parameters:

   - `<parent_link>`: Name of the link in the parent model to be used in creating a fixed joint with a link in the child model.
   - `<child_model>`: Name of the model to which this model will be connected
   - `<child_link>`: Name of the link in the child model to be used in creating a fixed joint with a link in the parent model.
   - `<topic>` (optional): Topic name to be used for detaching connections. Using <detach_topic> is preferred.
   - `<detach_topic>` (optional): Topic name to be used for detaching connections. If multiple detachable plugin is used in one model, `detach_topic` is REQUIRED to detach child models individually.

  - `<attach_topic>` (optional): Topic name to be used for attaching connections. If multiple detachable plugin is used in one model, `attach_topic` is REQUIRED to attach child models individually.
  - `<output_topic>` (optional): Topic name to be used for publishing the state of the detachment. If multiple detachable plugin is used in one model, `output_topic` is REQUIRED to publish child models state individually.
  - `<suppress_child_warning>` (optional): If true, the system will not print a warning message if a child model does not exist yet. Otherwise, a warning message is printed. Defaults to false.
    
---

### Follow Actor
 Make an actor follow a target entity in the world.
 SDF parameters
 <target>: Name of entity to follow.
 <min_distance>: Distance in meters to keep from target's origin.
 <max_distance>: Distance in meters from target's origin when to stop following. When the actor is back within range it starts following again.
 <velocity>: Actor's velocity in m/s
 <animation>: Actor's animation to play. If empty, the first animation in the model will be used.
 <animation_x_vel>: Velocity of the animation on the X axis. Used to coordinate translational motion with the actor's animation.

---

### Force Torque
This system manages all Force-Torque sensors in simulation. Each FT sensor reports readings over Ignition Transport.
Regardless of the setting of //sensor/force_torque/frame the pointof application of the force is at the sensor's origin. /sensor/force_torque/frame only changes the coordinate frame in which the quantities are expressed, not the point of application.

---

### IMU
This system manages all IMU sensors in simulation. Each IMU sensor reports vertical position, angular velocity, and linear acceleration readings over Ignition Transport.

Example:
```
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="your_model">
    <!-- ... other model properties ... -->

    <!-- Include the IMU sensor -->
    <sensor name="imu_sensor" type="imu">
      <!-- IMU sensor properties -->
      <always_on>true</always_on>
      <update_rate>100.0</update_rate>
      <topic>~/your_model/imu</topic>
    </sensor>

    <!-- Link where the IMU sensor is attached -->
    <link name="your_link">
      <!-- ... link properties ... -->

      <!-- Attach the IMU sensor to the link -->
      <sensor name="imu_sensor" type="imu">
        <!-- IMU sensor properties -->
        <always_on>true</always_on>
        <update_rate>100.0</update_rate>
        <topic>~/your_model/imu</topic>
      </sensor>
    </link>

    <!-- ... other model elements ... -->
  </model>
</sdf>
```

In this Example:
<sensor name="imu_sensor" type="imu">: Define an IMU sensor with specific properties.

<always_on>: Set to true to ensure that the sensor is always active.
<update_rate>: Specifies the update rate in Hz for the sensor.
<topic>: Defines the ROS topic where the IMU data will be published. Replace "~/your_model/imu" with your desired topic name.
<link name="your_link">: Identify the link in your model where the IMU sensor is attached.

Inside the link element, you attach the IMU sensor with the same properties as defined above.

---

### Log Video Recorder
 System which recordings videos from log playback. There are two ways to specify what entities in the log playback to follow and record videos for: 1) by entity name and 2) by region. See the
 following parameters:
  - `<entity>`         Name of entity to record.
  - `<region>`         Axis-aligned box where entities are at start of log
  + `<min>` Min corner position of box region.
  + `<max>`  Max corner position of box region.
  - `<start_time>`     Sim time when recording should start
  - `<end_time>`       Sim time when recording should end
  - `<exit_on_finish>` Exit ign-gazebo when log playback recording ends
    
  When the recording is finished. An `end` string will be published to the `/log_video_recorder/status` topic and the videos are saved to a timestamped directory

  ---
### Logical Camera
A logical camera sensor that reports objects detected within its frustum readings over ign transport

Example:
```
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="your_model">
    <!-- ... other model properties ... -->

    <!-- Include the logical camera sensor -->
    <sensor name="logical_camera_sensor" type="logical_camera">
      <!-- Logical camera sensor properties -->
      <always_on>true</always_on>
      <update_rate>10.0</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
      </camera>
      <plugin name="your_logical_camera_plugin" filename="libYourLogicalCameraPlugin.so">
        <!-- Custom plugin parameters (if applicable) -->
      </plugin>
    </sensor>

    <!-- ... other model elements ... -->
  </model>
</sdf>
```
In this Example:
`<sensor name="logical_camera_sensor" type="logical_camera">`: Define a logical camera sensor with specific properties.

<always_on>: Set to true to ensure that the sensor is always active.
<update_rate>: Specifies the update rate in Hz for the sensor.
<camera>: Define camera-specific properties.
<horizontal_fov>: Set the horizontal field of view for the camera.
<image>: Define the image properties (width and height) of the camera's output.
`<plugin name="your_logical_camera_plugin" filename="libYourLogicalCameraPlugin.so">`: Include a custom logical camera plugin, if needed. This is where you can specify the plugin name and the shared library filename for your custom logic.

--- 
### Navsat
System that handles navigation satellite sensors, such as GPS, that reports position and velocity in spherical coordinates (latitude / longitude) over Ignition Transport.
The NavSat sensors rely on the world origin's spherical coordinates being set, for example through SDF's `<spherical_coordinates>` tag or the `/world/world_name/set_spherical_coordinates` service.

---


---

[comment]: <> (### Triggered Publisher)
