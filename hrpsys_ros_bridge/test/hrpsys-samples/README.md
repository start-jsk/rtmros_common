Examples
========================================================================================================================

This page shows basic example for hrpsys controller using [SampleRobot](https://github.com/fkanehiro/hrpsys-base/blob/master/sample/SampleRobot).  
This page is an Euslisp example for [hrpsys-base example](https://github.com/fkanehiro/hrpsys-base/blob/master/sample/SampleRobot).

To learn more about API, please refer to [API DOC in hrpsys-base](http://fkanehiro.github.io/hrpsys-base/).

# samplerobot-data-logger.l
DataLogger is hrpsys-base RTC for data logging of hrpsys-base RTCs 
1. Launch hrpsys-simulator

    ```
    rtmlaunch hrpsys_ros_bridge samplerobot.launch
    ```
2. Euslisp example

    ```
    roseus `rospack find hrpsys_ros_bridge`/test/hrpsys-samples/samplerobot-data-logger.l
    irteusgl$ (samplerobot-data-logger-demo)
    ```

# samplerobot-remove-force-offset.l
RemoveForceSensorLinkOffset is hrpsys-base RTC to remove hands or feet from force sensor values. 
1. Launch hrpsys-simulator

    ```
    rtmlaunch hrpsys_ros_bridge samplerobot.launch USE_UNSTABLE_RTC:=true
    ```
2. Euslisp example

    ```
    roseus `rospack find hrpsys_ros_bridge`/test/hrpsys-samples/samplerobot-remove-force-offset.l
    irteusgl$ (samplerobot-remove-force-offset-demo)
    ```

# samplerobot-impedance-controller.l
ImpedanceController is hrpsys-base RTC for cartesian impedance control. 
1. Launch hrpsys-simulator

    ```
    rtmlaunch hrpsys_ros_bridge samplerobot.launch USE_UNSTABLE_RTC:=true
    ```
2. Euslisp example

    ```
    roseus `rospack find hrpsys_ros_bridge`/test/hrpsys-samples/samplerobot-impedance-controller.l
    irteusgl$ (samplerobot-impedance-controller-demo)
    ```

# samplerobot-auto-balancer.l
AutoBalancer is hrpsys-base RTC to generate walking pattern and control Center Of Gravity for legged robots without sensor feedback. 
1. Launch hrpsys-simulator

    ```
    rtmlaunch hrpsys_ros_bridge samplerobot.launch USE_UNSTABLE_RTC:=true
    ```
2. Euslisp example

    ```
    roseus `rospack find hrpsys_ros_bridge`/test/hrpsys-samples/samplerobot-auto-balancer.l
    irteusgl$ (samplerobot-auto-balancer-demo)
    ```
    <div align="center"><p><img src="https://raw.githubusercontent.com/fkanehiro/hrpsys-base/master/sample/SampleRobot/img/ABC.png" alt="AutoBalancer" title="AutoBalancer" width=300/></p></div>  

# samplerobot-stabilizer.l
1. Launch hrpsys-simulator

    ```
    export BUSH_CUSTOMIZER_CONFIG_PATH=`rospack find openhrp3`/../OpenHRP-3.1/customizer/sample1_bush_customizer_param.conf
    rtmlaunch hrpsys_ros_bridge samplerobot.launch hrpsys_precreate_rtc:=PDcontroller USE_UNSTABLE_RTC:=true PROJECT_FILE:=`rospack find hrpsys`/share/hrpsys/samples/SampleRobot/SampleRobot.500.torque.xml
    ```
2. Euslisp example

    ```
    roseus `rospack find hrpsys_ros_bridge`/test/hrpsys-samples/samplerobot-stabilizer.l
    irteusgl$ (samplerobot-stabilizer-demo)
    ```

# samplerobot-carry-object.l
This is example to carry objects maintaining the balance using ImpedanceController, Auto-Balancer, Stabilizer and another following program.  
By the additional controller, robot's position of the COG(center of gravity) is determined so that the gravity acting on the robot and reaction force from the object are balanced.  
This program checks the saturation of reaction force measured by force sensor and determines the reference force (force value used in calculation of the Reference COG).  

This example includes following three demos.

    Lift 5kg weight with both arms and walk.
    Lift 5kg weight with one hand and walk.
    Push 20kg weight (coefficient of dynamic friction is 0.5). 

1. Launch hrpsys-simulator

    ```
    export BUSH_CUSTOMIZER_CONFIG_PATH=`rospack find openhrp3`/../OpenHRP-3.1/customizer/sample1_bush_customizer_param.conf
    rtmlaunch hrpsys_ros_bridge samplerobot.launch hrpsys_precreate_rtc:=PDcontroller USE_UNSTABLE_RTC:=true PROJECT_FILE:=`rospack find hrpsys`/share/hrpsys/samples/SampleRobot/SampleRobot.carryobject.xml USE_OBJECTCONTACTTURNAROUNDDETECTOR:=true
    ```
2. Euslisp example

    ```
    roseus `rospack find hrpsys_ros_bridge`/test/hrpsys-samples/samplerobot-carry-object.l
    irteusgl$ (samplerobot-carry-object-demo)
    ```
<div align="center"><p>
 <img src="https://raw.githubusercontent.com/fkanehiro/hrpsys-base/master/sample/SampleRobot/img/DualarmCarry.png" alt="DualarmCarry" title="DualarmCarry" height=200/>
 <img src="https://raw.githubusercontent.com/fkanehiro/hrpsys-base/master/sample/SampleRobot/img/SinglearmCarry.png" alt="SinglearmCarry" title="SinglearmCarry" height=200/>
 <img src="https://raw.githubusercontent.com/fkanehiro/hrpsys-base/master/sample/SampleRobot/img/DualarmPush.png" alt="DualarmPush" title="DualarmPush" height=200/>
</p></div>


# Terrain Walk examples
This is terrain walk examples to use AutoBalancer. In this section, slope walking, stair climbing-up, and stair climbing-down are introduced. 

## Example for slope walking
1. Launch hrpsys-simulator
    ```
    rtmlaunch hrpsys_ros_bridge samplerobot.launch PROJECT_FILE:=`rospack find hrpsys`/share/hrpsys/samples/SampleRobot/SampleRobot.TerrainFloor.SlopeUpDown.xml
    ```
2. Euslisp example
    ```
    roseus `rospack find hrpsys_ros_bridge`/test/hrpsys-samples/samplerobot-terrain-walk.l
    irteusgl$ (demo-slope-updown) ;; currently, robot falls down.
    ```
    <div align="center"><p><img src="https://raw.githubusercontent.com/fkanehiro/hrpsys-base/master/sample/SampleRobot/img/Slope.png" alt="Slope" title="Slope" width=300/></p></div>  

## Example for stair climbing-up
1. Launch hrpsys-simulator
    ```
    rtmlaunch hrpsys_ros_bridge samplerobot.launch PROJECT_FILE:=`rospack find hrpsys`/share/hrpsys/samples/SampleRobot/SampleRobot.TerrainFloor.StairUp.xml
    ```
2. Euslisp example
    ```
    roseus `rospack find hrpsys_ros_bridge`/test/hrpsys-samples/samplerobot-terrain-walk.l
    irteusgl$ (demo-stair-up)
    ```
    <div align="center"><p><img src="https://raw.githubusercontent.com/fkanehiro/hrpsys-base/master/sample/SampleRobot/img/StairUp.png" alt="StairUp" title="StairUp" width=300/></p></div>  

## Example for stair climbing-down
1. Launch hrpsys-simulator
    ```
    rtmlaunch hrpsys_ros_bridge samplerobot.launch PROJECT_FILE:=`rospack find hrpsys`/share/hrpsys/samples/SampleRobot/SampleRobot.TerrainFloor.StairDown.xml
    ```
2. Euslisp example
    ```
    roseus `rospack find hrpsys_ros_bridge`/test/hrpsys-samples/samplerobot-terrain-walk.l
    irteusgl$ (demo-stair-down)
    ```
    <div align="center"><p><img src="https://raw.githubusercontent.com/fkanehiro/hrpsys-base/master/sample/SampleRobot/img/StairDown.png" alt="StairDown" title="StairDown" width=300/></p></div>  


# Starting for DRC Testbed Model Simulation
1. Launch hrpsys-simulator
    ```
    rtmlaunch hrpsys_ros_bridge samplerobot.launch PROJECT_FILE:=`rospack find hrpsys`/share/hrpsys/samples/SampleRobot/SampleRobot.DRCTestbed.xml
    ```
    <div align="center"><p><img src="https://raw.githubusercontent.com/fkanehiro/hrpsys-base/master/sample/SampleRobot/img/DRCTestbed.png" alt="DRC Testbed" title="DRC Testbed" height=300/></p></div>  
