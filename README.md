# OmniWhegPro
## Overview
这个项目在原OmniWheg的基础上，增加了一些功能，使其更加完善。主要的改进有：
1. 更换了新的电机，使用了更加强劲的电机，使得机器人的承重能力更强，终于可以爬楼梯了。
2. 更换了新的舵机，使得机器人的wheg机械结构的展开更容易了。
3. 增加了一台NUC作为主控，并实现了用ros2来控制机器人。
4. 增加了一个激光雷达和RGBD相机，并完成了测试。
5. 完成了基础的控制接口的代码实现，可以通过ros2以及遥控器来控制机器人的运动。
接下来我将分别介绍机器人的各个部分。
## Structure
机器人主体用铝合金管以及三角铁拼接而成，轮子以及连接部分用3D打印的材料制作，总体不是很坚固，未来应考虑用焊接以及加工件的方式提升整体的强度。
## Motor
电机使用了大功率的无刷电机，具体参数参考[电机资料](doc/ref/SDM11064W%E7%94%B5%E6%9C%BA%E8%B5%84%E6%96%99)。其通讯方式为CAN总线，具体协议写在资料中。值得注意的是，电机的初始电流限制较小，导致机器人爬楼梯时很难爬上去，在配置完这个参数后，机器人可以轻松爬上楼梯。
## Servo
舵机是由PWM控制的，采用无反馈的开环控制，所以上位机不知道轮子展开的具体情况。舵机的参数参考[舵机资料](doc/ref/舵机data%20sheet.html)。因为舵机的工作电压为12V，但Robomaster A型板只能提供5V的电压，所以其工作电压为单独供电，而信号线接在A型板上。有时因为这个的原因需要把供电段的地线和A型板的地线连接起来，否则会出现信号不稳定的情况。
## NUC
NUC的密码是natsu。上位机使用了ros2 humble作为代码的基础框架，实现了用UART串口通讯（cp210x转换器，就插在NUC上）。代码主要有两个package，一个是uart_converter,接受串口数据并发布到ros2的topic上，另一个是uart_sender，发送数据到开发板上控制机器人运动。
## Sensors
机器人上有一个激光雷达和一个RGBD相机，激光雷达使用的是Hokuyo，RGBD相机使用的是Intel Realsense D435i。相关的驱动已经安装好并测试过了，可以直接使用。
## Control
### Remote Control
机器人的遥控器使用的是Robomaster的遥控器，通过串口通讯来控制机器人的运动。左侧摇杆的左右控制机器人的方向，上下为特殊功能。调到最下方会校准机器人同向两个轮子的相位，以便爬楼梯（在机器人悬空的情况下用），调上目前没有功能。右侧摇杆的左右控制机器人的左右平移，上下控制机器人的前后平移。摇杆上方的拨杆控制机器人轮子的展开情况。拨杆向上时，轮子展开，向下时轮子收起。拨杆中间时，机器人处于上位机控制状态，不响应遥控器的指令。
### ROS2 Control
机器人的ROS2控制使用的是uart_sender这个package，具体通讯协议很简单：发一个数组到开发板，前4位控制舵机，后四位控制轮子的速度。具体情况建议看代码，很短。

## Conclusion & TODO
很遗憾我没能把所有的工作做完，留下了这样一个“烂尾”的项目。但是我还是想总结一下这个项目的收获和不足。目前来说，机器人的主要功能（爬楼梯）已经完成，但因为整体强度不足的原因，无法做太多测试。机器人的控制方式也比较简单，只有遥控器和上位机，设想中的自动避障以及导航功能都还没有完成。如果以发表的角度考虑的话，机器人的外观还需要重新设计包装一下，现在太丑了，完全拍不了视频和照片。然后设想中的姿态平衡功能也还没有完成，目前的重量对于舵机来说似乎有点太重了。总的来说，这个项目还有很多需要改进的地方，但是我还是很开心可以做出这样一个机器人，也特别感谢Soren老师在我转过来之后一直支持我做它。

------------------------ ENGLISH -------------------------------------

# OmniWhegPro
## Overview
This project builds upon the original OmniWheg design by adding new features to make it more sophisticated. The main improvements include:

Replacement of stronger motors, enhancing the robot's load-bearing capacity, finally enabling it to climb stairs.
Upgrading of new servos, facilitating the expansion of the wheg mechanism.
Integration of an NUC as the main controller, utilizing ROS 2 for robot control.
Incorporation of a laser radar and RGBD camera, both of which have been successfully tested.
Implementation of foundational control interfaces, enabling robot movement control through ROS 2 and a remote controller.
Next, I will provide individual explanations for the various components of the robot.

## Structure
The robot's main body is constructed using aluminum alloy pipes and triangular brackets. The wheels and connecting parts are fabricated using 3D-printed materials. The overall structure is not extremely robust; future enhancements should consider methods such as welding and additional processing to increase overall strength.

## Motor
High-powered brushless motors are employed for the robot, with specific parameters available in the [Motor Data](doc/ref/SDM11064W Motor Data). These motors communicate via a CAN bus, with detailed protocols outlined in the provided documentation. Notably, initial current limits on the motors were too low, making it difficult for the robot to climb stairs. After configuring this parameter, the robot can ascend stairs with ease.

## Servo
Servos are controlled via PWM and utilize open-loop control without feedback. As a result, the upper-level system lacks precise knowledge of the wheg mechanism's state. Servo specifications are detailed in the [Servo Data](doc/ref/Servo Data Sheet). Due to the servo's operational voltage being 12V and the Robomaster A-type board only providing 5V, a separate power supply is used for the servos, while the signal line connects to the A-type board. Occasionally, grounding the power supply segment and the A-type board is necessary to prevent signal instability.

## NUC
The password for the NUC is "natsu". The upper-level system utilizes ROS 2 Humble as the foundational framework for the code. UART serial communication is established using a cp210x converter plugged into the NUC. The code comprises two primary packages: "uart_converter," which receives serial data and publishes it to ROS 2 topics, and "uart_sender," which transmits data to the development board to control robot movement.

## Sensors
The robot is equipped with a laser radar and an RGBD camera. The laser radar is a Hokuyo model, while the RGBD camera is an Intel Realsense D435i. Relevant drivers have been installed, tested, and are ready for direct use.

## Control
### Remote Control
The robot's remote control utilizes Robomaster's remote controller, employing serial communication for robot control. The left joystick controls left and right robot direction, with up and down controlling special functions. Moving the joystick to the lowest position calibrates the phase of the two wheels in the robot's forward direction, aiding in stair climbing (used when the robot is suspended). Moving the joystick to the uppermost position currently serves no function. The right joystick's left and right movement controls lateral robot translation, while up and down control forward and backward translation. A toggle switch above the joystick controls the expansion of the robot's wheels. Upward toggle expands the wheels, while downward toggle retracts them. In the middle position, the robot is under upper-level control and does not respond to remote controller commands.

### ROS2 Control
The robot's ROS 2 control employs the "uart_sender" package, utilizing a straightforward communication protocol: sending an array to the development board, with the first four elements controlling servos and the latter four elements controlling wheel speeds. For precise details, reference the code, which is succinct.

### Conclusion & TODO
Regrettably, I couldn't complete all the planned work, leaving this project in an unfinished state. Nevertheless, I'd like to summarize the project's accomplishments and shortcomings. Currently, the robot's primary functionality (climbing stairs) has been achieved. However, due to overall structural weaknesses, extensive testing is challenging. The robot's control methods are also relatively simple, limited to the remote controller and upper-level system. The envisioned obstacle avoidance and navigation capabilities remain unfinished. From a publishing standpoint, the robot's appearance needs redesigning for better aesthetics; currently, it's unattractive and unsuitable for photography or videography. Additionally, the anticipated posture balancing function remains incomplete, as the current weight appears excessive for the servos. In essence, this project has room for significant improvement. Nonetheless, I'm pleased to have created such a robot and extend gratitude to Soren for supporting me throughout this project since my transfer to the MARS lab. While I didn't achieve the publication goal, I've still learned a great deal from this experience.