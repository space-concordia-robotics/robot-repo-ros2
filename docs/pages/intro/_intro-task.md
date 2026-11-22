# Introduction Software Task

# ==Old Intro Task==

## Your Starter Checklist

To help you get up and running smoothly, here's a list of foundational topics to explore and become comfortable with:

- [ ] **Git & GitHub Basics**\
  Learn essential Git commands:\
  `git fetch`, `git pull`, `git commit`, `git push`, `git switch`, `git log`, etc.
- [ ] **Linux Command Lines**\
  Get familiar with:
    - `cd`, `ls`, `mkdir`, `code .`
    - How to source your ROS workspace in `.bashrc`
    - Creating and managing ROS 2 workspaces and packages
- [ ] **ROS 2 Fundamentals**\
  Understand the core structure and purpose of ROS 2\. Learn how it enables distributed robot systems.
- [ ] **Nodes & Topics**\
  These are the building blocks of ROS-based applications. Get to know how they work and interact.
- [ ] **The Rover**\
  Finally, start learning about the rover itself&mdash;its architecture, components, and how you'll interface with it through code and testing. Hands-on time is
  highly encouraged!

## Getting Started

There are many sources you can use in order to get the ball rolling on your learning path. To get you started, here are a couple of documents/videos you could
look at to familiarize yourself with ROS2:

### Videos

This is a great [starter course](https://concordia.udemy.com/course/ros2-for-beginners/) on everything there is to know about ROS2 basics \[Concordia has a
partnership with udemy therefore, you can watch all of this for free!\])

- [Another good udemy course on getting started with ROS2](https://www.udemy.com/course/ros2-how-to)
- [Video on how to download UTM \[if on mac\]](https://www.youtube.com/watch?v=nUhQy5PDj2A)

### Documentation

- [This website](../resources/index.md)
- [ROS2 Cookbook (A General Guide)](https://fer.gs/ros2_cookbook/client_libraries/index.html)
- [ROS2 Jazzy Concept Documention](https://docs.ros.org/en/jazzy/Concepts/Basic.html)
- [ROS2 Jazzy Ubuntu Installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) (ROS2 jazzy download documentation)
- [Ubuntu 24.04 download](https://releases.ubuntu.com/noble/) (select the *desktop image*)
- [Oracle Virtualbox](https://www.virtualbox.org/) (Good for virtualizing linux on your machine)
- [Github Command Cheat Sheet](https://education.github.com) (Good Refresher)

## Your First Task: GitHub + ROS2 Packages

To help you get hands-on and reinforce the basics, we'd like you to complete this introductory task.
It will give you practice with ROS2, GitHub, and both C++ and Python development.
Feel free to use any of the videos/documentation found above!

**Here's what you need to do:**

1. **Create a new GitHub repository**\
   This is where you'll upload your code for this task and future personal practice. Make it public or private &mdash; up to you!
2. **Set up a new ROS2 workspace**\
   Use either your virtual machine or your local Linux setup (Ubuntu 24.04). Make sure the workspace builds and sources correctly.
3. **Inside that workspace, create two packages:**
    - A **Python** package (using `ament_python`)
    - A **C++** package (using `ament_cmake`)
4. **Implement a simple publisher and subscriber in each package**
    - You could do anything you'd like! For example: have the publisher send a string like `"Hello from ROS2!"` and have the subscriber print it to the
      terminal.
5. **Build your workspace and test your nodes**\
   Make sure they work using `ros2 run` or by launching them together.
6. **Push everything to your GitHub repo**\
   Be sure to commit your `src/`, `CMakeLists.txt`, `package.xml`, and any launch or config files you make.

If you're ever unsure about anything, don't hesitate to ask questions &mdash; everyone here started somewhere, and we're all learning together.

## **Next Steps & Communication**

- **Share Your Work with Us**
    - Send the link to your GitHub repository to both Rayan Raad and Micheal Lythgoe
    - Don't be afraid to communicate with either of us for help or guidance
- **Get Feedback**
    - We will review your repository, run your code, and give you feedback.
    - If any issues come up, we'll guide you through fixing them
- **Choose Your First Intro Task**
    - After approval, we'll assign or suggest an **introductory task** (from our current task list).
    - This will include either more integrated work with the rover, or something that will be actively used by the rover pilots on our next test!
- **Stay in Touch**
    - Always let us know in the team chat when:
        - You've completed a task.
        - You're stuck and need help.

----

# ==New intro task==

<!-- TODO 2026-11-22 (Will Free): tweak this & clean it up -->

## Introduction

A Space Concordia Robotics software member should learn the software skills presented in this document.
These skills include Terminal CLI, Git, ROS2, Linux, C++, Python, CAN, Networking and Bash.
Upon becoming a software member, they will contribute to the software development of one of the team's projects.
These projects include the Arm, Autonomy, Drone, Mobility & Frame, Power, Science and Telemetry, Tracking & Control projects.
They are also a driving force for making sure that mission testing is possible.
This is because the software is what implements the behaviour and control of the rover.
A software member usually has the skills to write clean, efficient and functional code and will need to learn the mechanical,
electrical and scientific knowledge involved in the implementation in the rover.
Their software skills should aid in the correct implementation of the intentions of the various disciplines.

## Skills

The skills presented in this section are not necessary to master. A basic understanding of the presented skills, as well as exposure to and curiosity about
these subjects will allow for a member to interface with the rover in order to develop and debug its behaviour. These skills take years to learn, so it is
important to keep an open mind in order to acquire new knowledge and find new solutions to problems you may already know how to solve.

### Linux

The rover has an Nvidia Jetson which takes care of the high level computing. It runs the Ubuntu 24.04 operating system. Ubuntu is a Linux based operating system
which ROS2 is mainly developed on top of. Ideally, a software member would have access to a PC which runs Ubuntu 24.04. This being said, there are ways to do
ROS2 development on common operating systems like Windows and MacOS. These alternatives are not always easy to work with because extra pipelines must be created
in order to access hardware and networks that are connected to your PC. On Windows and MacOS, the recommended solutions are WSL and UTM, respectively.
Understanding the Linux file structures and features is very important for developing the rover software. Every operating system has the terminal and secure
shell (SSH) functionalities. Together they can be used to connect to an Ubuntu computer which is connected to your network. This however will generally require
you to be able to utilize the terminal through Command Line Interfaces (CLI).

### Terminal CLI

Upon opening a terminal, a user will only be able to navigate through text-based commands. These commands, called command line interfaces, allow the user to
interface with a computer system through lines of text which map to a command on that system. The rover's Nvidia Jetson is the computer system that a software
member will need to interface with. There are common prebuilt commands that originate from the terminal and the operating system. They are used to navigate and
modify the Linux file system and execute the available functionalities. Extra commands can be installed onto your device through the command line in order to
gain extra functionality. Learning how to use CLIs is very important when developing, modifying and executing ROS2 packages.

### ROS2

A functionality, where CLI is heavily used, is ROS2. It stands for Robot Operating System, however it is not an OS, it is a software framework for communication
within a robotic system. SCRB currently uses ROS2 Jazzy which is supported on the Ubuntu 24.04 Linux flavor. There are some fundamental features, which are
native to ROS2, that a Software member should be familiar with. These are Nodes, Topics, Messages, Services, Actions, Packages and Launch files. These features
are utilized through CLIs.

#### Nodes

A ROS2 node is a program which performs a specific task and is defined with either C++ or Python code. In a ROS2 workspace, there are multiple nodes running at
the same time. All of these nodes can communicate with each other through topics, services and actions. These communication structures are defined within the
code of the node.

#### Topics

A ROS2 topic is a stream of data which is published/generated by a node. They have unique identifiers and data structures. Using these two topic attributes,
another node can subscribe/listen to the data stream.

#### Messages

A ROS2 message is a data structure which is built from other message types. There are standard messages which are the primitive data structures and are not
built from other message types. These messages are the defined data structures in a ROS2 topic.

#### Services

A ROS2 service is a connection between two nodes, a server and a client. Here a client will request a service from the server. Upon receiving the request, the
server will perform a quick operation and return a response to the client. Analogously to a function, a request is like the input values to a function, the
output value is the response and the service is said function. When using a service it is usually expected for the response to arrive almost instantaneously
after sending a request.

#### Actions

A ROS2 action is a connection between two nodes for running operations which are expected to run for a longer period of time. Once again, there is a server and
a client node. When the client sends a request and goal, the server, which received the request, will send out an immediate response indicating that the request
was received and that the server has begun completing the operation task. As the server is running the operation, it will continuously send back to the client a
message with the status of the operation. Eventually, the server completes the tasks and sends the results of the operation, as well as a response to the client
indicating that the operation was completed.

#### Launch Files

All of the nodes can be called through the CLI. However, this can become very tedious when increasing the number of nodes and setting many parameters for each
one of them. The solution for this are launch files. You can run all of your desired nodes through a file containing their descriptions. It can then be invoked
through the CLI with a single command.

#### Packages

A ROS2 package is a set of files where nodes, topics, messages, services, actions, launch files and configuration files are defined. It makes it easier to
organize and distribute ROS2 software functionalities. Usually a package will encapsulate a certain functionality, this allows for a ROS2 workspace to have the
freedom of utilising the packages that are needed instead of having one package that does everything.

### Python

The Python programming language is used in ROS2 to define launch files and to program nodes. The ROS Python client library (rclpy) is used to create a desired
node architecture. Python has the benefit of being beginner friendly, having a large library ecosystem and enabling quick prototyping and development.

### C++

The Python programming language is used in ROS2 to program nodes. The ROS C++ client library (rclcpp) is used to create a desired node architecture. C++ has the
benefit of being very fast and having very easy memory management. This is a more difficult language for a new programmer to learn. When it comes to development
in ROS2, the developer should decide which language to use depending on the goal of the node that is being developed.

### Bash

Bash stands for Bourne Again Shell, and is a command-line shell and programming language. It allows you to create scripts for interacting with a computer
system. A bash file is populated by CLI commands and bash control flow. These two features are very powerful together as it allows for the automation of many
tasks. The first line in any Bash file is: \#\!/bin/bash.

### Networking

During the development, testing and deployment of the software of the rover, the team uses networking infrastructure to connect and communicate with the rover.
Usually, the Local Area Network (LAN) involves connecting the rover and base station computer to a router which manages the data of the network. A device would
connect itself to a network through access points. These access points can be wireless or physical. The router used by the team supports wireless and wired
access through wifi and ethernet cables respectively. The team also utilizes radio communication in order to communicate with the rover from at most 2km away.
Within the hardware of the rover there is a Control Area Network (CAN) which connects the rover's computer to all of its actuators and sensors.

#### LAN

LAN is a network which connects multiple computers together. This is the type of network that is used by the team. A DHCP, living in the team's router,
configures the network such that a user can connect itself to the network. Once the user's device is connected to the LAN it can communicate with the radios,
the rover computer and other IP devices. Each device has a MAC address which is a unique identifier of the physical device. The network configuration maps IP
addresses to a MAC address which allows for the flow of information through the network. This is used to connect the ROS2 data from multiple devices, to stream
video data and secure shell (SSH) to another device. SSH allows for a user to access another device's terminal from the user's device through the network.

#### Radio

The team currently uses a 5.8GHz radio frequency system to communicate with the rover from the base station. A physical radio is attached to a pole at the base
station and on the rover. The basestation physical radio also houses its antenna, however the rovers radio is connected to a 5.8GHz antenna through coaxial
cables. Through ethernet cables these radios are connected to the LAN and create a wireless connection between both radios. The radio connections can be
modified through the web interfaces of the radios, which are accessed through their respective IP addresses in the network.

#### CAN

Within the whole rover system, custom PCBs can be found which communicate with each other and the jetson through a CAN network. Each PCB is created to implement
a functionality, usually controlling actuators or reading sensors. They are all physically connected to the CAN bus through the use of two wires, CAN low and
CAN high. The can bus constantly is inhabited by CAN frames which are composed of CAN IDs and CAN payloads and are encoded in bits. The CAN ID of a frame
represents the type of frame that is being sent. This allows for the other devices that are connected to the network to identify whether a frame is of interest
to them. It also allows for a device to parse the frame payload appropriately. The frame payload is the data that is being transmitted by the frame. A CAN
architecture is created by the team in order to manage the needed frames of the network.

### Git

Git is utilized to manage the software written by the team. It is a version control system which keeps track of the changes made in a project. This allows for
the team to always have a functioning version of the codebase, while also improving it. Git is utilized through its CLI and is connected to data that is stored
in GitHub. The team has a GitHub organization which contains the repositories of the rover project.

#### Organization

The GitHub organization of the team is [space-concordia-robotics](https://github.com/space-concordia-robotics). Each member of the team that has an account in
the organization is either part of the General Electrical, General Mechanical or General Software team. This is based on the member’s initial role in the team
and their field of study. A new software member will also be assigned to a SCRB-2026-2027 team in order to be placed in the appropriate project, where it be
Autonomy, Drone, Arm, Frame & Mobility, Power, Science or TT\&C.

#### Repository

Git repositories are where the codebases live. They store all of the modification history of the codebase in order to be able to return to a functioning version
if needed. During development, a version of the codebase lives in the developer's computer. The changes that are done in their working local repository are then
committed and pushed to the repository which lives in GitHub. The GitHub repository is shared with everyone who has access to it.

#### Commits

Once a developer is satisfied with the modification made on their repository, they can commit their changes. Committing changes is equivalent to saving the
current version to git. When committing, it is important to ensure that the associated message is descriptive and representative of the changes.

#### Branches

Branches, in git, are copies of the codebases they were branched from. They are used to develop features without directly modifying the functioning codebase. A
branch should be created with a specific intention and purpose. Once the modifications meet the intentions and purpose of the branch, it would be merged to the
codebase it was branched from.

#### Pushing

Pushing is the action of updating the GitHub repository with the changes that are committed on a local repository. This is how a developer shares their
modifications and code with the rest of the team.

#### Pulling

Pulling is the action of updating your local git repository with changes that were made from the GitHub repository. This ensures that the codebase a developer
is working on is up to date with the team's codebase. It is important to pull before making changes as it prevents merge conflicts from existing between both
git repositories.

#### Pull Request

A Pull Request, sometimes called PR, is a request from a user to combine the codebases between two branches. Usually a user would want to merge their
development branch with the main branch. A PR needs to be reviewed by another member before merging the branches together. Often merge conflicts can arise and
decisions on what to keep and change must be made in order to resolve the conflict.

#### Workflow

The general workflow of a project is the following. First, the developer needs to connect their GitHub account to their computer. They will then be able to
develop in a GitHub repository from their own device. Once their account is created, they will need to clone the repository in which they want to develop
features or fix bugs. They now have a copy of the GitHub repository as a repository in their system. They will then need to create a branch from main in order
to add their changes. With the changes now done, they will add and commit the changes to their local repository and then push their changes to their branch.
Once the branch is ready to be merged with the main repository, a Pull Request will be opened. The PR will go through a test in order to evaluate whether it is
ready to be merged with main. If it is not ready, the pull request will have feedback on how to fix the issues. Once all is good, the developer will push once
again and the PR will be re-reviewed and merged to main. The developer would have successfully contributed to the main codebase of the project.

## Projects

An SCRB software member will utilise the presented skill in order to contribute to one of the following projects. The combination of these projects results in
the creation of the total rover.

### Arm (MA)

The Arm project is responsible for the maintenance, conception and upgrading of the rover robotic arm. The arm has 6 degrees of freedom and at the moment has
one end-effector tool which is a gripper. Each degree of freedom is tied to a mechanical joint. These joints also have an encoder associated with them. A
software member would program the joint actuations and the reception of the joints current positions, velocity and effort (torque). With these programmed
functionalities, the software members are responsible for implementing the forward kinematics, inverse kinematics, path trajectory and autonomy of the robotic
arm.

### Autonomy (AU)

The Autonomy project is responsible for the utilization of the rovers sensors (IMU, GPS, LiDAR, depth camera, cameras, etc.) in order to generate the
appropriate drive train control allowing for the completion of autonomous navigation. Autonomous navigation is possible through sensor fusion to enable accurate
odometry, object detection, object avoidance, goal definition and mission planning. All of these features are programmed and tested by the software member.

### Drone (DR)

The Drone project is responsible for the maintenance, conception and upgrading of the SCRB drone. The project builds a drone from the ground up. It is used to
support the rover during missions. It helps the rover by providing a third person point of view for the rover pilot and allows it to extend the range of the
network through a relay antenna attached to the drone. A software member is responsible for connecting the drone to the rover and base station network.

### Mobility & Frame (MF)

The Mobility & Frame project is responsible for the maintenance, conception and upgrading of the rover’s drive train, electrical box and project interfaces. The
drive train is the wheel configuration used to allow the rover to navigate the ground of its environment. The electrical box is the housing which protects most
of the electronics which control the rover along with the Nvidia Jetson. The project interfaces are the physical connections which attach the project modules.
For this project, a software member would be responsible for controlling the rover's drivetrain. They would also make sure that the positioning of sensors and
other rover components are respected in the software description.

### Power (PW)

The Power project is responsible for the power system of the entire rover. It ensures that each component is safely powered up. Its team implements the source
of all of the power telemetry data. A software member has the responsibility of ensuring that the telemetry display is accurate and complete.

### Science (SP)

The Science project is responsible for running the scientific experiments on the rover. The mechanical and electrical members of the project integrate the
payload. The software members of the team are responsible for the control of the various systems in the payload, gathering and analysing the data of the
experiments. They make sure that the intentions of the scientists are respected and that they have the accurate and well formatted results of the experiments.

### Telemetry, Tracking & Command (TC)

The TT\&C project is responsible for the networking and communication within and between the rover and the base station. This project is heavily populated with
software members. This project oversees the implementation of the CAN and LAN networks. They are responsible for connecting everything together and making sure
that the team has full control over each actuator and is able to read every sensor on the rover. They make sure that the base station can communicate with the
rover and view its video streams. It is also their responsibility to make a nice UI for the people who will be in the base station during the missions.

## Task

The introduction task and its instructions can be found in the following repository:
[https\://github.com/space-concordia-robotics/Software\_Intro\_Task\_2026-2027](https://github.com/space-concordia-robotics/Software_Intro_Task_2026-2027)
Make sure to create a fork of the repository in order to complete the objectives. Feel free to reach out if you are stuck at a certain point. Once the task is
complete, you will need to present it to the current Software Manager of the Space Concordia Robotics division.

## Goal at SCRB

In general, a person who joins the Space Concordia Robotics division is interested in learning more about space robotics. The biggest project, at the moment, is
the robotic rover for space exploration. It is a good opportunity to learn more about the subjects that are touched upon throughout your degree and to learn
more about the other disciplines involved in the project, such as project management, mechanical, electrical and computer engineering. Through working on this
project, a member gains hands-on experience working on a technical project within a large team.

### Outreach

One of the best ways to evaluate the understanding of the project is to present it to someone. During outreach events, a member will need to present the project
to curious students, faculty or the general public. They will learn to answer questions and explain complex ideas to those involved in the conversations. As a
software member, they will need to also explain their experience and their role in the project. Participating in as many outreach events as possible is greatly
encouraged, as it helps with the presence of the team and could help the member expand on their career connections.

### Testing

In order to make sure that the rover is functioning properly, the team regularly tests the system and its subsystems. Some tests can be done inside and others
have to be done outside. A member would be responsible for contributing to the tests in order to assure that they can happen smoothly. Helping out with setup
and logistics is greatly encouraged as it will help with the evaluation of the rover. It also allows for the members to learn more about testing and validation.
As a software member you will learn how to log data, analyze the data, set up the testing scripts and control the rover.

### University Rover Challenge (URC)

The University Rover Challenge is an international space rover competition. In the 2025-2026 season, there were 116 teams who gave in a System Acceptance Review
document and presentation video. This is the first opportunity for teams to get points in the competition. It is also the way in which the competition evaluates
which teams will be within the finalists and compete at the Mars Desert Research Station in the state of Utah. The Robotics division of Space Concordia was one
of the 38 finalists who had the opportunity to compete in the missions. There are four missions that the rover must complete at URC, including the Science,
Delivery, Equipment Servicing and Autonomous Navigation missions. These missions are summarized in the following paragraphs.

#### Science Mission

In the science mission, the science team needs to come up with multiple scientific experiments which would determine whether a site contains signs of life. For
each site, the team needs to extract a surface soil sample, record the GNSS coordinate, take a panoramic photograph, take a microscopic photograph and run the
experiments on the sample. At one of the sites, the rover will need to extract a deep soil sample at a depth of at least 10cm from the surface. Once the mission
is complete, the team will need to present a surface and deep soil sample along with the experiments and engineering done to create the systems which are used
in the mission. For our team all of the systems are housed in the science payload.

#### Delivery Mission

In the delivery mission, the rover has the 6 degree of freedom robotic arm attached to it. There are many ways to gain points in this mission. The first way is
by moving a rock from one location to another. Another way is to gather wreckage pieces and bring them back to an astronaut. The drone can also be used to read
a number of a sign which is not accessible to the rover. The rover can also travel a total distance of 200m in order to pick up an object and bring it back to
its starting point. Finally, a second stage of the mission can be unlocked in order to deliver objects at an even further distance from the base station. This
mission has the great challenge of having big mountains which block the signal between the rover and the basestation.

#### Equipment Servicing Mission

In the equipment servicing mission, the rover must use its robotic arm once again to complete tasks on a dexterity board. On this board, the rover will need to
plug a USB-C cord in order to extract data from a computer. It will also need to place a test tube inside of the box and then place the box inside a drawer.
Specific buttons, switches and a valve need to be activated by the rover. A hose with a gator lock needs to be locked onto an open pipe. Finally, the rover arm
will need to be able to autonomously type a sequence of characters onto a keyboard.

#### Autonomous Navigation Mission

In the autonomous navigation mission, the rover will need to autonomously traverse the desert to points of interest defined beforehand. Two of the locations,
that are given by GNSS coordinates, must be reached within a 3m accuracy. Two more locations are found by searching for AR tags and stopping at a distance of 5m
and 10m from the tags. The rover utilizes computer vision in order to identify and localize an orange mallet, a rock pick hammer and a 1 liter water bottle in
order to stop within 3m, 3m and 10m from the object respectively. At each point of interest, the rover must indicate through colored light whether it has found
one of these locations.

### European Rover Challenge (ERC)

This year, on top of participating in URC, the team will attempt to participate in the European Rover Challenge. This new competition introduces new challenges
which the current team has started preparing for this year. These challenges include the Science, Navigation, Maintenance, Probing and Presentation tasks, which
are briefly summarized below. The competition takes place in a Marsyard located in Krakow, Poland.

#### Science Task

In the Science Task, the rover must be able to complete three subtasks. These are the Exploration and Construction, the Surface Sampling and the Deep
Sub-Surface Sampling subtasks. The Exploration and Construction task has the goal of conducting a scientific hypothesis checkup and building a structure, while
cooperating with other rovers on the field. In the Surface Sampling and Deep Sub-Surface Sampling tasks, a sample must be collected and analyzed using
instruments in the rover.

#### Navigation Task

In the Navigation Task, the rover must autonomously navigate to five checkpoints in the Marsyard. In order to obtain full points, the task must be completed
with no video stream. In the task, the drone must be controlled while respecting flight instructions given during the mission and capture two photographs during
flight.

#### Maintenance Task

In the Maintenance Task, the team must employ the robotic arm in order to evaluate a control panel. This task utilises AR tags in order to allow the robot to
complete the task autonomously. The rover will be expected to configure switches, measure electrical parameters, power on an electromagnetic lock, and insert an
RJ-45 connector.

#### Probing Task

In the Probing Task, the team is tasked with locating three probes distributed throughout the Marsyard. Once the probes have been found, they must be installed
onto a designated location on the rover. This task must be completed as fast as possible in order to get the most points.

#### Presentation Task

In the Presentation Task, the rover and the team behind it are presented in front of the competition judges. In this presentation, the team structure, the
rover’s software, electrical and mechanical components, the competition tasks strategies, the technical solutions and the team’s strategies for overcoming
challenges and lessons learned must all be presented. After the presentation, the team must be ready for a Q\&A session to discuss with the judges.

## Conclusion

The information and task presented in this document is meant as a first exposure to the software development of the Space Concordia Robotics division. It is
normal not to fully understand every concept in this document. It is important to stay curious and not be afraid to ask questions and try new things. Everyone
will break something eventually and breaking something means that you were doing something. Joining the Robotics division of Space Concordia is a really good
way of implementing what you are learning in your degree and to work on a long-term project with a big team.
