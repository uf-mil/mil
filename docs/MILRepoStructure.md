'''bash
.
├── branding                                                                (Branding)                                         
│   ├── mil.svg
│   ├── mil_white.svg
│   ├── mil_with_text.ai
│   ├── robosub                                                             (Logos/Images for Robosub)
│   └── robotx                                                              (Logos/Images for RobotX)
├── deprecated                                                              (Code no longer in use)
│   ├── installer                                                           (MIL Set up/Introduction)
│   ├── mil_common                                                          (Common deprecated code)
│   │   ├── CATKIN_IGNORE
│   │   ├── gnc                                                             (GNC Deprecated)
│   │   │   └── rawgps_common
│   │   │       └── scripts                                                 (RawGPS Common Scripts)
│   │   ├── mil_tools                                                       (MIL ROS Tools Deprecated)
│   │   │   └── mil_ros_tools
│   │   └── perception                                                      (Perception UI Deprecated)
│   │       └── point_cloud_object_detection_and_recognition
│   │           └── tools                                                   (Perception UI Tools Deprecated)
│   ├── NaviGator                                                           (NaviGator Code Deprecated)
│   │   ├── CATKIN_IGNORE
│   │   ├── gnc                                                             (Rospy Deprecated)
│   │   │   └── navigator_msg_multiplexer                                   (NaviGator Message Multiplexer Deprecated)
│   │   │       └── nodes                                                   (NaviGator Message Multiplexer Nodes Deprecated)
│   │   ├── mission_control                                                 (NaviGator Missions/Launch Deprecated)
│   │   │   ├── navigator_launch
│   │   │   │   └── launch                                                  (NaviGator Mission Control Launch Deprecated)
│   │   │   │       └── subsystems                                          (NaviGator Mission Launch Subsystems Deprecated)
│   │   │   └── navigator_missions                                          (NaviGator Missions Deprecated)
│   │   │       ├── navigator_singleton                                     (NaviGator Singleton Deprecated)
│   │   │       ├── nav_missions                                            (NaviGator Missions Code Deprecated)
│   │   │       ├── nav_missions_lib                                        (NaviGator Missions Lib Deprecated)
│   │   │       ├── nav_missions_test                                       (NaviGator Missions Test Deprecated)
│   │   │       └── nodes                                                   (NaviGator Missions Nodes Deprecated)
│   │   ├── mission_systems                                                 (Mission_Systems Deprecated)
│   │   │   ├── navigator_scan_the_code                                     (Scan the Code Deprecated)
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── navigator_scan_the_code                                 (Scan the Code Source Code Deprecated)
│   │   │   │   │   └── scan_the_code_lib
│   │   │   └── shooter                                                     (Shooter deprecated)
│   │   │       ├── launch
│   │   │       └── nodes                                                   (Shooter Nodes Deprecated)
│   │   ├── perception                                                      (Perception deprecated)
│   │   │   ├── navigator_lidar_oa
│   │   │   │   └── src                                                     (NaviGator Perception Source Code Deprecated)
│   │   │   └── navigator_vision                                            (NaviGator Vision deprecated)
│   │   │       ├── exFAST_SparseStereo
│   │   │       │   └── src                                                 (SparseStereo Source Deprecated)
│   │   │       │       ├── example
│   │   │       │       └── sparsestereo                                    (SparseStereo Code Deprecated)
│   │   │       └── nodes                                                   (NaviGator Nodes Deprecated)
│   │   ├── simulation                                                      (NaviGator Simulation Deprecated)
│   │   │   ├── navigator_2dsim
│   │   │   └── navigator_gazebo                                            (NaviGator Gazebo Deprecated)
│   │   │       ├── include                                                 (NaviGator Gazebo Include Deprecated)
│   │   │       │   └── navigator_gazebo
│   │   │       ├── launch                                                  (NaviGator Launch Deprecated)
│   │   │       ├── models                                                  (NaviGator Models Deprecated)
│   │   │       │   ├── buoys                                               (NaviGator Buoys Deprecated)
│   │   │       │   ├── coral_survey                                        (NaviGator Coral Survey Deprecated)
│   │   │       │   ├── detect_and_deliver                                  (NaviGator Detect and Deliver Deprecated)
│   │   │       │   ├── find_the_break                                      (Find the Break Deprecated)
│   │   │       │   ├── identify_symbols_and_dock                           (NaviGator Symbols and Dock Deprecated)
│   │   │       │   ├── markers                                             (NaviGator Markers deprecated)
│   │   │       │   ├── scan_the_code                                       (NaviGator Gazebo Scan the Code Deprecated)
│   │   │       │   ├── sky_box                                             (Sky Box Deprecated)
│   │   │       │   └── wamv                                                (WamV Deprecated)
│   │   │       ├── src                                                     (Src Deprecated)
│   │   │       └── worlds                                                  (Worlds Deprecated)
│   │   ├── utils                                                           (NaviGator Utils Deprecated)
│   │   │   ├── navigator_emergency_control                                 (NaviGator Emergency Control Deprecated)
│   │   │   │   ├── bootloader                                              (NaviGator Emergency Control Bootloader Deprecated)
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── firmware                                                (NaviGator Utils Firmware Deprecated)
│   │   │   │   ├── include                                                 (NaviGator Utils Include Deprecated)
│   │   │   │   │   └── navigator_emergency_control
│   │   │   │   └── src                                                     (NaviGator Src Deprecated)
│   │   │   ├── navigator_msgs                                              (NaviGator Messages Deprecated)
│   │   │   │   └── srv                                                     (NaviGator Messages SRV Deprecated)
│   │   │   ├── navigator_tools                                             (NaviGator Tools Deprecated)
│   │   │   │   └── navigator_tools                                         
│   │   │   └── templates                                                   (Testing Templates Deprecated)
│   │   │       ├── cv_testing
│   │   └── xbox_joy_node                                                   (NaviGator XBox Joy Node Deprecated)
│   │       └── src                                                         (NaviGator XBox Source Deprecated)
│   ├── scripts                                                             (Scripts Deprecated)
│   ├── SubjuGator                                                          (Subjugator Deprecated)
│   │   ├── CATKIN_IGNORE
│   │   ├── command                                                         (Subjugator Command Deprecated)
│   │   │   ├── sub8_launch                                                 (Sub8 Launch Deprecated)
│   │   │   │   └── launch                              
│   │   │   │       └── subsystems
│   │   │   └── sub8_missions                                               (Sub8 Missions Deprecated)
│   │   │       └── sub8_missions                                           (Sub8 Missions Code Deprecated)
│   │   ├── drivers                                                         (Subjugator Drivers Deprecated)
│   │   │   └── sub8_videoray_m5_thruster                                   (Sub8 VideoRay M5 Thruster Deprecated)
│   │   │       ├── config                                                  (Sub8 Drivers Config Deprecated)
│   │   │       │   ├── calibration                                         (Sub8 Drivers Calibration Deprecated)
│   │   │       │   └── firmware_settings                                   (Sub8 Firmware YAML Files Deprecated)
│   │   │       ├── nodes                                                   (Sub8 Drivers Nodes Deprecated)
│   │   │       ├── sub8_thruster_comm                                      (Sub8 Drivers Thruster Comm Deprecated)
│   │   │       └── test                                                    (Sub8 Drivers Test Deprecated)
│   │   ├── gnc                                                             (SubjuGator GNC Deprecated)
│   │   │   ├── sub8_controller                                             (Sub8 Controller Deprecated)
│   │   │   │   ├── cfg                                                     (PD Controller Config Deprecated)
│   │   │   │   ├── include                                                 (Subjugator GNC Include Deprecated)
│   │   │   │   │   ├── sub8_controller                            
│   │   │   │   │   └── sub8_cvx                                            (Sub8 CVX Math C Files Deprecated)
│   │   │   │   └── nodes                                                   (Sub8 GNC Nodes Deprecated)
│   │   │   └── sub8_system_id                                              (Sub8 System ID Deprecated)
│   │   ├── simulation                                                      (Sub8 Simulation Files Deprecated)
│   │   │   └── sub8_montecarlo                                             (Sub8 MonteCarlo Deprecated)
│   │   │       ├── nodes                                                   (Sub8 Nodes MonteCarlo Plotter Deprecated)
│   │   │       └── sub8_montecarlo_tools                                   (Sub8 Montecarlo Controller Deprecated)
│   │   ├── sub8_rqt                                                        (Sub8 Alarm GUI Deprecated)
│   │   │   ├── resource                                                    (Sub8 RQT Resources Deprecated)
│   │   │   ├── scripts
│   │   │   │   └── rqt_alarms
│   │   │   └── sub8_rqt_gui
│   │   └── utils                                                           (Sub8 Thruster/Actuator Code Deprecated)
│   │       └── sub8_diagnostics
│   │           └── scripts
│   ├── sylphase-sonar                                                      (Sylphase Sonar Deprecated)
│   │   ├── downsample                                                      (Sylphase Sonar Downsample Deprecated)
│   │   ├── plot                                                            (Sylphase Sonar Plot Deprecated)
│   │   ├── plot_once                                                       (Sylphase Sonar Plot Once Deprecated)
│   │   ├── plot_pings                                                      (Sylphase Sonar Pings Deprecated)
│   │   ├── psd                                                             (Sylphase Sonar PSD Deprecated)
│   │   └── ros_bridge                                                      (Sylphase Sonar ROS Bridge Deprecated)
│   └── wikis                                                               (Wikis Deprecated)
│       ├── mil_common                                                      (Move Files Mil Common Deprecated)
│       ├── NaviGator                                                       (Move files NaviGator Deprecated)
│       └── SubjuGator                                                      (Move files Subjugator Deprecated)
├── docker                                                                  (Docker)
│   ├── base                                                                (Set up Environment and Time)
│   │   ├── Dockerfile
│   │   └── system_install
│   ├── ci-server                                                           (CI-Server)
│   │   └── Dockerfile
│   ├── dev                                                                 (Code for Docker)
│   │   ├── Dockerfile
│   │   ├── dynparam
│   │   └── user_install
│   ├── noetic                                                              (Corrected CI Issues)
│   │   └── Dockerfile
│   └── vrx_trial                                                           (Autoflake, Pint, Reformat)
│       ├── Dockerfile
│       ├── dynparam
│       ├── run_vrx
│       └── vrx_trial_install
├── docs                                                                    (RQT Docs)                                            
│   ├── design                                                              (Design Docs)     
│   │   ├── odom_estimator
│   │   └── passive_sonar                                                   (Passive Sonar and Migrate Pneumatic Board)
│   ├── Doxyfile
│   ├── electrical                                                          (Electrical Docs)
│   ├── extensions                                                          (Code Extensions)
│   ├── images                                                              (Images Docs)
│   │   └── gazebo
│   ├── indyav                                                              (RST Files Documenting Software and Hardware)
│   │   ├── hardware                                                        (Hardware Docs)
│   │   │   └── go_kart                                                     (Go Kart Docs)
│   │   └── software                                                        (Software Docs)
│   │       ├── control                                                     (Control RST)
│   │       │   └── joydrive                                                (JoyDrive RST)
│   │       ├── planning                                                    (Planning Docs)
│   │       │   └── indyav_path                                             (PathFinding Docs)
│   │       └── simulation                                                  (Simulation Docs)
│   │            └──gazebo                                                  (Gazebo Docs)
│   ├── infrastructure                                                      (Infrastructure Docs)
│   ├── mechanical                                                          (Mechanical Docs)
│   ├── navigator                                                           (NaviGator Docs)
│   ├── reference                                                           (CI Improvements and ROS)
│   │   ├── axros
│   │   └── mission                                                         (Mission Docs)
│   ├── software                                                            (Markdown Additions to Software)
│   ├── _static                                                             (Installation Instructions for MacOS using UTM)
│   ├── subjugator                                                          (Markdown Files for Testing Changes)           
│   ├── _templates                                                          (Copy Button and Search Bar)
│   └── uf-mil-pygments                                                     (UF MIL Pygments)
├── infra                                                                   (MIL Network Infrastructure)
│   └── network-box
├── LICENSE
├── mil_common                                                              (MIL Common Software)
│   ├── axros                                                               (Axros)
│   │   ├── axros                                                           (Axros Extensions for ROS)
│   │   │   ├── src                                                         (Axros Source Code)
│   │   │   │   └── axros
│   │   │   └── test                                                        (Axros Test Code)
│   │   └── scripts                                                         (Axros Dockerfile)
│   │       └── Dockerfile
│   ├── drivers                                                             (MIL Common Drivers)
│   │   ├── LMS1xx                                                          (SICK LMS1xx Drivers)                
│   │   │   ├── include             
│   │   │   │   └── LMS1xx                                                  (LMS1xx Include Files)
│   │   │   ├── launch                                                      (LMS1xx Launch Driver)
│   │   │   ├── meshes                                                      (LMS1xx Meshes)       
│   │   │   ├── scripts
│   │   │   │   ├── find_sick
│   │   │   │   └── set_sick_ip
│   │   │   ├── src                                                         (LMS1xx Source)
│   │   │   ├── test                                                        (LMS1xx Test Code)
│   │   │   └── urdf
│   │   ├── mil_acoustic_modems                                             (Raspberry PI 3 B+ Units and EvoLogic Acoustics)
│   │   ├── mil_blueview_driver                                             (Telodyne BlueView SDK for Images in ROS)
│   │   │   ├── include                                                     (BlueView Include Files)
│   │   │   ├── launch
│   │   │   ├── msg                                                         (BlueView Message)
│   │   │   ├── src                                                         (BlueView Source)
│   │   │   └── test
│   │   ├── mil_passive_sonar                                               (Passive Sonar Algorithms for Finding Direction of Pinger)    
│   │   │   ├── launch
│   │   │   ├── msg                                                         (Passive Sonar Messages)
│   │   │   ├── scripts                                                     (Passive Sonar Python Scripts)
│   │   │   ├── src                                                         (Passive Sonar Source Code)
│   │   │   │   └── mil_passive_sonar
│   │   │   └── srv
│   │   ├── mil_pneumatic_actuator                                          (ROS Driver for Interaction with Solenoids)
│   │   │   ├── launch
│   │   │   │   └── example.launch
│   │   │   ├── mil_pneumatic_actuator                                      (Mil Pneumatic Actuator Code)
│   │   │   ├── nodes                                              
│   │   │   │   └── pneumatic_actuator_node
│   │   │   └── srv
│   │   ├── mil_usb_to_can                                                  (MIL Interface for USB to CAN)
│   │   │   ├── launch
│   │   │   │   └── example.launch
│   │   │   ├── mil_usb_to_can
│   │   │   │   ├── sub8                                                    (Sub8 USB to CAN)
│   │   │   │   └── sub9                                                    (Sub9 USB to CAN)
│   │   │   └── test                                                        (Test Drivers for Sub8 and Sub9)
│   │   ├── pointgrey_camera_driver                                         (ROS Compatible PointGrey Camera Drivers)
│   │   │   ├── image_exposure_msgs
│   │   │   │   └── msg                                                     (PointGrey Messages)
│   │   │   ├── pointgrey_camera_description                                (PointGrey Camera Docs)
│   │   │   │   ├── launch
│   │   │   │   ├── meshes
│   │   │   │   ├── rviz
│   │   │   │   └── urdf                                                    (PointGrey URDF)
│   │   │   ├── pointgrey_camera_driver                                     (PointGrey Camera Drivers)
│   │   │   │   ├── cfg
│   │   │   │   ├── cmake
│   │   │   │   │   └── download_flycap
│   │   │   │   ├── debian
│   │   │   │   │   └── udev
│   │   │   │   ├── include                                                 (PointGrey Driver Include)
│   │   │   │   │   └── pointgrey_camera_driver
│   │   │   │   ├── launch                                                  (PointGrey Camera Launch)
│   │   │   │   ├── src                                                     (PointGrey Source Code)
│   │   │   │   └── udev
│   │   │   ├── statistics_msgs                                             (PointGrey Statistiscs Messages)
│   │   │   │   └── msg
│   │   │   └── wfov_camera_msgs                                            (WFOV Camera Messages)
│   │   │       └── msg
│   │   ├── roboteq                                                         (ROS Driver for RobotEq Motor Controllers)
│   │   │   ├── roboteq_diagnostics                                         (RobotEq Diagonstics)
│   │   │   │   ├── nodes
│   │   │   │   │   └── diagnostic_publisher
│   │   │   │   └── package.xml
│   │   │   ├── roboteq_driver                                              (RobotEq Drivers)
│   │   │   │   ├── include                                                 (RobotEq Drivers Include)
│   │   │   │   │   └── roboteq_driver
│   │   │   │   ├── launch                                                  (RobotEq Launch Drivers)
│   │   │   │   ├── mbs         
│   │   │   │   └── src                                                     (RobotEq Source Drivers)
│   │   │   └── roboteq_msgs                                                (RobotEq Messages)
│   │   │       └── msg
│   │   └── sabertooth2x12                                                  (ROS Driver for Sabertooth Controller)
│   │       ├── launch
│   │       ├── nodes
│   │       │   └── sabertooth2x12ros
│   │       └── sabertooth2x12                                              (Sabertooth Driver Code)
│   ├── gnc                                                                 (MIL GNC)
│   │   ├── mil_bounds                                                      (MIL Boundary)
│   │   │   ├── cfg  
│   │   │   ├── mil_bounds                                                  (MIL Boundary Code)
│   │   │   ├── nodes                                                       (MIL Boundary Nodes)
│   │   │   │   ├── bounds_from_rviz
│   │   │   │   └── bounds_server
│   │   │   └── test                                                        (MIL Boundary Test Code)
│   │   ├── odom_estimator                                                  (Odometer Estimator)
│   │   │   ├── data
│   │   │   ├── doc
│   │   │   ├── include                                                     (Odometer Estimator Include)
│   │   │   │   └── odom_estimator                                          (Odemeter Estimator H Files)
│   │   │   ├── mainpage.dox
│   │   │   ├── msg                                                         (Odometer Estimator Messages)
│   │   │   ├── src                                                         (Odometer Estimator Source Code)
│   │   │   └── srv                                                         (Odometer Estimator SRV)
│   │   ├── odometry_utils                                                  (Odometer Utilities)
│   │   │   └── src                                                         (Odometer Utilities Source Code)
│   │   └── rawgps_common                                                   (RawGPS Common)
│   │       ├── msg                                                         (RawGPS Message Files)
│   │       └── src                                                         (RawGPS Source)
│   │           └── rawgps_common                                           (RawGPS Source Code)
│   ├── Jenkinsfile                                                         (Jenkins File)
│   ├── mil_missions                                                        (MIL ROS Missions)
│   │   ├── action                                                          (Do Mission)
│   │   │   └── DoMission.action
│   │   ├── launch                                                          (Launch Example)
│   │   ├── mil_missions_core                                               (MIL Missions Code Core)
│   │   ├── mil_missions_examples                                           (MIL Missions Examples Code)
│   │   ├── mil_missions_gui                                                (MIL Missions GUI)
│   │   ├── nodes                                                           (MIL Missions Nodes)
│   │   │   ├── mission_client
│   │   │   └── mission_server
│   │   ├── resource
│   │   └── test                                                            (MIL Missions Test Code)
│   ├── perception                                                          (MIL Perception)
│   │   ├── mil_mlp                                                         (MIL Machine Learning Pipeline)
│   │   │   ├── CMakeLists.txt  
│   │   │   ├── docker_tf                                                   (Docker Transfer)
│   │   │   │   ├── Dockerfile
│   │   │   │   └── transfer_learning                                       (Docker Transfer Learning)
│   │   │   ├── ldp                                                         (MIL LDP)
│   │   │   │   └── labelbox2pascal                                         (LabelBox2 Pascal)
│   │   │   │       └── pascal_voc_writer                                   (Pascal Code)
│   │   │   │           └── templates
│   │   │   └── setup.py
│   │   ├── mil_vision                                                      (MIL Vision)
│   │   │   ├── include                                                     (MIL Vision Include Files)
│   │   │   │   └── mil_vision_lib
│   │   │   │       ├── colorizer                                           (Colorizer HPP Files)
│   │   │   │       └── image_acquisition                                   (Image Acquisition HPP Files)
│   │   │   ├── mil_vision_tools                                            (MIL Vision Tools Code)
│   │   │   ├── object_classification                                       (MIL Object Classification Code)
│   │   │   ├── ros_tools                                                   (MIL Vision ROS Tools Code)
│   │   │   └── src                                                         (MIL Vision Source Code)
│   │   │       └── mil_vision_lib                                          (Mil Vision Libraries)
│   │   │           └── colorizer
│   │   ├── point_cloud_object_detection_and_recognition                    (Point Cloud Detection and Reception)
│   │   │   ├── cfg
│   │   │   ├── include                                                     (Point Cloud Detection and Reception Include)
│   │   │   │   └── point_cloud_object_detection_and_recognition
│   │   │   ├── launch                                                      (Point Cloud Detection and Reception Launch)
│   │   │   ├── nodes                                                       (Point Cloud Detection and Reception Nodes)
│   │   │   └── src                                                         (Point Cloud Detection and Reception Source Code)
│   │   └── yolov7-ros                                                      (ROS Noetic Package For YOLOv7)
│   │       ├── launch
│   │       ├── LICENSE
│   │       └── src                                                         (YOLOv7 Source)
│   │           ├── cfg                                                     (YOLOv7 Config)
│   │           │   ├── baseline                                            (YOLOv7 Baseline)
│   │           │   ├── deploy                                              (YOLOv7 Deploy)
│   │           │   └── training                                            (YOLOv7 Training)
│   │           ├── data                                                    (YOLOv7 Data)
│   │           ├── figure
│   │           ├── inference
│   │           │   └── images
│   │           ├── mil_weights                                             (YOLOv7 MIL Weights)
│   │           ├── models                                                  (YOLOv7 Models)
│   │           ├── scripts
│   │           └── utils                                                   (YOLOv7 Utilities)
│   │               ├── aws
│   │               ├── google_app_engine                                   (YOLOv7 Google App Engine)
│   │               │   └── Dockerfile
│   │               └── wandb_logging                                       (YOLOv7 WandB loggin)
│   ├── ros_alarms                                                          (ROS Alarms)
│   │   ├── include                                                         (ROS Alarms Include)
│   │   │   └── ros_alarms
│   │   ├── Jenkinsfile
│   │   ├── launch                                                          (ROS Alarms Launch)
│   │   ├── nodes                                                           (ROS Alarms Nodes)
│   │   │   ├── clear
│   │   │   ├── monitor
│   │   │   ├── raise
│   │   │   └── report
│   │   ├── ros_alarms                                                      (ROS Alarms Code)
│   │   ├── scripts                                                         (ROS Alarms Scripts)
│   │   │   └── Dockerfile
│   │   ├── src                                                             (ROS Alarms Source Code)
│   │   │   └── ros_alarms
│   │   └── test                                                            (ROS Alarms Test Files)
│   │       ├── axros                                                       (ROS Alarms Axros)
│   │       ├── roscpp                                                      (ROS Alarms cpp Files)
│   │       ├── rospy                                                       (ROS Alarms ROSPy Files)
│   │       ├── rostests                                                    (ROS Alarms ROSTests)
│   │       └── test_handlers                                               (ROS Alarms Test Handlers)
│   ├── ros_alarms_msgs                                                     (ROS Alarms Messages)
│   │   ├── msg                                                             (Alarm Messages)
│   │   └── srv                                                             (Alarm SRV)
│   ├── scripts                                                             (MIL Scripts)
│   │   ├── 999-ping-check
│   ├── simulation                                                          (MIL Simulation and Gazebo)
│   │   └── mil_gazebo                                                      (MIL Gazebo)
│   │       ├── include                                             
│   │       │   └── mil_gazebo                                              (MIL Gazebo Include)
│   │       ├── models                                                      (MIL Simulation Models)
│   │       │   └── pinger
│   │       ├── nodes                                                       (MIL Simulation Nodes)
│   │       │   └── set_gazebo_rtf
│   │       ├── src                                                         (MIL Simulation Source Code)
│   │       └── xacro                                                       (MIL Simulation Xacro)
│   └── utils                                                               (MIL Utils)
│       ├── mil_msgs                                                        (MIL Utils Messages)
│       │   ├── action   
│       │   ├── msg                                                         (MIL Utils Message Files)
│       │   └── srv                                                         (MIL Utils SRV)
│       ├── mil_poi                                                         (MIL POI (Points of Interest))
│       │   ├── launch                                                      (MIL POI Launch)
│       │   ├── mil_poi                                                     (MIL POI Code)
│       │   ├── msg                                                         (MIL POI Messages)
│       │   ├── nodes                                                       (MIL POI Nodes)
│       │   │   └── poi_server
│       │   └── srv                                                         (MIL POI SRV)
│       └── mil_tools                                                       (MIL Tools)
│           ├── CMakeLists.txt
│           ├── include                                                     (MIL Tools Include)
│           │   └── mil_tools
│           ├── launch                                                      (MIL Tools Launch)
│           ├── mil_misc_tools                                              (MIL Disc Tools Code)
│           ├── mil_ros_tools                                               (MIL ROS Tools Code)
│           ├── mil_tools                                                   (MIL Tools Init)
│           ├── nodes                                                       (MIL Tools Nodes)
│           ├── scripts                                                     (MIL Tools Scripts)
│           ├── src                                                         (MIL Tools Source)
│           │   └── mil_tools
│           └── test                                                        (MIL Tools Test)
├── NaviGator                                                               (NaviGator)
│   ├── docs                                                                (NaviGator Docs)
│   ├── etc                                                                 (NaviGator Etc)
│   │   ├── fstab
│   │   ├── network                                                         (NaviGator Network)
│   │   │   └── interfaces
│   │   ├── rc.local
│   │   ├── security                                                        (NaviGator Security)
│   │   └── udev                                                            (NaviGator Udev)
│   │       └── rules.d
│   ├── gnc                                                                 (NaviGator GNC)
│   │   ├── navigator_controller                                            (NaviGator Controller)
│   │   │   ├── nodes                                                       (NaviGator Controller Nodes)
│   │   ├── navigator_msg_multiplexer                                       (Navigatotr Message Multiplexer)
│   │   │   ├── cfg
│   │   │   └── nodes                                                       (NaviGator Message Nodes)
│   │   ├── navigator_path_planner                                          (NaviGator Path Planner)
│   │   │   ├── action                                                      (NaviGator Action Files)
│   │   │   ├── lqRRT                                                       (NaviGator lqRRT)
│   │   │   │   ├── demos                                                   (NaviGator Path Planner Demos)
│   │   │   │   │   └── lqrrt_ros                                           (NaviGator lqRRT ROS)
│   │   │   │   │       ├── action                                          (NaviGator lqRRT Action Files)
│   │   │   │   │       ├── behaviors                                       (NaviGator lqRRT Behaviors Code)
│   │   │   │   │       ├── launch                                          (NaviGator lqRRT Launch Code)
│   │   │   │   │       └── nodes                                           (NaviGator lqRRT Nodes)
│   │   │   │   ├── LICENSE
│   │   │   │   └── lqrrt                                                   (NaviGator lqRRT Secondary)
│   │   │   ├── navigator_path_planner                                      (NaviGator Path Planner Code)
│   │   │   ├── nodes                                                       (NaviGator Path Planner Nodes)
│   │   └── navigator_thrust_mapper                                         (NaviGator Thrust Mapper)
│   │       ├── navigator_thrust_mapper                                     (NaviGator Thurst Mapper Code)
│   │       └── nodes                                                       (NaviGator Thrust Mapper Nodes)
│   ├── hardware_drivers                                                    (NaviGator Hardware Drivers)
│   │   └── navigator_kill_board                                            (NaviGator Kill Board)
│   │       ├── navigator_kill_board                                        (NaviGator Kill Board Code)
│   │       ├── nodes                                                       (NaviGator Kill Board Nodes)
│   │       └── test                                                        (NaviGator Kill Board Tests)
│   ├── mission_control                                                     (NaviGator Mission Control)
│   │   ├── navigator_alarm                                                 (NaviGator Alarm)
│   │   │   └── navigator_alarm_handlers                                    (NaviGator Alarm Handler Code)
│   │   ├── navigator_launch                                                (NaviGator Launch)
│   │   │   ├── config                                                      (NaviGator Launch Config)
│   │   │   │   └── camera_calibration                                      (NaviGator Camera Calibration)
│   │   │   ├── launch                                                      (NaviGator Mission Control Launch Code)
│   │   │   │   ├── gnc                                                     (NaviGator Mission Control Launch GNC)
│   │   │   │   ├── gnc.launch
│   │   │   │   ├── hardware                                                (NaviGator Mission Control Launch Hardware)
│   │   │   │   │   └── cameras                                             (NaviGator Mission Control Launch Cameras)
│   │   │   │   ├── perception                                              (Launch Perception)
│   │   │   │   ├── shore                                                   (Launch Shore)
│   │   │   │   └── vrx                                                     (Launch VRX)
│   │   │   └── package.xml
│   │   └── navigator_missions                                              (NaviGator Missions)
│   │       ├── launch                  
│   │       ├── navigator_missions                                          (NaviGator Missions Code)
│   │       ├── test                                                        (NaviGator Mission Test)
│   │       └── vrx_missions                                                (NaviGator Mission VRX)
│   ├── mission_systems                                                     (NaviGator Mission Systems)
│   │   └── navigator_find_the_break                                        (NaviGator Find the Break)
│   ├── perception                                                          (NaviGator Mission Perception)
│   │   └── navigator_vision                                                (NaviGator Vision)
│   │       ├── config                                                      (NaviGator Vision Config)
│   │       │   ├── stc
│   │       │   └── vrx                                                     (NaviGator Vision VRX)
│   │       ├── datasets                                                    (NaviGator Vision DataSets)
│   │       │   ├── dock_target_images                                      (NaviGator Vision Target Images)
│   │       ├── include                                                     (NaviGator Vision Include)            
│   │       │   └── missions                                                (NaviGator Vision Include Missions)
│   │       │       └── underwater_shape_identification.hpp
│   │       ├── navigator_vision                                            (NaviGator Vision Code)
│   │       ├── nodes                                                       (NaviGator Vision Node)
│   │       │   ├── bbox_pcodar
│   │       │   ├── detect_deliver_target
│   │       │   └── shape_identification                                    (NaviGator Shape Identification Code)
│   │       │       └──GrayscaleContour                                     (NaviGator Grayscale Contour Code)
│   │       ├── src                                                         (NaviGator Vision Source)
│   │       │   └── missions                                                (NaviGator Vision Source Missions)
│   │       └── templates                                                   (NaviGator Mission Templates)
│   │           └── underwater_shape_identification                         (NaviGator Underwater Shape Identification Images)
│   ├── readme.md
│   ├── satellite                                                           (NaviGator Satellite)
│   │   └── rviz_satellite
│   │       ├── launch                                                      (NaviGator Satellite Launch Code)
│   │       └── src                                                         (NaviGator Satellite Source Code)
│   ├── scripts                                                             (NaviGator Scripts)
│   │   ├── Dockerfile
│   │   └── on_boot
│   ├── simulation                                                          (NaviGator Simulation)
│   │   ├── navigator_gazebo                                                (NaviGator Gazebo)
│   │   │   ├── include                                                     (NaviGator Gazebo Include)
│   │   │   │   └── navigator_gazebo                                        (NaviGator Gazebo hpp Files)
│   │   │   ├── launch                                                      (NaviGator Gazebo Launch)
│   │   │   ├── models                                                      (NaviGator Gazebo Models)
│   │   │   │   ├── find_totems_challenge
│   │   │   │   └── robotx_pinger
│   │   │   ├── nodes                                                       (NaviGator Gazebo Nodes)
│   │   │   ├── package.xml
│   │   │   ├── src                                                         (NaviGator Gazebo Source Code)
│   │   │   ├── test                                                        (NaviGator Gazebo Tests)
│   │   │   ├── urdf                                                        (NaviGator Gazebo URDF)
│   │   │   │   └── navigator_vrx                                           (NaviGator Gazebo VRX)
│   │   │   └── worlds                                                      (NaviGator Worlds)
│   │   │       ├── competition                                             (NaviGator Competition Worlds)
│   │   │       ├── dynavDemo                                               (NaviGator dynavDemo Worlds)
│   │   │       ├── entrance                                                (NaviGator Entrance Gate World)
│   │   │       ├── pathfinding                                             (NaviGator Pathfinding Worlds)
│   │   │       ├── scan_dock_fling                                         (NaviGator Scan Dock Worlds)
│   │   │       ├── stationkeeping                                          (NaviGator Station Keeping World)
│   │   │       ├── vision                                                  (NaviGator Vision Worlds)
│   │   │       └── wildlife                                                (NaviGator Wildlife Worlds)
│   │   └── VRX                                                             (NaviGator VRX)
│   │       ├── vrx
│   │       │   ├── images                                                  (NaviGator VRX Images)
│   │       │   ├── tools                                                   (NaviGator VRX Tools)
│   │       │   ├── usv_gazebo_plugins                                      (NaviGator Gazebo Plugins)
│   │       │   │   ├── include                                             (NaviGator Gazebo Plugins Include)
│   │       │   │   │   └── usv_gazebo_plugins
│   │       │   │   ├── launch                                              (NaviGator Launch Buoyancy Plugin)
│   │       │   │   ├── src                                                 (NaviGator Gazebo Plugins Source Code)
│   │       │   │   ├── test                                                (NaviGator Gazebo Test Code)
│   │       │   │   └── worlds                                              (NaviGator Gazebo Bouyancy Worlds)
│   │       │   ├── usv_msgs                                                (NaviGator USV Messages)
│   │       │   │   ├── msg                                                 (NaviGator Range Bearing Message)
│   │       │   │   └── package.xml
│   │       │   ├── vrx_2019                                                (NaviGator VRX 2019)
│   │       │   │   ├── config                                              (NaviGator VRX 2019 Config)
│   │       │   │   ├── launch                                              (NaviGator VRX 2019 Launch Code)
│   │       │   │   ├── package.xml
│   │       │   │   └── worlds                                              (NaviGator VRX 2019 Worlds)
│   │       │   │       └── yamls                                           (NaviGator VRX 2019 YAML files)
│   │       │   ├── vrx_gazebo                                              (NaviGator VRX Gazebo)
│   │       │   │   ├── config                                              (NaviGator VRX Gazebo Config)
│   │       │   │   │   └── wamv_config                                     (NaviGator VRX WamV Config)
│   │       │   │   │       ├── component_compliance
│   │       │   │   │       └── thruster_compliance                         (NaviGator Thruster Compliance)
│   │       │   │   ├── docs                                                (NaviGator VRX Gazebo Documents)
│   │       │   │   │   └── images                                          (NaviGator VRX Gazebo Images)
│   │       │   │   ├── include                                             (NaviGator VRX Gazebo Include)
│   │       │   │   │   └── vrx_gazebo                                      (NaviGator VRX Gazebo Include Files)
│   │       │   │   ├── launch                                              (NaviGator VRX Gazebo Launch Code)
│   │       │   │   ├── models                                              (NaviGator VRX Gazebo Models)
│   │       │   │   │   ├── 3d_lidar                                        (NaviGator VRX 3D Lidar)
│   │       │   │   │   │   └── mesh
│   │       │   │   │   ├── antenna                                         (NaviGator Antenna Models)
│   │       │   │   │   │   └── meshes
│   │       │   │   │   ├── ball_shooter                                    (NaviGator Ball Shooter Models)
│   │       │   │   │   │   ├── materials
│   │       │   │   │   │   │   └── textures
│   │       │   │   │   │   └── meshes                                      (NaviGator Ball Shooter Meshes)
│   │       │   │   │   ├── battery                                         (NaviGator Battery Models)
│   │       │   │   │   │   └── mesh
│   │       │   │   │   ├── black_totem                                     (NaviGator Black Totem Models)
│   │       │   │   │   ├── blue_projectile                                 (NaviGator Blue Projectile Models)
│   │       │   │   │   │   └── materials
│   │       │   │   │   │       └── scripts                                 (NaviGator Blue Projectile Scripts)
│   │       │   │   │   ├── blue_totem                                      (NaviGator Blue Totem Models)
│   │       │   │   │   ├── cpu_cases                                       (NaviGator CPU Cases Models)
│   │       │   │   │   │   └── mesh                                        (NaviGator CPU Cases Meshes)
│   │       │   │   │   ├── crocodile_buoy                                  (NaviGator Crocodile Buoy Model)
│   │       │   │   │   │   ├── materials
│   │       │   │   │   │   │   ├── scripts                                 (NaviGator Crocodile Buoy Scripts)
│   │       │   │   │   │   │   └── textures                                (NaviGator Crocodile Buoy Textures)
│   │       │   │   │   │   └── meshes                                      (NaviGator Crocodile Buoy Meshes)
│   │       │   │   │   ├── crocodile_static                                (NaviGator Crocodile Static Model)
│   │       │   │   │   ├── dock_2016                                       (NaviGator Dock 2016 Model)
│   │       │   │   │   ├── dock_2016_base                                  (NaviGator Dock 2016 Base Model)
│   │       │   │   │   ├── dock_2016_base_dynamic                          (NaviGator Dock 2016 Base Dynamic Model)
│   │       │   │   │   ├── dock_2016_dynamic                               (NaviGator Dock 2016 Dynamic Model)
│   │       │   │   │   ├── dock_2018                                       (NaviGator Dock 2018 Model)
│   │       │   │   │   ├── dock_2018_base                                  (NaviGator Dock 2018 Base Model)
│   │       │   │   │   ├── dock_2018_base_dynamic                          (NaviGator Dock 2018 Base Dynamic Model)
│   │       │   │   │   ├── dock_2018_dynamic                               (NaviGator Dock 2018 Dynamic Model)
│   │       │   │   │   ├── dock_2022                                       (NaviGator Dock 2022 Model)
│   │       │   │   │   ├── dock_2022_base                                  (NaviGator Dock 2022 Base Model)
│   │       │   │   │   ├── dock_2022_base_dynamic                          (NaviGator Dock 2022 Base Dynamic Model)
│   │       │   │   │   ├── dock_2022_dynamic                               (NaviGator Dock 2022 Dynamic Model)
│   │       │   │   │   ├── dock_block                                      (NaviGator Dock Block Model)
│   │       │   │   │   │   └── mesh
│   │       │   │   │   ├── dock_block_2x2                                  (NaviGator Dock Block 2x2 Model)
│   │       │   │   │   │   └── mesh
│   │       │   │   │   ├── dock_block_3x3                                  (NaviGator Dock Block 3x3 Model)
│   │       │   │   │   │   └── mesh
│   │       │   │   │   ├── dock_block_4x4                                  (NaviGator Dock Block 4x4 Model)
│   │       │   │   │   │   └── mesh
│   │       │   │   │   ├── dock_block_4x4_dynamic                          (NaviGator Dock Block 4x4 Dynamic Model)
│   │       │   │   │   │   └── mesh
│   │       │   │   │   ├── dock_permutations                               (NaviGator Dock Permuations of Shapes and Colors)
│   │       │   │   │   │   ├── vrx_dock_blue_circle__blue_circle   
│   │       │   │   │   │   ├── vrx_dock_blue_circle__blue_cross
│   │       │   │   │   │   ├── vrx_dock_blue_circle__blue_triangle
│   │       │   │   │   │   ├── vrx_dock_blue_circle__green_circle
│   │       │   │   │   │   ├── vrx_dock_blue_circle__green_cross
│   │       │   │   │   │   ├── vrx_dock_blue_circle__green_triangle
│   │       │   │   │   │   ├── vrx_dock_blue_circle__red_circle
│   │       │   │   │   │   ├── vrx_dock_blue_circle__red_cross
│   │       │   │   │   │   ├── vrx_dock_blue_circle__red_triangle
│   │       │   │   │   │   ├── vrx_dock_blue_cross__blue_circle
│   │       │   │   │   │   ├── vrx_dock_blue_cross__blue_cross
│   │       │   │   │   │   ├── vrx_dock_blue_cross__blue_triangle
│   │       │   │   │   │   ├── vrx_dock_blue_cross__green_circle
│   │       │   │   │   │   ├── vrx_dock_blue_cross__green_cross
│   │       │   │   │   │   ├── vrx_dock_blue_cross__green_triangle
│   │       │   │   │   │   ├── vrx_dock_blue_cross__red_circle
│   │       │   │   │   │   ├── vrx_dock_blue_cross__red_cross
│   │       │   │   │   │   ├── vrx_dock_blue_cross__red_triangle
│   │       │   │   │   │   ├── vrx_dock_blue_triangle__blue_circle
│   │       │   │   │   │   ├── vrx_dock_blue_triangle__blue_cross
│   │       │   │   │   │   ├── vrx_dock_blue_triangle__blue_triangle
│   │       │   │   │   │   ├── vrx_dock_blue_triangle__green_circle
│   │       │   │   │   │   ├── vrx_dock_blue_triangle__green_cross
│   │       │   │   │   │   ├── vrx_dock_blue_triangle__green_triangle
│   │       │   │   │   │   ├── vrx_dock_blue_triangle__red_circle
│   │       │   │   │   │   ├── vrx_dock_blue_triangle__red_cross
│   │       │   │   │   │   ├── vrx_dock_blue_triangle__red_triangle
│   │       │   │   │   │   ├── vrx_dock_green_circle__blue_circle
│   │       │   │   │   │   ├── vrx_dock_green_circle__blue_cross
│   │       │   │   │   │   ├── vrx_dock_green_circle__blue_triangle
│   │       │   │   │   │   ├── vrx_dock_green_circle__green_circle
│   │       │   │   │   │   ├── vrx_dock_green_circle__green_cross
│   │       │   │   │   │   ├── vrx_dock_green_circle__green_triangle
│   │       │   │   │   │   ├── vrx_dock_green_circle__red_circle
│   │       │   │   │   │   ├── vrx_dock_green_circle__red_cross
│   │       │   │   │   │   ├── vrx_dock_green_circle__red_triangle
│   │       │   │   │   │   ├── vrx_dock_green_cross__blue_circle
│   │       │   │   │   │   ├── vrx_dock_green_cross__blue_cross
│   │       │   │   │   │   ├── vrx_dock_green_cross__blue_triangle
│   │       │   │   │   │   ├── vrx_dock_green_cross__green_circle
│   │       │   │   │   │   ├── vrx_dock_green_cross__green_cross
│   │       │   │   │   │   ├── vrx_dock_green_cross__green_triangle
│   │       │   │   │   │   ├── vrx_dock_green_cross__red_circle
│   │       │   │   │   │   ├── vrx_dock_green_cross__red_cross
│   │       │   │   │   │   ├── vrx_dock_green_cross__red_triangle
│   │       │   │   │   │   ├── vrx_dock_green_triangle__blue_circle
│   │       │   │   │   │   ├── vrx_dock_green_triangle__blue_cross
│   │       │   │   │   │   ├── vrx_dock_green_triangle__blue_triangle
│   │       │   │   │   │   ├── vrx_dock_green_triangle__green_circle
│   │       │   │   │   │   ├── vrx_dock_green_triangle__green_cross
│   │       │   │   │   │   ├── vrx_dock_green_triangle__green_triangle
│   │       │   │   │   │   ├── vrx_dock_green_triangle__red_circle
│   │       │   │   │   │   ├── vrx_dock_green_triangle__red_cross
│   │       │   │   │   │   ├── vrx_dock_green_triangle__red_triangle
│   │       │   │   │   │   ├── vrx_dock_red_circle__blue_circle
│   │       │   │   │   │   ├── vrx_dock_red_circle__blue_cross
│   │       │   │   │   │   ├── vrx_dock_red_circle__blue_triangle
│   │       │   │   │   │   ├── vrx_dock_red_circle__green_circle
│   │       │   │   │   │   ├── vrx_dock_red_circle__green_cross
│   │       │   │   │   │   ├── vrx_dock_red_circle__green_triangle
│   │       │   │   │   │   ├── vrx_dock_red_circle__red_circle
│   │       │   │   │   │   ├── vrx_dock_red_circle__red_cross
│   │       │   │   │   │   ├── vrx_dock_red_circle__red_triangle
│   │       │   │   │   │   ├── vrx_dock_red_cross__blue_circle
│   │       │   │   │   │   ├── vrx_dock_red_cross__blue_cross
│   │       │   │   │   │   ├── vrx_dock_red_cross__blue_triangle
│   │       │   │   │   │   ├── vrx_dock_red_cross__green_circle
│   │       │   │   │   │   ├── vrx_dock_red_cross__green_cross
│   │       │   │   │   │   ├── vrx_dock_red_cross__green_triangle
│   │       │   │   │   │   ├── vrx_dock_red_cross__red_circle
│   │       │   │   │   │   ├── vrx_dock_red_cross__red_cross
│   │       │   │   │   │   ├── vrx_dock_red_cross__red_triangle
│   │       │   │   │   │   ├── vrx_dock_red_triangle__blue_circle
│   │       │   │   │   │   ├── vrx_dock_red_triangle__blue_cross
│   │       │   │   │   │   ├── vrx_dock_red_triangle__blue_triangle
│   │       │   │   │   │   ├── vrx_dock_red_triangle__green_circle
│   │       │   │   │   │   ├── vrx_dock_red_triangle__green_cross
│   │       │   │   │   │   ├── vrx_dock_red_triangle__green_triangle
│   │       │   │   │   │   ├── vrx_dock_red_triangle__red_circle
│   │       │   │   │   │   ├── vrx_dock_red_triangle__red_cross
│   │       │   │   │   │   └── vrx_dock_red_triangle__red_triangle
│   │       │   │   │   ├── foldable_chair                                  (NaviGator Foldable Chair Model)
│   │       │   │   │   │   └── mesh                                        (NaviGator Foldable Chair Mesh)
│   │       │   │   │   ├── foldable_table                                  (NaviGator Foldable Table Model)
│   │       │   │   │   │   └── mesh                                        (NaviGator Foldable Table Mesh)
│   │       │   │   │   ├── green_totem                                     (NaviGator Green Totem Model)
│   │       │   │   │   ├── ground_station                                  (NaviGator Ground Station Model)
│   │       │   │   │   ├── mb_marker_buoy_black                            (NaviGator MB Marker Buoy Black Model)
│   │       │   │   │   ├── mb_marker_buoy_green                            (NaviGator MB Marker Buoy Green Model)
│   │       │   │   │   │   ├── materials                                   (NaviGator MB Marker Buoy Green Materials)
│   │       │   │   │   │   │   └── scripts                                 (NaviGator MB Marker Buoy Green Scripts)
│   │       │   │   │   │   └── meshes                                      (NaviGator MB Marker Buoy Meshes)
│   │       │   │   │   ├── mb_marker_buoy_red                              (NaviGator MB Marker Buoy Red Model)
│   │       │   │   │   │   ├── materials                                   (NaviGator MB Marker Buoy Red Materials)
│   │       │   │   │   │   │   ├── scripts                                 (NaviGator MB Marker Buoy Red Scripts)
│   │       │   │   │   │   │   └── textures                                (NaviGator MB Marker Buoy Red Textures)
│   │       │   │   │   │   └── meshes                                      (NaviGator MB Marker Buoy Red Meshes)
│   │       │   │   │   ├── mb_marker_buoy_white                            (NaviGator MB Marker Buoy White Model)
│   │       │   │   │   ├── mb_round_buoy_black                             (NaviGator MB Round Buoy Black Model)
│   │       │   │   │   ├── mb_round_buoy_orange                            (NaviGator MB Round Buoy Orange Model)
│   │       │   │   │   │   ├── materials                                   (NaviGator MB Round Buoy Orange Materials)
│   │       │   │   │   │   │   ├── scripts                                 (NaviGator MB Round Buoy Orange Scripts)
│   │       │   │   │   │   │   └── textures                                (NaviGator MB Round Buoy Orange Textures)
│   │       │   │   │   │   └── meshes                                      (NaviGator MB Round Buoy Orange Meshes)
│   │       │   │   │   ├── mono_camera                                     (NaviGator Mono Camera Model)
│   │       │   │   │   │   └── mesh                                        (NaviGator Mono Camera Mesh)
│   │       │   │   │   ├── navigation_course                               (NaviGator Navigation Course Model)
│   │       │   │   │   ├── navigation_course0                              (NaviGator Navigation Course0 Model)
│   │       │   │   │   ├── navigation_course1                              (NaviGator Navigation Course1 Model)
│   │       │   │   │   ├── navigation_course2                              (NaviGator Navigation Course2 Model)
│   │       │   │   │   ├── navigation_course3                              (NaviGator Navigation Course3 Model)
│   │       │   │   │   ├── navigation_course4                              (NaviGator Navigation Course4 Model)
│   │       │   │   │   ├── navigation_course5                              (NaviGator Navigation Course5 Model)
│   │       │   │   │   ├── navigation_phase2_0                             (NaviGator Navigation Phase2_0 Model)
│   │       │   │   │   ├── navigation_phase2_1                             (NaviGator Navigation Phase2_1 Model)
│   │       │   │   │   ├── navigation_phase2_2                             (NaviGator Navigation Phase2_2 Model)
│   │       │   │   │   ├── navigation_phase2_3                             (NaviGator Navigation Phase2_3 Model)
│   │       │   │   │   ├── navigation_phase2_4                             (NaviGator Navigation Phase2_4 Model)
│   │       │   │   │   ├── navigation_task                                 (NaviGator Navigation Task Model)
│   │       │   │   │   ├── obstacle_course                                 (NaviGator Obstacle Course Model)
│   │       │   │   │   ├── obstacle_course0                                (NaviGator Obstacle Course0 Model)
│   │       │   │   │   ├── obstacle_course1                                (NaviGator Obstacle Course1 Model)
│   │       │   │   │   ├── obstacle_course2                                (NaviGator Obstacle Course2 Model)
│   │       │   │   │   ├── obstacles_phase2_0                              (NaviGator Obstacles Phase2_0 Model)
│   │       │   │   │   ├── obstacles_phase2_1                              (NaviGator Obstacles Phase2_1 Model)
│   │       │   │   │   ├── obstacles_phase2_2                              (NaviGator Obstacles Phase2_2 Model)
│   │       │   │   │   ├── obstacles_phase2_3                              (NaviGator Obstacles Phase2_3 Model)
│   │       │   │   │   ├── ocean                                           (NaviGator Ocean Model)
│   │       │   │   │   │   ├── materials                                   (NaviGator Ocean Materials)
│   │       │   │   │   │   │   ├── programs                                (NaviGator Ocean Programs)
│   │       │   │   │   │   │   │   └── GLSL                                (NaviGator Ocean GLSL Files)
│   │       │   │   │   │   │   ├── scripts                                 (NaviGator Ocean Scripts)
│   │       │   │   │   │   │   └── textures                                (NaviGator Ocean Textures)
│   │       │   │   │   │   └── meshes                                      (NaviGator Ocean Meshes)
│   │       │   │   │   ├── placard                                         (NaviGator Placard Model)
│   │       │   │   │   ├── placard_2022                                    (NaviGator Placard 2022 Model)
│   │       │   │   │   │   ├── materials
│   │       │   │   │   │   │   └── textures
│   │       │   │   │   │   └── meshes                                      (NaviGator Placard 2022 Meshes)
│   │       │   │   │   ├── platypus_buoy                                   (NaviGator Platypus Buoy Model)
│   │       │   │   │   │   ├── materials
│   │       │   │   │   │   │   ├── scripts                                 (NaviGator Platypus Buoy Scripts)
│   │       │   │   │   │   │   └── textures                                (NaviGator Platypus Buoy Textures)
│   │       │   │   │   │   └── meshes                                      (NaviGator Platypus Buoy Meshes)
│   │       │   │   │   ├── platypus_static                                 (NaviGator Platypus Static Model)
│   │       │   │   │   ├── polyform_a3                                     (NaviGator Polyform A3 Model)
│   │       │   │   │   ├── polyform_a5                                     (NaviGator Polyform A5 Model)
│   │       │   │   │   ├── polyform_a7                                     (NaviGator Polyform A7 Model)
│   │       │   │   │   ├── post                                            (NaviGator Post Model)
│   │       │   │   │   │   ├── materials                                   (NaviGator Post Materials)
│   │       │   │   │   │   │   └── textures                                (NaviGator Post Textures)
│   │       │   │   │   │   └── meshes                                      (NaviGator Post Meshes)
│   │       │   │   │   ├── red_totem                                       (NaviGator Red Totem Model)
│   │       │   │   │   ├── robotx_2016_finals_pinger_transit               (NaviGator RobotX 2016 Finals Pinger Transit Model)
│   │       │   │   │   ├── robotx_2016_qualifying_pinger_transit           (NaviGator RobotX 2016 Qualifying Pinger Transit Model)
│   │       │   │   │   ├── robotx_2018_entrance_gate                       (NaviGator RobotX 2018 Entrance Gate Model)
│   │       │   │   │   ├── robotx_2018_qualifying_avoid_obstacles          (NaviGator RobotX 2018 Qualifying Avoid Obstacles Model)
│   │       │   │   │   ├── robotx_2018_qualifying_avoid_obstacles_buoys    (NaviGator RobotX 2018 Qualifying Avoid Obstacles Buoys Model)
│   │       │   │   │   ├── robotx_light_buoy                               (NaviGator RobotX Light Buoy Moel)
│   │       │   │   │   │   └── mesh
│   │       │   │   │   ├── robotx_navigation_challenge                     (NaviGator RobotX Navigation Challenge Model)
│   │       │   │   │   ├── sandisland                                      (NaviGator Sand Island Model)
│   │       │   │   │   │   ├── materials                                   (NaviGator Sand Island Materials) 
│   │       │   │   │   │   │   └── textures                                (NaviGator Sand Island Textures)
│   │       │   │   │   │   └── meshes                                      (NaviGator Sand Island Meshes)
│   │       │   │   │   ├── sensor_post                                     (NaviGator Sensor Post Model)
│   │       │   │   │   │   └── mesh                                        (NaviGator Sensor Post Mesh)
│   │       │   │   │   ├── short_navigation_course0                        (NaviGator Short Navigation Course0 Model)
│   │       │   │   │   ├── short_navigation_course1                        (NaviGator Short Navigation Course1 Model)
│   │       │   │   │   ├── short_navigation_course2                        (NaviGator Short Navigation Course2 Model)
│   │       │   │   │   ├── surmark46104                                    (NaviGator Surmark46104 Model)
│   │       │   │   │   │   └── mesh                                        (NaviGator Surmark46104 Mesh)
│   │       │   │   │   ├── surmark950400                                   (NaviGator Surmark959488 Model)
│   │       │   │   │   │   ├── materials                                   (NaviGator Surmark959488 Materials)
│   │       │   │   │   │   │   └── scripts                                 (NaviGator Surmark959488 Scripts)
│   │       │   │   │   │   └── mesh                                        (NaviGator Surmark959488 Mesh)
│   │       │   │   │   ├── surmark950410                                   (NaviGator Surmark950410 Model)
│   │       │   │   │   │   ├── materials                                   (NaviGator Surmark950410 Materials)
│   │       │   │   │   │   │   └── scripts                                 (NaviGator Surmark950410 Scripts)
│   │       │   │   │   │   └── mesh                                        (NaviGator Surmark950410 Mesh)
│   │       │   │   │   ├── sydney_regatta                                  (NaviGator Sydney Regatta Model)
│   │       │   │   │   │   ├── materials                                   (NaviGator Sydney Regatta Materials)
│   │       │   │   │   │   │   ├── scripts                                 (NaviGator Sydney Regatta Scripts)
│   │       │   │   │   │   │   └── textures                                (NaviGator Sydney Regatta Textures)
│   │       │   │   │   │   └── meshes                                      (NaviGator Sydney Regatta Meshes)
│   │       │   │   │   ├── symbol_circle                                   (NaviGator Symbol Circle Model)
│   │       │   │   │   ├── symbol_cross                                    (NaviGator Symbol Cross Model)
│   │       │   │   │   ├── symbol_rectangle                                (NaviGator Symbol Rectangle Model)
│   │       │   │   │   ├── symbol_triangle                                 (NaviGator Symbol Triangle Model)
│   │       │   │   │   │   └── mesh                                        (NaviGator Symbol Mesh)
│   │       │   │   │   ├── tent                                            (NaviGator Tent Model)
│   │       │   │   │   │   └── mesh                                        (NaviGator Tent Mesh)
│   │       │   │   │   ├── turtle_buoy                                     (NaviGator Turtle Buoy Model)
│   │       │   │   │   │   ├── materials                                   (NaviGator Turtle Buoy Materials)
│   │       │   │   │   │   │   ├── scripts                                 (NaviGator Turtle Buoy Scripts)
│   │       │   │   │   │   │   └── textures                                (NaviGator Turtle Buoy Textures)
│   │       │   │   │   │   └── meshes                                      (NaviGator Turtle Buoy Meshes)
│   │       │   │   │   ├── turtle_static                                   (NaviGator Turtle Static Model)
│   │       │   │   │   ├── wavegauge                                       (NaviGator Wavegauge Model)
│   │       │   │   │   └── yellow_totem                                    (NaviGator Yellow Totem Model)
│   │       │   │   ├── msg                                                 (NaviGator VRX Gazebo Message)
│   │       │   │   ├── msgs                                                (NaviGator VRX Gazebo Messages)
│   │       │   │   ├── nodes                                               (NaviGator VRX Gazebo Nodes)
│   │       │   │   ├── scripts                                             (NaviGator VRX Gazebo Scripts)
│   │       │   │   ├── src                                                 (NaviGator VRX Gazebo Source)
│   │       │   │   │   ├── vrx_gazebo                                      (NaviGator VRX Gazebo Source Code)
│   │       │   │   │   │   └── __pycache__                                 (NaviGator VRX Gazebo Pycache)
│   │       │   │   ├── srv                                                 (NaviGator VRX Gazebo SRV)
│   │       │   │   ├── test                                                (NaviGator VRX Gazebo Test)
│   │       │   │   ├── utils                                               (NaviGator VRX Gazebo Utils)
│   │       │   │   └── worlds                                              (NaviGator VRX Gazebo Worlds)
│   │       │   │       ├── 2019_phase2                                     (2019 Phase 2 Worlds)
│   │       │   │       ├── 2019_practice                                   (2019 Practice Worlds)
│   │       │   │       ├── 2022_phase2                                     (2022 Phase 2 Worlds)
│   │       │   │       ├── 2022_practice                                   (2022 Practice Worlds)
│   │       │   │       ├── xacros                                          (NaviGator VRX Gazebo Xacros)
│   │       │   │       └── yamls                                           (NaviGator VRX Gazebo YAMLs)
│   │       │   ├── wamv_description                                        (NaviGator VRX WamV Description)
│   │       │   │   ├── models                                              (NaviGator VRX WamV Models)
│   │       │   │   │   ├── engine                                          (NaviGator VRX WamV Engine Model)
│   │       │   │   │   │   └── mesh                                        (NaviGator VRX WamV Engine Mesh)
│   │       │   │   │   ├── propeller                                       (NaviGator VRX WamV Propeller Model)
│   │       │   │   │   │   └── mesh                                        (NaviGator VRX WamV Propeller Mesh)
│   │       │   │   │   └── WAM-V-Base                                      (NaviGator VRX WAMV Base Model)
│   │       │   │   │       └── mesh                                        (NaviGator VRX WAMV Base Mesh)
│   │       │   │   └── urdf                                                (NaviGator VRX WAMV URDF)
│   │       │   │       └── thrusters                                       (NaviGator VRX WAMV Thruster Xacros)
│   │       │   ├── wamv_gazebo                                             (NaviGator VRX WAMV Gazebo)
│   │       │   │   ├── config                                              (NaviGator VRX WAMV Gazebo Config)
│   │       │   │   ├── launch                                              (NaviGator VRX WAMV Gazebo Launch Code)
│   │       │   │   ├── models                                              (NaviGator VRX WAMV Gazebo Models)
│   │       │   │   │   └── gps                                             (NaviGator VRX WAMV Gazebo GPS Model)
│   │       │   │   │       └── mesh                                        (NaviGator VRX WAMV Gazebo GPS Mesh)
│   │       │   │   └── urdf                                                (NaviGator VRX WAMV Gazebo URDF)
│   │       │   │       ├── components                                      (NaviGator VRX WAMV Gazebo URDF Xacro Components)
│   │       │   │       ├── dynamics                                        (NaviGator VRX WAMV Gazebo URDF Dynamics)
│   │       │   │       └── thruster_layouts                                (NaviGator VRX WAMV Gazebo URDF Thruster Layouts)
│   │       │   ├── wave_gazebo                                             (NaviGator Wave Gazebo)
│   │       │   │   ├── config                                              (NaviGator Wave Gazebo Config)
│   │       │   │   ├── launch                                              (NaviGator Wave Gazebo Launch Code)
│   │       │   │   ├── world_models                                        (NaviGator Wave Gazebo Worlds)
│   │       │   │   │   └── ocean_waves                                     (NaviGator Wave Gazebo Ocean Waves)
│   │       │   │   │       ├── materials                                   (NaviGator Wave Gazebo Ocean Waves Materials)
│   │       │   │   │       │   ├── programs                                (NaviGator Wave Gazebo Ocean Waves Programs)
│   │       │   │   │       │   ├── scripts                                 (NaviGator Wave Gazebo Ocean Waves Scripts)
│   │       │   │   │       │   └── textures                                (NaviGator Wave Gazebo Ocean Waves Textures)
│   │       │   │   │       └── meshes                                      (NaviGator Wave Gazebo Ocean Waves Meshes)
│   │       │   │   └── worlds                                              (NaviGator Wave Gazebo Worlds)
│   │       │   └── wave_gazebo_plugins                                     (NaviGator Wave Gazebo Plugins)
│   │       │       ├── include                                             (NaviGator Wave Gazebo Include)
│   │       │       │   └── wave_gazebo_plugins                             (NaviGator Wave Gazebo Plugin Files)
│   │       │       └── src                                                 (NaviGator Wave Gazebo Plugins Source Code)
│   │       ├── vrx-docker                                                  (NaviGator VRX Automated Elevation)
│   │       │   ├── generated                                               (NaviGator VRX Docker Genrated Worlds)
│   │       │   │   └── task_generated                                      (NaviGator VRX Task Generated Worlds)
│   │       │   │       ├── dock                                            (NaviGator VRX Dock)
│   │       │   │       │   ├── worlds                                      (NaviGator VRX Dock Worlds)
│   │       │   │       │   └── world_xacros                                (NaviGator VRX Dock World Xacros)
│   │       │   │       ├── nav_challenge                                   (NaviGator VRX Dock Nav Challenge)
│   │       │   │       │   ├── worlds                                      (NaviGator VRX Dock Nav Challenge Worlds)
│   │       │   │       │   └── world_xacros                                (NaviGator VRX Dock Nav Challenge Xacros)
│   │       │   │       ├── perception                                      (NaviGator VRX Dock Perception)
│   │       │   │       │   ├── worlds                                      (NaviGator VRX Dock Perception Worlds)
│   │       │   │       │   └── world_xacros                                (NaviGator VRX Dock Perception World Xacros)
│   │       │   │       ├── scan_and_dock                                   (NaviGator VRX Scan and Dock)
│   │       │   │       │   ├── worlds                                      (NaviGator VRX Scan and Dock Worlds)
│   │       │   │       │   └── world_xacros                                (NaviGator VRX Scan and Dock World Xacros)
│   │       │   │       ├── stationkeeping                                  (NaviGator VRX Stationkeeping)
│   │       │   │       │   ├── worlds                                      (NaviGator VRX Stationkeeping Worlds)
│   │       │   │       │   └── world_xacros                                (NaviGator VRX Stationkeeping World Xacros)
│   │       │   │       └── wayfinding                                      (NaviGator VRX Wayfinding)
│   │       │   │           ├── worlds                                      (NaviGator VRX Wayfinding Worlds)
│   │       │   │           └── world_xacros                                (NaviGator VRX Wayfinding World Xacros)                             
│   │       │   ├── multi_scripts                                           (NaviGator VRX Docker Multi Scripts)
│   │       │   ├── task_config                                             (NaviGator VRX Docker Task Config)
│   │       │   ├── team_config                                             (NaviGator VRX Docker Team Config)
│   │       │   │   ├── example_team                                        (NaviGator VRX Docker Example Team)
│   │       │   │   └── example_team_2                                      (NaviGator VRX Docker Example Team 2)
│   │       │   ├── utils                                                   (NaviGator VRX Docker Utils)
│   │       │   └── vrx_server                                              (NaviGator VRX Docker Server)
│   │       │       └── vrx-server                                          (NaviGator VRX Docker Server Shell Scripts)
│   │       │           ├── Dockerfile
│   │       └── vrx_logs                                                    (NaviGator VRX Logs)
│   │           └── 2019                                                    (NaviGator VRX Logs 2019)
│   ├── test                                                                (NaviGator Test)
│   │   └── navigator_test
│   │       ├── navigator_testing_suite                                     (NaviGator Testing Suite)
│   │       │   └── mission_perception_test                                 (NaviGator Mission Perception Test Code)
│   │       ├── navigator_test_lib                                          (NaviGator Test Lib)
│   │       ├── navigator_tests                                             (NaviGator Tests Code)                                 
│   │       │   └──mission_planner_yamls                                    (NaviGator Test Mission Planenr YAMLs)
│   │       └── nodes                                                       (NaviGator Test Nodes)
│   └── utils                                                               (NaviGator Utils)
│       ├── navigator_battery_monitor                                       (NaviGator Battery Monitor)
│       │   ├── nodes                                                       (NaviGator Battery Moniter Nodes)     
│       ├── navigator_gui                                                   (NaviGator Utils GUI)
│       │   ├── navigator_gui                                               (NaviGator GUI Code)
│       │   └── resource                                                    (NaviGator GUI Resources)
│       ├── navigator_judgepanel                                            (NaviGator Judge Panel)
│       │   └── src                                                         (NaviGator Judge Panel Source Code)
│       ├── navigator_msgs                                                  (NaviGator Utils Messages)
│       │   ├── action                                                      (NaviGator Utils Messages Actions)
│       │   ├── msg                                                         (NaviGator Utils Message Files)
│       │   └── srv                                                         (NaviGator Utils SRV)
│       ├── navigator_robotx_comms                                          (NaviGator RobotX Comms)
│       │   ├── navigator_robotx_comms                                      (NaviGator RobotX Comms Code)
│       │   ├── nodes                                                       (NaviGator RobotX Nodes)
│       │   └── test                                                        (NaviGator RobotX Test Code)
│       ├── navigator_tools                                                 (NaviGator Tools)
│       │   ├── cfg                                                         (NaviGator Tools Config)
│       │   ├── navigator_tools                                             (NaviGator Tools Code)
│       │   └── nodes                                                       (NaviGator Tools Nodes)
│       │       ├── limit_switch
│       │       ├── navigator_status_tui
│       │       └── sylphase_ros_driver
│       ├── remote_control                                                  (NaviGator Remote Control)
│       │   ├── navigator_emergency_control                                 (NaviGator Emergency Control)
│       │   │   └── nodes                                                   (NaviGator Emergency Control Nodes)
│       │   ├── navigator_joystick_control                                  (NaviGator Joystick Control)
│       │   │   └── nodes                                                   (NaviGator Joystick Control Nodes)
│       │   └── navigator_keyboard_control                                  (NaviGator Keyboard Control)
│       │       ├── nodes                                                   (NaviGator Keyboard Control Nodes)
│       │       └── remote_control_lib                                      (NaviGator Keyboard Control Remote Control Library)
│       └── voltage_gui                                                     (NaviGator Voltage GUI)
│           ├── resource                                                    (NaviGator Voltage GUI Resources)
│           └── src                                                         (NaviGator Voltage GUI Source Code)
├── proprietary                                                             (Proprietary)
├── provisioning                                                            (Provisioning)
├── scripts                                                                 (Scripts for ROS and Hardware)
│   ├── build_docker_containers
│   ├── build_docs
│   ├── build_vrx_trial_container
│   ├── display_docs
│   ├── dynparam -> ../docker/vrx_trial/dynparam
│   ├── fix_ownership_for_docker
│   ├── hardware_installers                                                 (Hardware Installers)
│   │   ├── install_bvtsdk
│   │   ├── install_flycap
│   │   ├── install_sylphase_sonar
│   │   └── install_udev_rules
│   ├── mount_fileserver
│   ├── rc.d                                                                (RC.D Shell Commands)
│   ├── run_development_container
│   ├── run_vrx_trial_container
│   ├── sshzobelisk
│   ├── system_install -> ../docker/base/system_install
│   ├── umount_fileserver
│   └── user_install -> ../docker/dev/user_install
└── SubjuGator                                                              (SubjuGator)
    ├── command                                                             (SubjuGator Command)
    │   ├── subjugator_alarm                                                (SubjuGator Alarm)
    │   │   ├── alarm_handlers                                              (SubjuGator Alarm Handlers Code)
    │   │   ├── launch                                                      (SubjuGator Alarm Launch)
    │   │   └── nodes                                                       (SubjuGator Alarm Nodes)
    │   ├── subjugator_keyboard_control                                     (SubjuGator Keyboard Control)
    │   ├── subjugator_launch                                               (SubjuGator Launch)
    │   │   ├── config                                                      (SubjuGator Launch Config)
    │   │   ├── launch                                                      (SubjuGator Launch Code)
    │   │   │   └── subsystems                                              (SubjuGator Launch Code Subsystems)
    │   │   │       └── cameras                                             (SubjuGator Launch Subsystems Cameras)
    │   │   │           ├── calibration                                     (SubjuGator Launch Subsystems Cameras Calibration)
    │   │   │           └── launch                                          (SubjuGator Launch Subsystems Cameras Launch)
    │   │   └── scripts                                                     (SubjuGator Launch Scripts)
    │   │       ├── depth_conn
    │   │       ├── dvl_conn
    │   │       └── imu_conn
    │   └── subjugator_missions                                             (SubjuGator Missiosn)
    │       ├── subjugator_missions                                         (SubjuGator Missions Code)
    │       └── tools                                                       (SubjuGator Missions Tools)
    │           ├── sub8
    │           └── sub9
    ├── docs                                                                (Docs)
    ├── drivers                                                             (Drivers)
    │   ├── magnetic_compensation                                           (Magnetic Compensation Drivers)
    │   │   ├── sub8_magnetic_dynamic_compensation                          (Sub8 Magnetic Dynamic Compensation Drivers)
    │   │   │   ├── msg                                                     (Sub8 Magnetic Dynamic Compensation Driver Messages)
    │   │   │   └── src                                                     (Sub8 Magnetic Dynamic Compensation Drivers Source Code)
    │   │   └── sub8_magnetic_hardsoft_compensation                         (Sub8 Magnetic Hardsoft Compensation Drivers)
    │   │       ├── scripts                                                 (Sub8 Magnetic Hardsoft Compensation Drivers Scripts)
    │   │       │   └── generate_config
    │   │       └── src                                                     (Sub8 Magnetic Hardsoft Compensation Drivers Source Code)
    │   ├── sub8_actuator_board                                             (Sub8 Actuator Board Drivers)
    │   │   ├── srv
    │   │   └── sub8_actuator_board                                         (Sub8 Actuator Board Drivers Code)
    │   ├── sub8_adis16400_imu                                              (Sub8 Adis16400 IMU Drivers)
    │   │   ├── include                                                     (Sub8 Adis16400 IMU Drivers Include)
    │   │   │   └── adis16400_imu                                           (Sub8 Adis16400 IMU Drivers H Files)
    │   │   └── src                                                         (Sub8 Adis16400 IMU Drivers Source Code)
    │   ├── sub8_depth_driver                                               (Sub8 Depth Drivers)
    │   │   ├── include                                                     (Sub8 Depth Drivers Include)
    │   │   │   └── depth_driver                                            (Sub8 Depth Drivers Depth Driver H Files)
    │   │   ├── scripts                                                     (Sub8 Depth Drivers Scripts)
    │   │   │   └── fake_depth
    │   │   └── src                                                         (Sub8 Depth Drivers Source Code)
    │   ├── sub8_driver_utils                                               (Sub8 Driver Utils)
    │   │   └── camera_utils                                                (Sub8 Driver Camera Utils)
    │   │       └── src                                                     (Sub8 DC1394 Driver Code)
    │   ├── sub8_rdi_dvl                                                    (Sub8 RDI DVL)
    │   │   ├── include                                                     (Sub8 RDI DVL Include)
    │   │   │   └── rdi_explorer_dvl
    │   │   ├── scripts                                                     (Sub8 RDI DVL Scripts)
    │   │   │   └── fake_dvl
    │   │   └── src
    │   ├── sub8_thrust_and_kill_board                                      (Sub8 Thrust and Kill Board)
    │   │   └── sub8_thrust_and_kill_board                                  (Sub8 Thrust and Kill Board Code)
    │   ├── sub9_thrust_and_kill_board                                      (Sub9 Thrust and Kill Board)
    │   │   ├── sub9_thrust_and_kill_board                                  (Sub9 Thrust and Kill Board Code)
    │   │   └── test                                                        (Sub9 Thrust and Kill Board Test)
    │   └── sub_actuator_board                                              (Sub9 Actuator Board)
    │       ├── srv
    │       ├── sub_actuator_board
    │       └── test
    ├── etc                                                                 (Etc)
    │   ├── netplan
    │   └── rc.local
    ├── gnc                                                                 (GNC)
    │   ├── c3_trajectory_generator                                         (C3 Trajectory Genrator)
    │   │   ├── include                                                     (C3 Trajectory Genrator Include)
    │   │   ├── src                                                         (C3 Trajectory Genrator Source)
    │   │   └── srv                                                         (C3 Trajectory Genrator SRV)
    │   ├── rise_6dof                                                       (Rise 6Dof)
    │   │   ├── cfg                                                         (Rise 6Dof Config)
    │   │   ├── scripts                                                     (Rise 6Dof Scripts)
    │   │   ├── src                                                         (Rise 6Dof Source)
    │   │   │   └── rise_6dof                                               (Rise 6Dof Code)
    │   │   └── srv                                                         (Rise 6Dof SRV)
    │   ├── subjugator_controller                                           (SubjuGator Controller)
    │   │   ├── cfg                                                         (SubjuGator Controller Config)
    │   │   └── nodes                                                       (SubjuGator Controller Nodes)
    │   │       └── adaptive_controller                                     (SubjuGator Controller Adaptive Controller)
    │   ├── subjugator_system_id                                            (SubjuGator System ID)
    │   │   └── subjugator_system_id
    │   └── subjugator_thruster_mapper                                      (SubjuGator Thruster Mapper)
    │       ├── config                                                      (SubjuGator Thruster Mapper Config)
    │       ├── nodes                                                       (SubjuGator Thruster Mapper Nodes)
    │       └── test                                                        (SubjuGator Thruster Mapper Test)
    ├── Jenkinsfile
    ├── perception                                                          (Perception)
    │   ├── subjugator_perception                                           (SubjuGator Perception)
    │   │   ├── cfg                                                         (SubjuGator Perception Config)
    │   │   ├── include                                                     (SubjuGator Perception Include)
    │   │   │   ├── subjugator_perception
    │   │   │   └── subjugator_vision_lib                                   (SubjuGator Perception Lib Files)
    │   │   ├── ml_classifiers                                              (SubjuGator Perception ML Classifiers)
    │   │   │   ├── dice                                                    (ML Classifiers Dice)
    │   │   │   │   ├── frozen_graphs
    │   │   │   │   │   └── faster_rcnn_243_dice_v2
    │   │   │   │   ├── protos                                              (ML Classifiers Protos)
    │   │   │   │   └── utils                                               (ML Classifiers Utils)
    │   │   │   └── path_marker                                             (ML Classifiers Path Marker)
    │   │   │       ├── Path_Inference                                      (ML Classifiers Path Inference)
    │   │   │       │   └── faster_rcnn_inception_v2_path
    │   │   │       │       └── saved_model
    │   │   │       ├── protos                                              (ML Classifiers Protos)
    │   │   │       └── utils                                               (ML Classifiers Path Marker Utils)
    │   │   ├── nodes                                                       (SubjuGator Perception Nodes)
    │   │   ├── src                                                         (SubjuGator Perception Source)
    │   │   │   ├── subjugator_perception                   
    │   │   │   └── subjugator_vision_lib                                   (SubjuGator Vision Lib)
    │   │   ├── subjugator_vision_tools                                     (SubjuGator Vision Tools)
    │   │   │   ├── HOG                                                     (SubjuGator Vision HOG)
    │   │   │   ├── labelling                                               (SubjuGator Vision Labelling)
    │   │   │   └── machine_learning                                        (SubjuGator Vision Machine Learning)
    │   │   └── test                                                        (SubjuGator Vision Test)
    │   └── subjugator_pointcloud                                           (SubjuGator PointCloud)
    │       ├── include                                                     (SubjuGator PointCloud Include)
    │       ├── launch                                                      (SubjuGator PointCloud Launch)
    │       └── src                                                         (SubjuGator PointCloud Source)
    ├── scripts                                                             (SubjuGator Scripts)
    │   ├── Dockerfile
    │   └── on_boot
    ├── simulation                                                          (SubjuGator Simulation)
    │   ├── subjugator_gazebo                                               (SubjuGator Gazebo)
    │   │   ├── config                                                      (SubjuGator Gazebo Config)
    │   │   ├── diagnostics                                                 (SubjuGator Gazebo Diagnostics)
    │   │   │   └── gazebo_tests                                            (SubjuGator Gazebo Tests)
    │   │   ├── include                                                     (SubjuGator Gazebo Include)
    │   │   │   └── subjugator_gazebo
    │   │   ├── launch                                                      (SubjuGator Gazebo Launch)
    │   │   ├── models                                                      (SubjuGator Gazebo Models)
    │   │   │   ├── ball                                                    (SubjuGator Gazebo Ball Models)
    │   │   │   ├── bin                                                     (SubjuGator Gazebo Bin Models)
    │   │   │   ├── buoys                                                   (SubjuGator Gazebo Buoy Models)
    │   │   │   ├── dice_2018                                               (SubjuGator Gazebo Dice 2018 Models)
    │   │   │   ├── esp_board                                               (SubjuGator Gazebo ESP Board Model)
    │   │   │   ├── gripper                                                 (SubjuGator Gazebo Gripper Model)
    │   │   │   ├── markers                                                 (SubjuGator Gazebo Markers Models)
    │   │   │   │   └── 2017_and_before                                     (SubjuGator Models before 2017)
    │   │   │   ├── nav_gate                                                (SubjuGator Gazebo Nav Gate Model)
    │   │   │   ├── octogon                                                 (SubjuGator Gazebo Octogon Model)
    │   │   │   ├── posters                                                 (SubjuGator Gazebo Posters Models)
    │   │   │   ├── qual_pole                                               (SubjuGator Gazebo Qual Pole Models)
    │   │   │   ├── roulette_wheel_2018                                     (SubjuGator Gazebo Roulette Wheel 2018)
    │   │   │   ├── sky_box                                                 (SubjuGator Gazebo Sky Box)
    │   │   │   ├── start_gate                                              (SubjuGator Gazebo Start Gate Model)
    │   │   │   ├── start_gate_2022                                         (SubjuGator Gazebo Start Gate 2022 Model)
    │   │   │   ├── sub8                                                    (SubjuGator Gazebo Sub8 Model)
    │   │   │   ├── tentacle                                                (SubjuGator Gazebo Tentacle Model)
    │   │   │   ├── torpedo                                                 (SubjuGator Gazebo Torpedo Model)
    │   │   │   ├── torpedo_board                                           (SubjuGator Gazebo Torpedo Board Model)
    │   │   │   └── transdec                                                (SubjuGator Gazebo Transdec Model)
    │   │   ├── nodes                                                       (SubjuGator Gazebo Nodes)
    │   │   ├── src                                                         (SubjuGator Gazebo Source Code)
    │   │   ├── srv                                                         (SubjuGator Gazebo SRV)
    │   │   ├── subjugator_gazebo_tools                                     (SubjuGator Gazebo Tools)
    │   │   ├── urdf                                                        (SubjuGator Gazebo URDF)
    │   │   └── worlds                                                      (SubjuGator Gazebo Worlds)
    │   └── subjugator_simulation                                           (SubjuGator Simulation)
    │       ├── launch                                                      (SubjuGator Simulation Launch)
    │       ├── nodes                                                       (SubjuGator Simulation Nodes)
    │       │   └── dynamics_simulator                                      
    │       ├── srv                                                         (SubjuGator Simulation SRV)
    │       └── subjugator_sim_tools                                        (SubjuGator Simulation Tools)
    │           ├── meshes                                                  (SubjuGator Simulation Tools Meshes)
    │           ├── physics                                                 (SubjuGator Simulation Tools Physics)
    │           ├── rendering                                               (SubjuGator Simulation Tools Rendering)
    │           ├── shaders                                                 (SubjuGator Simulation Tools Shaders)
    │           └── widgets                                                 (SubjuGator Simulation Tools Widgets)
    ├── sub.rviz
    └── utils                                                               (SubjuGator Utils)
        ├── subjugator_build_tools                                          (SubjuGator Build Tools)
        │   ├── include                                                     (SubjuGator Build Tools Include)
        │   │   └── subjugator_build_tools
        │   └── src                                                         (SubjuGator Build Tools Source)
        │       └── subjugator_build_tools
        ├── subjugator_diagnostics                                          (SubjuGator Diagnostics)
        │   ├── scripts                                                     (SubjuGator Diagnositcs Scripts)
        │   └── subjugator_exception                                        (SubjuGator Diagnositcs Exception Code)
        └── subjugator_msgs                                                 (SubjuGator Messages)
            ├── msg
            └── srv                                                         (SubjuGator Messages SRV)

1033 directories, 3358 files


'''