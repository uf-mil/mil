I will be adding full documentation to the wiki when I am more complete but for the moment here is what you need to know

Run goal serverr:

    rosrun sub8_mission_control goal_server

The server is what will eventually be used to command overall goal position and track the progress of the vehicle along its path and for use in missions.
At the moment it just returns True to all sources while I am working on the client

The client is an importable module that can be imported and used to control the boat in any file or from the shell. The server must be running for any client to issue commands. 

The src folder currently has a test file with test cases and examples, there will be more to come