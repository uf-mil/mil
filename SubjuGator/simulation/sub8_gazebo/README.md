# Gazeb

# Current Maintainer/servitor: Nicholas Suhlman, Grymestone@ufl.edu

A brief tutorial on how to gazeb.

> If you wish to simulate the sub from scratch, you must first invent the universe. <br>
> &nbsp;&nbsp;&nbsp;-- Carl Sagan

### To run the sim
To launch with the gui:
`roslaunch sub8_gazebo duck.launch`

To launch without the gui: `roslaunch sub8_gazebo duck.launch gui:=false`

To launch without the cameras: `roslaunch sub8_gazebo duck.launch cameras:=false`

And these can be combined if you'd like.

**NOTE:** You can run the sub with or without a gui. Disabling one or both of these is intended for use when testing controllers or other things that only rely on the data and not the actual visuals. Also it is intended for people who have shitty computers that would benefit from not having to run the gazebo window and the cameras (which saves a nontrival amount of CPU time).

### To add custom textures to models

You will need to install Blender from the website, not using sudo apt-get install, as this package manager downloads an older version of Blender that does not come with the DAE Export option which we need also known as Collada. 

This guide assumes you have created a mesh and texture using SolidWorks. All the models I have been given from Solidworks have already come with meshes so I did not have to create them. The files should be formatted as STL from SolidWorks.  Look up guides on how to bind textures to UV Maps in Blender. UV maps can be changed by changing your mode to Edit Mode, which should display the map. By pressing U you should be able to "Smart Unwrap" or "Unwrap" the mesh. This allows for easy texture painting later. 

Once this is done, change your display to UV Editing mode which should open a second window displaying your UV Mesh that is completely unwrapped and separated. If you are still in Edit Mode you should see lines which should be recognizable as the UV Mesh of the model. Now you can start painting! Add a new material and begin editing. There are plenty of tutorials for this online and I would look a few up if you get stuck at this stage. Blender is admittedly a complicated program that is a pain to use, but once you figure it out the process becomes fairly streamlined and easy.

That is what you will be doing for the most part to create your texture files. You'll want to save the texture file as a separate png for this. An interesting side note about Blender, the way the UV Maps work and meshes work, you cannot scale an object differently in each dimension and preserve the mesh. The entire object must be scaled at the same ratio (.5,.5,.5 for example). Otherwise when you export the object you will encounter many terrible errors trying to load it into Gazebo.

tldr; Scale your objects evenly in blender or else the machine spirits will be displeased.

1. Open DAE file and move to library_images tag. If one does not exist, create it.
2. Add image id and import image within this tag. Example found in colored_lid.dae. Close library_images tag.
3. Immediately after, go to the library_effects tag or create one directly beneath library_images.
4. Create Material Effect and Material Effect 2.
5. These will allow you to bind the image to a material as its texture. Examples found in colored_lid.dae.
6. Close library_effects tag and open library_materials tage if one does not yet exist.
7. Follow format set in colored_lid.dae.
8. Within the mesh settings under the library_geometry tag, search for materials and ensure they carry the same name as your material.
9. In colored_lid, the material is simply called 'Material'.
10. Finally under the library_visual_scenes search for the instance_geometry tag that creates your mesh.
11. Here you shall place bind_material and bind your materials to the mesh, applying your texture to the object.
12. Copy the format as shown in colored_lid.dae.
13. Once this is all completed, you now need to create a SDF file. The SDF file is the file that Gazebo will actually be loading and it directly references your new DAE file. Creating an SDF file mostly involves copying over an existing SDF, and changing the name and important attributes such as mass and inertia to match the object you are importing. 
14. You will need to change the model name, link name, and pretty much every other name in the SDF file you copied to reflect the new model. Take special note of the model and link name, as those will be used in the duck.launch file to identify the object. Also ensure the url path to the DAE file is correct. 
15. With the SDF files completed, you can open up the duck.launch file and add the model to Gazebo. Follow the format set up by other models being loaded in such as the buoys or dice. 
16. Name and model in the launch file should both be the same and be equivalent to the model name and link name you set up in the SDF file. Take note of the coordinates you are placing the models at and make sure to adjust as needed. 
17. If all goes well you should see your model loaded in next time you boot Gazebo!

**NOTE:** I have not tested trying to apply more than one texture. To create the texture, I used Blender. This created a texture wrapped around the mesh. I exported the texture file as a png as you can see in the bin folder of models. The image you use should be in the same directory as the dae file. I have not tested putting it elsewhere. Additionally, if you see yellow and black lines on your object as opposed to your  texture, it means Gazebo/dae file could not find you texture! Check to make sure the name of the image matches the name of the file being imported. If you see a standard grey object, this means you did not import or code the  texture properly. Recheck the steps and compare to either the skybox or the colored_lid dae files.  

### Teleport Pose Coordinates for Challenges
To use command: subc tp STRING
  To input a saved location, merely type the name of the location where you wish to teleport. Shorthand names located in the teleport_locs.yaml file in  sub8_gazebo/configs. Ex:
  subc tp buoy

  To input raw coords input in quotes the X,Y,Z coords you wish to teleport to as detailed below:
  subc tp "X Y Z"

Raw coords if desired:
1. Bin Assembly - -24 14 -3.5
2. Start Gate - 16 13 -1.5  (Will need to rotate sub to actually see it)
3. Nav Gate - 26 -13 -1.5 (Will need to rotate sub to actually see it)
4. Buoy - 19 20 -1
5. Octogon - floats on surface above 20 -20 -1
6. Squilliamson & his single tentacle- 21 11 -2.75
