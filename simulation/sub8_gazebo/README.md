# Gazeb

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
1. Open DAE file and move to library_images tag. If one does not exist, create it.
2. Add image id and import image within this tag. Example found in colored_lid.dae. Close library_images tage.
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

**NOTE:** I have not tested trying to apply more than one texture. To create the texture, I used Blender. This created a texture wrapped around the mesh. I exported the texture file as a png as you can see in the bin folder of models. The image you use should be in the same directory as the dae file. I have not tested putting it elsewhere. Additionally, if you see yellow and black lines on your object as opposed to your  texture, it means Gazebo/dae file could not find you texture! Check to make sure the name of the image matches the name of the file being imported. If you see a standard grey object, this means you did not import or code the  texture properly. Recheck the steps and compare to either the skybox or the colored_lid dae files.  
