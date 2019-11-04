The software team has changed naming conventions for topic names and frame ids several times. 
This means that when exploring older subjugator bag files for testing and development, 
you may stumble upon files with different topics or frames for various sensors (particularly cameras) 
than those used today. For this reason, we wrote a generic utility,```fix_bag.py```, for going through a bag and renaming topics and frame_ids in headers.

### Usage
```
rosrun sub8_ros_tools fix_bag.py
```
``` 
usage: fix_bag.py [-h] --in INFILE [--out OUTFILE]
                  [--topics oldtopic:newtopic [oldtopic:newtopic ...]]
                  [--frames oldframe:newframe [oldframe:newframe ...]]

Fix bag topics/frame_ids

optional arguments:
  -h, --help            show this help message and exit
  --in INFILE, -i INFILE
                        Bag to read and fix
  --out OUTFILE, -o OUTFILE
                        Bag to output with fixed content, defaults to
                        INPUTFILENAME_fixed.bag
  --topics oldtopic:newtopic [oldtopic:newtopic ...]
                        list of topics to remap, seperated by a colon, ex
                        /down_cam/:/camera/down/ /my_odom:/odom If ends in a
                        slash (ex: /cameras/:/cams/, all topics after slash
                        will be remaped
  --frames oldframe:newframe [oldframe:newframe ...]
                        list of frame_ids in headers to be maped, ex:
                        my_camera:cam_front


```

## Useful remaps
The following flags to ```--topics``` and ```--frames``` will handle most sensor name convention changes in bags
```
--topics /down/:/camera/down/left/ /down/left/:/camera/down/left/ /down_camera/:/camera/down/left/ /stereo/right/:/camera/front/right/ /stereo/left/:/camera/front/left/ /forward_camera/:/camera/front/
```
```
--frames downward:down_left_cam down_camera:down_left_cam stereo_front:front_stereo /stereo/right_cam:front_right_cam /stereo/left_cam:front_left_cam forward_camera:front_stereo
```

### Contribute
If you find more topics/frames that have been renamed, please update this wiki.