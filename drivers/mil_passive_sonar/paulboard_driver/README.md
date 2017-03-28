# Paulboard Driver

The scripts here are used to run the old 'Paulboard', the hydrophone board,
that MIL started using in 2013.

The driver node in this package used to interface with the old 'hydrophone'
package. Communication would occur via custom ROS msg's.

## TODO
If there is a need or want to continue using this board, as opposed to Jake
Easterling's, then the sonar_driver node in mil_passive_sonar should be,
ammended to be able to import the paulboard_driver code.

## Explanation of `permute` parameter

A `permute` value of `w x y z` means that the connector labeled Aw on the PCB
maps to hydrophone 0 as the hydrophones package wants it, Ax maps to 1, Ay
maps to 2, and Az maps to 3.

