roboteq
=======

ROS driver for serial Roboteq motor controllers. This driver is suitable for use with Roboteq's
Advanced Digital Motor Controllers, as described in [this document][1]. Devices include:

Brushed DC: HDC24xx, VDC24xx, MDC22xx, LDC22xx, LDC14xx, SDC1130, SDC21xx  
Brushless DC: HBL16xx, VBL16xx, HBL23xx, VBL23xx, LBL13xx, MBL16xx, SBL13xx  
Sepex: VSX18xx

The node works by downloading a MicroBasic script to the driver, which then publishes ASCII sentences at 10Hz and 50Hz with the data corresponding to the Status and per-channel Feedback messages published by the driver.

[1]: http://www.roboteq.com/index.php/docman/motor-controllers-documents-and-files/documentation/user-manual/7-nextgen-controllers-user-manual/file
