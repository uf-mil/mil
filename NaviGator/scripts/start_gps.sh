#!/bin/bash

# Create systemd service
service_file='/etc/systemd/system/sdgps.service'

# Remove existing service file if it exists
if [ -f $service_file ]; then
    rm $service_file
fi

# Write the content of the service file without outputting to terminal
echo '[Unit]' > $service_file
echo 'Description=Start GPS Service for NaviGator' >> $service_file
echo 'After=network.target' >> $service_file
echo '' >> $service_file
echo '[Service]' >> $service_file
echo 'ExecStart=NaviGator/mission_control/navigator_launch/launch/hardware/gps.launch' >> $service_file
echo 'User=navigator' >> $service_file
echo 'Restart=always' >> $service_file
echo '' >> $service_file
echo '[Install]' >> $service_file
echo 'WantedBy=multi-user.target' >> $service_file

# Reload systemd to recognize the new service
systemctl daemon-reload
systemctl enable sdgps.service
systemctl start sdgps.service
