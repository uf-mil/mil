FROM uf-mil:base
ARG BRANCH=master
ARG GIT_URL=https://github.com/uf-mil/mil.git

RUN sudo apt-get update

# Create a mil-vrx-trial user and make them a sudoer
RUN useradd --uid 1000 --create-home --shell /bin/bash mil-vrx-trial \
  && echo "" >> /etc/sudoers \
  && echo "mil-vrx-trial ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Switch to the new user
USER mil-vrx-trial
WORKDIR /home/mil-vrx-trial

ADD dynparam /tmp/dynparam
RUN sudo cp /tmp/dynparam /opt/ros/melodic/lib/dynamic_reconfigure/dynparam

ADD --chown=mil-vrx-trial:mil-vrx-trial vrx_trial_install /tmp/vrx_trial_install
RUN chmod +x /tmp/vrx_trial_install && /tmp/vrx_trial_install ${BRANCH} ${GIT_URL}
RUN mkdir -p /home/mil-vrx-trial/catkin_ws/src/mil

ADD --chown=mil-vrx-trial:mil-vrx-trial run_vrx /home/mil-vrx-trial/run_vrx
CMD ["/home/mil-vrx-trial/run_vrx"]
