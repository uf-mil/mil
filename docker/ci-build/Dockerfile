FROM uf-mil:base

# Create a mil-ci user and make them a sudoer
RUN useradd --uid 10000 --create-home --shell /bin/bash mil-ci \
  && echo "" >> /etc/sudoers \
  && echo "mil-ci ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# For docs generation
RUN apt install -y python3-pip && pip3 install sphinx recommonmark sphinx_rtd_theme

# Switch to the new user
USER mil-ci
WORKDIR /home/mil-ci
RUN mkdir -p /home/mil-ci/catkin_ws/src/mil
VOLUME /home/mil-ci/catkin_ws/src/mil
