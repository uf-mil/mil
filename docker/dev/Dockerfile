FROM uf-mil:base

# Create a mil-dev user and make them a sudoer
RUN useradd --uid 10000 --create-home --shell /bin/bash mil-dev \
  && echo "" >> /etc/sudoers \
  && echo "mil-dev ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Switch to the new user
USER mil-dev
WORKDIR /home/mil-dev

ADD --chown=mil-dev:mil-dev user_install /tmp/user_install
RUN chmod +x /tmp/user_install && /tmp/user_install
RUN mkdir -p /home/mil-dev/catkin_ws/src/mil
