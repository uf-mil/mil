#**PyODE**

**PyODE is a set of open-source Python bindings for The Open Dynamics Engine, an open-source physics engine. PyODE also includes an XODE parser.**

**It is used for the navigator simulation and is necessary to run any simulation with graphics.**

PyODE seems to be broken, so we must compile from source. Thanks to Forrest for this discovery.

    rm -fr /tmp/pyode-build && mkdir -p /tmp/pyode-build && cd /tmp/pyode-build && sudo apt-get build-dep -y python-pyode && sudo apt-get remove -y python-pyode && apt-get source --compile python-pyode && sudo dpkg -i python-pyode_*.deb
