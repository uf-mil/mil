The Continuous Integration (CI) service for MIL projects is a self-hosted [Jenkins](https://jenkins.io/index.html) server with a [Docker](https://www.docker.com) backend. It is located [here](https://ci.mil.ufl.edu/jenkins/blue/pipelines) and MIL credentials can be used to log in to it.

MIL had been using [Semaphore](https://semaphoreci.com) before the upgrade to Ubuntu 16.04 and ROS Kinetic. Due to this new software stack not being supported on that platform, and after considering other available services, it was decided to self-host the CI infrastructure for increased performance and configurability. This page is meant to provide an overview of the way that the service was configured and why certain design decisions were made. If you have any questions that are not answered on this page, feel free to contact [Anthony Olive](https://github.com/whispercoros) with any further questions.


# Jenkins
Jenkins is a task automation server primarily focused on continuous integration. It allows a build pipeline to be defined for each project that we work on so that a series of checks can be performed on changes to that project to ensure that those changes meet our requirements. The [Blue Ocean](https://jenkins.io/projects/blueocean) suite of plugins have been installed to improve the look and functionality of the server.

#### Design Decision
Jenkins has been in development for a long time. It is very stable and has an active community working on it. Using plugins developed by the community, it was simple to configure our instance based on our needs.

#### Pipeline as Code
The pipeline for Jenkins builds of MIL repositories is defined in a file that sits in the root directory of the repository. This file must be named `Jenkinsfile`. See [this page](https://jenkins.io/solutions/pipeline) for more information and the [pipeline file](https://github.com/uf-mil/mil_common/blob/master/Jenkinsfile) for the mil_common repository for an example. Note that the entire pipeline is wrapped in a `dockerNode(image: 'uf-mil:mil_common')` block. This provisions a Docker container for the build based on the mil_common image. Each stage of the build process is run in a new shell, so the MIL runcom must be sourced for each of them (the output of this is piped to `/dev/null` to reduce verbosity). At the time of writing, there are four stages in the build process for projects:

* `Checkout` - pull the branch or pull request that is to be built and any other repositories needed for the build
* `Build` - run `catkin_make` on the workspace to ensure that the project builds
* `Test` - run all ROS tests in the workspace and output the results to ensure that they all pass
* `Format` - run `clang_format` and `flake8` to ensure that all code meets our style guide requirements

#### Github
Using the [Github Organization Folder plugin](https://wiki.jenkins-ci.org/display/JENKINS/GitHub+Organization+Folder+Plugin), it is trivial to interact with many of the features Github offers for CI. The plugin is configured to scan the organization at a regular interval in order to index new repositories and uses an organization hook to initiate builds whenever a pull request or branch is updated. The Jenkins server has a personal authentication token for for the mil-bot Github account, which is used for the sole purpose of updating the build status of commit messages.

#### Authentication
The [Active Directory plugin](https://wiki.jenkins-ci.org/display/JENKINS/Active+Directory+Plugin) allows the server to authenticate users with their MIL credentials (the same credentials used to access the fileserver). These need to be obtained by talking to the lab's network administrator (currently [Daniel Dugger](https://github.com/duggerd)). This is coupled with the [Matrix Authorization Strategy plugin](https://wiki.jenkins-ci.org/display/JENKINS/Matrix+Authorization+Strategy+Plugin), which allows permissions to be given out selectively to Active Directory groups. At the time of writing, there is a `CI Admin` group as well as one for each project. Normal users will be able to perform basic tasks, such as canceling and restarting builds.

#### Apache Reverse Proxy
The Jenkins instance sits behind an Apache reverse proxy under the `/jenkins` path. This enables us to run other web services on the domain as well as to secure the service with TLS. See [this guide](https://wiki.jenkins-ci.org/display/JENKINS/Running+Jenkins+behind+Apache) for more information on how this was done. Keep in mind that the `AllowEncodedSlashes NoDecode` must be set for the virtual hosts on port 80 and port 443 as well (this took us far too long to figure out).


# Docker
Docker is the platform that is used to provision hosts for each build. It enables the creation of an image based on a build file that can contain environment variables, commands to run, external volume configurations, and more. The filesystem for this image is built in layers of differences using aufs. For more information on how Docker works and how to use it, please refer to the [documentation](https://docs.docker.com/get-started).

#### Design Decision
Docker was chosen for a few different reasons. Most importantly, it allows us to spin up ephemeral containers. This means that a container can be quickly created based on the image for a project and then that container can be torn down once the build process for a pull request or branch is complete. For each container, a new filesystem layer is created on top of the image to temporarily store changes, such as compiled code. After the build, that layer is simply discarded. This system also enables us to spin up multiple containers at once on top of the same image, meaning we are not limited to just one build at a time (this would be the case with a free subscription to an online CI service). Being run in a container, the images will also have access to higher performance hardware and even the server's Nvidia GTX 980 GPU. Last but not least, since all of the dependencies are fetched when an image is created, builds will be much faster.

#### Implementation in Jenkins
The [Docker Slaves plugin](https://wiki.jenkins-ci.org/display/JENKINS/Docker+Slaves+Plugin) is used to connect Jenkins to Docker. This plugin requires that the `jenkinsci/slave:latest` image is present for running commands inside the Docker slave and that the `buildpack-deps:scm` image is present for checking out the required Git commit. 

#### Image Build Files
See the [example](https://github.com/uf-mil/mil_common/blob/master/scripts/Dockerfile) in this repository for information on how build files should be formatted. The file is well commented and should be fairly self explanatory so long as you understand Docker. The location that the build file for a project should be stored is `scripts/Dockerfile` relative to the repository's root directory.