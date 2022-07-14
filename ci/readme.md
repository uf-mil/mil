# Continuous Integration

:::{warning}
These docs need to be updated to show the interactions of CI with GitHub Actions,
the CI process that we are moving towards.
:::

MIL uses [BuildKite](https://buildkite.com/uf-mil) as our continuous integration system. This system
automatically tests pull requests to ensure they build, pass unit/integration tests, and satisfy formatting requriements.

##  Updating/Running the BuildKite agent
While the web frontend for the CI is hosted for free by the good folks at BuildKite, the actual
builds run on MIL's servers. The server that receives new requests from BuildKite and runs the build
is called the "agent". Our agent runs inside of a docker container `uf-mil:ci-server` on the MIL ci server.

If the agent needs to be run on a new server or updated, perform the following steps *from the server*:

* Install docker on the host machine (the remainder of the instructions assume the host machine is running a modern Ubuntu distribution)
* Kill the current instance if it is running ```sudo docker kill buildkite-agent```
* Change directory to a copy of the latest master branch of this repository. ```cd <repo>```
* Build/rebuild the docker container ```./scripts/build_docker_containers```. This may take a while.
* Grab the buildkite agent token from [this link](https://buildkite.com/organizations/uf-mil/agents). You will need access to the uf-mil buildkite organization, which Kevin can give you.
* Run the agent ```sudo BUILDKITE_AGENT_TOKEN=<token from step above> ./ci/start_ci_server```. You should see some nice info messages as buildkite starts.
* Verify the agent is running [here](https://buildkite.com/organizations/uf-mil/agents)

Note: this process should be automated on the server to happen on boot, using systemd or something.

## What happens in a build
When a new build is triggered from commits to master or a pull request, the following happens:
* The agent receives the commit to checkout to from buildkite
* The agent reads from ```.buildkite/pipeline.yml``` to get the current build configuration
* The agent builds the docker containers in this repo, tagging them with the commit being built
* The agent starts an instance of the `uf-mil:dev` container built from this commit and copies the git repo into in
* The container runs ```ci/build```, streaming the output to the web interface and failing the build if any command in that script returns a non-zero exit code

## Testing CI process locally
To see if your code pass will CI
`CLEAN_BUILD=true ./scripts/run_development_container /home/mil-dev/catkin_ws/src/mil/ci/run_ci`
