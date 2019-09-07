# Continuous Integration
MIL uses [BuildKite](https://buildkite.com/uf-mil) as our continuous integration system. This sytem
automaticly tests pull requests to ensure they build, pass unit/integration tests, and satisfy formating requriements.

##  Updating/Running the BuildKite agent
While the web frontend for the CI is hosted for free by the good folks at BuildKite, the actual
builds run on MIL's servers. The server that receives new requests from BuildKite and runs the build
is called the "agent". Our agent runs inside of a docker container `uf-mil:ci-server` on the MIL ci server.

If the agent needs to be run on a new server or updated, perform the following steps *from the server*:

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
* The agent starts an instance of the `uf-mil:ci-build` container built from this commit and copies the git repo into in
* The container runs ```ci/build```, streaming the output to the web interface and failing the build if any command in that script returns a non-zero exit code

## TODOs
* Use the [docker-compose-buildkite-plugin](https://github.com/buildkite-plugins/docker-compose-buildkite-plugin) to build the `uf-mil:ci-build` container for each new build so changes to the Dockerfile/install scripts are tested by ci
* Investigate if build containers/data are properly being deleted after they run, so the server hard drive does not fill up
