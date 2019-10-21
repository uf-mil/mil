# VRX
## Development WorkFlow
The recommended development workflow is:

1. Run the dev continer:
	`./scripts/run_development_container`
2. Branch from the repo and edit whatever files you want to change with what ever text editor you want. Any changes you make to the files from your host machine will immidiately show up in the container and when you re-run(for python changes)/ compile(for cpp changes) these changes will immidiately take effect. Do the bulk of your development in this stage running mil and vrx code in the same container for fast turn around time in testing.

3. Once you are satified with running your code in the development container, you can move on to running your changes against the VRX server. 
	
	This is how we will be evaluated by OSRF.

4. Make a Trial Container:

	Commit and push your changes to a git hub repository on a branch. ie: `my_branch` at `https://github.com/ME/mil.git`

	Navigate to the root of the repo.

	# If you pushed to a fork of the uf-mil github:

	run `./docker/vrx_trial/build_vrx_trial_container my_branch https://github.com/ME/mil.git`
	
	# If you pushed to the uf-mil github:	
	
	run `./docker/vrx_trial/build_vrx_trial_container my_branch`

	*NOTE: Make sure when building your trial container, it the code actually compiles.*
	
5. Run your container with a terminal for sanity check:
	
	Make sure your container actually does what you want when it starts up.
	
	run `./docker/vrx_trial/run_vrx_trial_container`	

6. Now you should have a trial container!

	Follow the instructions at `https://bitbucket.org/osrf/vrx-docker/src/default/` to run this container against the vrx server. 