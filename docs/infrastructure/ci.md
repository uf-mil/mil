# Continuous Integration

Continuous integration is a helpful software engineering practice completed in
order to ensure code quality over a long period of time. Continuous integration
ensures that multiple tests run on each commit, allowing you to see if your
newly committed code may be causing problems. Code which "fails CI" (or doesn't
pass one of the various tests) will not be merged into the master branch,
because it causes problems.

Currently, we use GitHub Actions as our CI provider. It provides quick check markers to
show which specific tests failed for a given commit.

## Architecture and Terminology
:::{graphviz} ci_architecture.dot
:::

### Runner Environment

Currently, our continuous integration is run in a Docker container on one of our
computers, Shuttle. This container is given unlimited resources to the host
machine in order to help jobs complete quickly.

The Docker Compose file for this container is stored locally on Shuttle. To start
up the Docker container, you will need to run `docker-compose up -d` inside
`~/runners` (where the compose file is stored).

### Runs, Jobs, Steps

There are multiple pieces to the GitHub Actions CI. The largest piece is the run.
This is the sum of all jobs in the run. Some runs may run more jobs than others.
Currently, the CI job that's spun up on each commit runs around 15 jobs. Inside
each job are multiple steps. Similar to a Dockerfile, each step specifies one
action to do: setup Python, download the repo, and compile a package are all
separate steps. Steps are shared between jobs.

Each run is also associated with a graph showing the relationship between the
jobs in the run. This is helpful to see what jobs other jobs depend on, as well
as why a job might be failing.

## Using CI

Continuous integration aims to be functional and automatic, and fulfills this mission
by automatically running when you push a new commit to any branch. However,
sometimes you may want more control over the CI process.

To view the output of a job more closely, click on the "Details" tab next to
the status check of an action. You will then see each step of the job. You can
view the logs of these steps to see what exactly the job executed.

If you want to restart a job, click on the "Re-run all jobs" button. This will allow
you to re-run the jobs. If the job is currently running (or stuck), you will likely
need to cancel it before you will be allowed to re-run it.

## Updating CI

To update the CI process, you will need to change the GitHub Actions file, found
in `~/.github/workflows`. Whenever you change the file and push your changes
to the repo, you can head over to the "Actions" tab of the GitHub repo site
to see how your new changes changed the CI process.
