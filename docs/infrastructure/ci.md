# Continuous Integration

Continuous integration is a helpful software engineering practice completed in
order to ensure code quality over a long period of time. Continuous integration
ensures that multiple tests run on each commit, allowing you to see if your
newly committed code may be causing problems. Code which "fails CI" (or doesn't
pass one of the various tests) will not be merged into the master branch,
because it causes problems.

Currently, we use GitHub Actions as our CI provider. GitHub Actions makes it
easy to integrate CI into our use of GitHub. It provides quick check markers to
show which specific tests failed for a given commit.

## Self-Hosted Runners

Currently, we use self-hosted runners as our provider of CI. On Zobelisk,
we run 12 Docker containers that each serve as a separate runner. Each runner
can handle on job at a time. This means that a lot of CI jobs are automatically
run in parallel. If too many CI jobs are requested at once, a queue will begin
to form.

These runners run in the background, and should not be killed. If the Docker
containers are for some reason killed, you will need to launch the `docker-compose.yaml`
file with `docker-compose up -d`. You will need to get a valid auth token to add
GitHub self-hosted runners from the GitHub Actions self-hosted runner adder GUI.

## Runs, Jobs, Steps

There are multiple pieces to the GitHub Actions CI. The largest piece is the run.
This is the sum of all jobs in the run. Some runs may run more jobs than others.
Currently, the CI job that's spun up on each commit runs around 15 jobs. Inside
each job are multiple steps. Similar to a Dockerfile, each step specifies one
action to do: setup Python, download the repo, and compile a package are all
separate steps. Steps are shared between jobs.

Each run is also associated with a graph showing the relationship between the
jobs in the run. This is helpful to see what jobs other jobs depend on, as well
as why a job might be failing.

## Updating CI

To update the CI process, you will need to change the GitHub Actions file, found
in `~/.github/workflows`. Whenever you change the file and push your changes
to the repo, you can head over to the "Actions" tab of the GitHub repo site
to see how your new changes changed the CI process.
