# Team 11 Autonomous Robots - project monorepo

This repository contains all source code for the Autonomous Robots group project of Team

Team members:

- Erik Håkansson
- Kris Huang

### Structure & Organisation

```bash
.gitlab-cicd        # main ci/cd specification (pulls child ci/cd specifications from e.g. /robot or /cloud)
./docs              # documentation that is not specific to any one module
./infra             # infrastructure as code (Ansible playbooks for automated deployment)
./boilerplates      # boilerplates for constructing new services
./cloud             # platform services that supports robot, e.g. remote configuration server
./robot             # services for the robot
```

## Getting started

### Repository setup

1. Clone the Repository

```bash
# via ssh (requires you having added your public ssh key to your gitlab profile)
git clone git@git.chalmers.se:erhaka/team11-project-monorepo.git

# via https
git clone git@git.chalmers.se:erhaka/team11-project-monorepo
```

2. Change your directory to the service you wish to run and follow its getting started instructions

```bash
# e.g.
cd robot/MQTTBridgeService

# often it should be sufficient to build and run the accompanying dockerfile
docker compose up -f robot-service-app.yml
```

### Making a new service

To make a new service, it is advised to use one of the boilerplates; you can get up and running with:

```bash
# replace my new service with a good short name describing that the service is meant for.
cp -r boilerplate/cloud-service-go cloud/mynewservice
cp -r boilerplate/robot-service-cpp robot/mynreservice
```

Note, if you try to define a service for a different platform, e.g. a cloud service in the robot/ directory,
CI/CD will not be able to pick up the gitlab-ci definition and run the build steps. See above prepended directory
and make sure its in the right place!

### Contributing & Collaboration

If you are new to using pull requests and git, [trunk-based-git](https://www.atlassian.com/continuous-delivery/continuous-integration/trunk-based-development) is a great guide for the style of git and project deployment we'll use. Every need service or alteration of an existing service starts with you making a feature branch for the given service. You then make add and commit changes to that feature branch e.g.:

```bash
# create a new feature branch. Note its imporant to prepend feature/<mydescription> for the CI/CD to do its job.
git checkout -B feature/cloud-configuration-server-setup

# create a new microservice from the boilerplates
cp -r boilerplate/cloud-service-g cloud/configuration-server-setup
cd cloud-configuration-server-setup

# below portion can be repeated multiple times as your contribution includes more changes or files
echo "hello, world in a textfile" > hello.txt

# add the file (so gut can track changes for it)
git add hello.txt

# running git status should reveal the file being tracked
git status

# commit the changes to the added file (i.e. the text entry)
git commit hello.txt -m "addded a nice text"

# -m is for a message describing what you've done! It is advised to try to be clear
# what was done, e.g. "more-stuff" is worse than "added-namespace-creation". (Don't sweat it too much though!)[https://xkcd.com/1296/]

# let's now try to change the file to see how git works
echo "HELLO, WORLD IN A TEXTFILE BUT CAPS" > hello.txt

# see the change being not committed
git status

# commit the change
git commit hello.txt -m "textfile with more oompf"

# backing up your change (and make it ready for merging)
git push -u origin feature/cloud-configuration-server-setup
```

Once you've pushed your last change and you feel done you can create a new pull request to bring your changes into the main branch. Follow the [guide](https://www.google.com/search?client=firefox-b-d&q=gitlab+pull+request) and once your merge request is reviewd and approved, you changes will automatically released.
