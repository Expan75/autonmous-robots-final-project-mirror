# Ansible & Infra-as-code

[Ansible](https://www.ansible.com/) is a way to write reproduceable configuration tasks for any host running an ssh server. We’ll use it to ensure that anyone can setup everything that’s needed for every host (just two for now but can work for thousands).

### Inventories

Ansible relies on a list of hosts that expose an ssh server. Ansible runs from a client machine (e.g. a developer), ssh:ing into hosts on the developers behalf (as the developer!). While a bash script could do the same, Ansible includes a bunch of nice things like idempotentency (detects if a task is already complete and skips it) and fault-tolerance (shouts if something goes wrong, optionally retrying). The hosts that can be targeted are listed in _invetory.yml_. These hosts can be grouped to make management more easy, e.g.:

```bash
# pings all servers in the inventory (make sure VPN is on!)
ansible -m ping -i inventory.yml

# build cloud
ansible-playbook bootstrap.yml -i hosts.yml

# build robot (note the trailing ',' is important, even for a single host, and replace kiwi_ip1 with an ip address!)
ansible-playbook bootstrap.yml -i kiwi_ip1,
```

### Playbooks

These are effectively templates for performing a set of tasks, like installing docker or running a set of commands. Ansible enables us to compose playbooks, allow us to have one playbook call others. This is very powerful.

### Getting started

```bash
# install
python3 -m pip install --user ansible-dev-tools

# verify installation was successful with:
ansible --version
```

### Contributing and testing

Unlike containerised workloads, ansible operates on hosts. This makes testing a bit tricky. Luckily, its easy to setup a VM simply for testing via vagrant and automatically pick up whatever ansible playbook you want to test. Note that you will need virtualbox installed system wide.

##### Install virtualbox

```bash
# macos, see https://www.virtualbox.org/wiki/Downloads for other OS:es
brew install --cask virtualbox
```

##### Install Vagrant

Available at [vagrant](https://developer.hashicorp.com/vagrant/install)

```bash
# alternatively install via brew on MacOS
brew tap hashicorp/tapbrew
install hashicorp/tap/hashicorp-vagrant
```

##### Use Vagrant to setup a virtualbox VM and provision via Ansible

```bash
# bring vm up
vagrant up

# once up, go ahead and apply the provisioning (ansible playbook)
vagrant provision
```
