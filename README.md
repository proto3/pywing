# Pywing

Pywing is a all-in-one software for hot wire foam CNC cutting.

## How to use it ?

First of all, clone the repository and open a terminal inside pywing folder.

Install and create a virtualenv (you can skip this and install dependencies directly on your system) :
```shell
# install virtualenv (use your dist packages manager if not apt)
sudo apt-get install virtualenv

# create a python3 virtualenv
virtualenv -p /usr/bin/python3 env

# enter your virtualenv
source env/bin/activate #
```

Install pywing dependencies and run :
```shell
# install pywing dependencies
pip install pyqt5 pyqtgraph

#run pywing
python pywing.py
```
