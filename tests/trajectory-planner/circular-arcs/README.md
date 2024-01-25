# Decoding JSON5 logsa

TP debug log output can be compile-time enabled with the following make arguments:

```
make ... EXTRA_DEBUG='-DTP_DEBUG -DTC_DEBUG -DTP_INFO_LOGGING'
```

These log files use JSON5 format, a superset of JSON. The reason we use this format is that it makes the debug macros easier to write (leave in trailing commas, and don't have to quote field names).

An example of how to parse the log files is shown in parse\_tp\_logs.py. To run this python file, you need python 3 installed, along with some dependencies. A virtual environment is the easiest way to do this:

```
sudo apt install python3 python3-tk python3-pip python3-virtualenv python3-numpy
mkvirtualenv -p `which python3` lcnc_tp_test
```

Once the virtual environment is running, you can install requirements from the config file:
```
pip install -r tp_log_parsing.cfg
```
