This folder contains data recorded from the current sensor using the rosbag command. This data is published by power_signal_node.py. The rosbag command can be used as follows:

>> rosbag record <topic-names>

To play the file, run:

>> rosbag play <bag-files> 

You can type "rostopic list" to view the topics being published by the bag file. You can subscribe to those topics by using the "rostopic echo" command. You could also use rqt_plot to visualize the data.

For example, to visualize the power signal data in rqt:

>> rosbag play solderIron_laptop.bag
>> rqt_plot /power

About files:
------------
solderingIron_laptop.bag: First a laptop is plugged in (at approximately 10s mark) for a few seconds, and then removed. Then a soldering iron is plugged in (at approximately 30s mark). Finally, both the laptop and soldering iron are plugged in simultaneously (at approximtaly 48s).
