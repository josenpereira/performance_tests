# performance_tests

In this package we are interested in measuring
the receival rate of published messages according
to the published rate and to compare it for different
languages (c++, python) and combinations of publishers
and subscribers.

The package is composed of 4 nodes: 2 publishers and 2
subscriber, one publisher and subscriber in c++, another 
pair in python.

For all combination publisher-subscriber the behavior is
the same. The publisher publishes SuperAwesome messages
in the super_awesome_topic at an increasing rate. This 
rate starts at 1 and doubles every 5 seconds. The 
subscriber counts how many messages it as received in
each 5 seconds time interval and uses that information
to estimate the rate of received messages.

To run this behavior there are four launch files
implementing each of the 4 different combinations:

test_cpp2cpp.launch
test_cpp2py.launch
test_py2py.launch
test_py2cpp.launch

After compiling the package (using standard catkin_make approach), run

roslaunch performance_tests test_X2Y.launch

selecting the combination you are interested in. Each 
launch will terminate with an error with numerical
representation limits are reached. This behavior can and
should be improved.

To produce some comparison results we have ran each launch
file 5 time to produce data for each combination. This data was then averaged and used to produced the results
shown in graph_results.png.