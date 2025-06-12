# RCLPY Test Package

 [RCLPY Test Package](#rclpy-test-package)
  - [Responsible](#responsible)
  - [Package description](#package-description)
  - [Classes](#classes)
    - [TestSpinner](#testspinner)
    - [TestPublisher and TestSubscriber](#testpublisher-and-testsubscriber)
    - [TestServiceClient and TestServiceServer](#testserviceclient-and-testserviceserver)
  - [Usage](#usage)

## Responsible
This package is in charge of:

    Wilmer David Garzon Caceres 
    wilmer.garzon@kiwibot.com

## Package description

This package contains some utils for unit testing in Python nodes, to make unit test implementation easier. This package is highly inspired by the package [cmr_tests_utils](https://github.com/cmrobotics/cmr_tests_utils) that is used for rclcpp package tests.

## Classes

### TestSpinner

This is the most important class for testing, it allows to spin of several nodes in a different thread, so you can create asserts in your test function without worrying about manual spinning, use it in this way:

```python
import rclpy
import time
from rclpy_tests import TestSpinner
# more imports

def test_example():
    # Initialize ROS context
    rclpy.init() 

    #create spinner
    spinner = TestSpinner()

    # Initialize your nodes ...
    node1 = MyAwesomeNode1()
    node2 = MyAwesomeNode2()

    # Add the nodes to the spinner
    spinner.add_node(node)

    try:
      # spin once in this thread
      spinner.spin_once(timeout_sec)

      # Or create a second thread to spin nodes continuously
      spinner.spin()

      # Add some test logic
      node1.pub()

      # Wait for subscriber
      time.sleep(0.1)
        assert node2.some_method(), "Some message"
    finally:
        # spinner will automatically destroy the second thread :D
        # But you can do it manually with spinner.cancel_spin()
        rclpy.shutdown()

```

Note how the asserts instructions are in a `try` block while `rclpy.shutdown()` is in a `finally` block, this is **really important** to avoid block the test execution when a test fails.

### TestPublisher and TestSubscriber

These classes allow the creation of nodes with a publisher or a subscriber to a single topic, see the example:

```python
import rclpy
from rclpy_tests import TestSpinner, TestSubscriber, TestPublisher
import time

# More imports

def test_ping_pong():
    rclpy.init()

    spinner = TestSpinner()

    pub_node = TestPublisher("/topic_name", MsgType)
    sub_node = TestSubscriber("/topic_name", MsgType)

    # Add nodes to the spinner
    spinner.add_node(pub_node)
    spinner.add_node(sub_node)
    spinner.spin()

    try:
        # Use TestPublisher.publish(msg) for single message publishing
        pub_node.publish(msg) 

        time.sleep(0.1)
        assert sub_node.has_data_been_received() # Confirm if the subscriber has get some message
        msg = sub_node.get_received_msg().data # get Message value from subscriber

        # Use TestPublisher.publish(msg, period, times) for multiple messages publishing
        # period must be > 0 in seconds
        # times is used to indicate how many messages will be published, use time <= 0 for no limit
        pub_node.publish(msg, period, times)
        time.sleep(2.0)

        sub_node.get_frequency() # get message frequency
        sub_node.msg_count # get number of message received
    finally:
        rclpy.shutdown()
```

### TestServiceClient and TestServiceServer

These classes allow the creation of nodes with a service client or a service server to a single service, see the example:

```python
import rclpy
from rclpy_tests import TestSpinner, TestServiceClient, TestServiceServer
import time

# More imports

def test_ping_pong():
    rclpy.init()

    spinner = TestSpinner()

    srv_client = TestServiceClient("/service_name", SrvType)
    srv_server = TestServiceServer("/service_name", SrvType)

    spinner.add_node(srv_client)
    spinner.add_node(srv_server)
    spinner.spin()
    try:
        srv_server.set_response(SrvType.Response) # Set service server response

        srv_client.call(SrvType.Request) # Call service

        time.sleep(0.1)
    
        srv_server.get_last_request() # Get last service request

        srv_client.get_response() # Get response from client side
    finally:
        rclpy.shutdown()
```

## Usage

In your rclpy package.xml, add a new test dependency

```xml
  <test_depend>rclpy_tests</test_depend>
```

Add a test folder in the package tree, and write your test using pytest, remember to add `tests_require=["pytest"]` in the setup.py see an example in this package [setup.py](setup.py).

To run the test, you can use our alias `test-package` or use directly the `colcon tests` instruction.

```bash
test-package my_awesome_package
```