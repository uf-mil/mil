# Services

**Components of a ROS Service**
Some ROS classes distinguishes themselves by employing a service-oriented model rather than the usual topic-based approach. In ROS, Services act as the conduits for interaction between nodes, functioning in a request-response manner. While ROS topics enable asynchronous data exchange, services facilitate nodes in seeking specific actions or information from other nodes, awaiting a subsequent response before proceeding. This method of waiting before proceeding is known as a synchronous data exchange. This proves especially valuable in tasks that require direct engagement, such as data retrieval or computations.

A ROS service structure is bifurcated into two components:

1. **Server**: This node provides the service and waits for incoming requests from other nodes. In simpler terms, it carries out the operation you have specified.

2. **Client**: This node sends requests to the service server and waits for the response.

Here is the official ROS documentation on services: [https://wiki.ros.org/Services](https://wiki.ros.org/Services)

For a concrete illustration, consider the case of a service designed to query and update an array of integers:

1. **Define the Service Messages**

Create two separate service message files in your ROS package's `srv` folder:

- `ArrayUpdate.srv`:

```
# ArrayUpdate.srv
int32 index
int32 value
---
bool success
```

- `ArrayQuery.srv`:
```
# ArrayQuery.srv
int32 index
---
int32 value
```

2. **Implement the Service Server**

Create a Python script named `array_service_server.py`:

```python
#!/usr/bin/env python

import rospy
from your_package_name.srv import ArrayUpdate, ArrayQuery
from std_msgs.msg import Int32MultiArray

class ArrayServiceServer:
    def __init__(self):
        self.array_data = [10, 20, 30, 40, 50]

    def handle_array_update(self, request):
        if 0 <= request.index < len(self.array_data):
            self.array_data[request.index] = request.value
            return ArrayUpdateResponse(True)
        else:
            return ArrayUpdateResponse(False)

    def handle_array_query(self, request):
        if 0 <= request.index < len(self.array_data):
            return ArrayQueryResponse(self.array_data[request.index])
        else:
            return ArrayQueryResponse(-1)  # Invalid index

def main():
    rospy.init_node('array_service_server')
    server = ArrayServiceServer()

    rospy.Service('array_update', ArrayUpdate, server.handle_array_update)
    rospy.Service('array_query', ArrayQuery, server.handle_array_query)

    rospy.spin()

if __name__ == '__main__':
    main()
```

Replace `your_package_name` with the actual name of your ROS package.

3. **Implement the Service Clients**

Create two separate Python scripts for interacting with the service:

- `array_update_client.py`:
```python
#!/usr/bin/env python

import rospy
from your_package_name.srv import ArrayUpdate

def array_update_client(index, value):
    rospy.wait_for_service('array_update')
    try:
        array_update = rospy.ServiceProxy('array_update', ArrayUpdate)
        response = array_update(index, value)
        return response.success
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == '__main__':
    rospy.init_node('array_update_client')

    index_to_update = 2
    new_value = 35
    success = array_update_client(index_to_update, new_value)

    if success:
        print(f"Value at index {index_to_update} was updated to {new_value}")
    else:
        print(f"Failed to update value at index {index_to_update}")
```

- `array_query_client.py`:
```python
#!/usr/bin/env python

import rospy
from your_package_name.srv import ArrayQuery

def array_query_client(index):
    rospy.wait_for_service('array_query')
    try:
        array_query = rospy.ServiceProxy('array_query', ArrayQuery)
        response = array_query(index)
        return response.value
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == '__main__':
    rospy.init_node('array_query_client')

    index_to_query = 2
    value = array_query_client(index_to_query)

    if value != -1:
        print(f"Value at index {index_to_query}: {value}")
    else:
        print(f"Invalid index {index_to_query}")
```

Replace `your_package_name` with your actual ROS package name.
