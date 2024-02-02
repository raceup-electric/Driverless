from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


# create reader instance and open for reading
with Reader('C:/Users/jkaed/Downloads/') as reader:
    # topic and msgtype information is available on .connections list
    for connection in reader.connections:
        print(connection.topic, connection.msgtype)

    # iterate over messages
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/velodyne_points':
            msg = deserialize_cdr(rawdata, connection.msgtype)
            #print(msg.header.frame_id)

    # messages() accepts connection filters
    connections = [x for x in reader.connections if x.topic == '/velodyne_points']
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = deserialize_cdr(rawdata, connection.msgtype)
        print(msg.header.frame_id)