import csv
import rclpy
import sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

def extract_data_to_csv(bag_file, topic_name, csv_file):
    # Initialize ROS client library
    rclpy.init()

    # Create a reader to read the bag file
    storage_options = rosbag2_py.StorageOptions(uri=bag_file, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Get the topic type
    topic_type = None
    for topic_metadata in reader.get_all_topics_and_types():
        if topic_metadata.name == topic_name:
            topic_type = topic_metadata.type
            break

    if topic_type is None:
        print(f"Topic {topic_name} not found in the bag file.")
        return

    # Get the message class for the topic type
    msg_class = get_message(topic_type)

    # Handle the /encoder_data topic
    if topic_name == "/encoder_data":
        # Open the CSV file for writing
        with open(csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['timestamp', 'data'])

            # Iterate through the messages in the bag
            while reader.has_next():
                (topic, data, timestamp) = reader.read_next()
                if topic == topic_name:
                    # Deserialize the message
                    msg = deserialize_message(data, msg_class)

                    # Write the timestamp and data to the CSV file
                    writer.writerow([timestamp, msg.data])

    # Handle the /cmd_vel topic
    elif topic_name == "/cmd_vel":
        # Open the CSV file for writing
        with open(csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['timestamp', 'linear_x', 'linear_y', 'linear_z', 'angular_x', 'angular_y', 'angular_z'])

            # Iterate through the messages in the bag
            while reader.has_next():
                (topic, data, timestamp) = reader.read_next()
                if topic == topic_name:
                    # Deserialize the message
                    msg = deserialize_message(data, msg_class)

                    # Write the timestamp and cmd_vel data to the CSV file
                    writer.writerow([
                        timestamp,
                        msg.linear.x,
                        msg.linear.y,
                        msg.linear.z,
                        msg.angular.x,
                        msg.angular.y,
                        msg.angular.z
                    ])

    # Handle the /joy topic
    elif topic_name == "/joy":
        # Open the CSV file for writing
        with open(csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['timestamp', 'frame_id', 'axes', 'buttons'])

            # Iterate through the messages in the bag
            while reader.has_next():
                (topic, data, timestamp) = reader.read_next()
                if topic == topic_name:
                    # Deserialize the message
                    msg = deserialize_message(data, msg_class)

                    # Flatten the axes and buttons lists
                    axes_str = ','.join([str(a) for a in msg.axes])
                    buttons_str = ','.join([str(b) for b in msg.buttons])

                    # Write the timestamp, frame_id, axes, and buttons data to the CSV file
                    writer.writerow([
                        timestamp,
                        msg.header.frame_id,
                        axes_str,
                        buttons_str
                    ])

    print(f"Data extracted to {csv_file}")

    # Shutdown the ROS client library
    rclpy.shutdown()

if __name__ == "__main__":
    bag_file = sys.argv[1]  # Path to your ROS2 bag file
    topic_name = sys.argv[2]  # Topic to extract
    csv_file = sys.argv[3]  # Output CSV file

    extract_data_to_csv(bag_file, topic_name, csv_file)
