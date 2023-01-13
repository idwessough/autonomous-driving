import pandas as pd
# Beta script emulating autonomous behaviour without real data inputs

# Initialize data to retrieve from ROSDarknet
distance_from_object_center = 0.5  # meters
detected_objects = []
bounding_boxes = []
confidence_score = None

# Load correlation table with all actions to do for each object and distances
actions_df = pd.read_csv("actions_to_do_from_objects.csv")


def distance_from_object_center():
    """Get the distance from the object center.

    Returns:
        _type_: _description_
    """    
    distance_subscriber = rospy.Subscriber("/distance_from_object_center", Float32, callback)
    return distance_from_object_center


def detected_objects():
    """Get the detected objects.

    Returns:
        _type_: _description_
    """    
    detected_objects_subscriber = rospy.Subscriber("/detected_objects", String, callback)
    return detected_objects


def bounding_boxes():
    """Get the bounding boxes.

    Returns:
        _type_: _description_
    """    
    bounding_boxes_subscriber = rospy.Subscriber("/bounding_boxes", String, callback)
    return bounding_boxes

    
def confidence_score():
    """Get the confidence score.

    Returns:
        _type_: _description_
    """    
    confidence_score_subscriber = rospy.Subscriber("/confidence_score", Float32, callback)
    return confidence_score


def callback(data):
    """Callback function for the subscribers.

    Args:
        data (_type_): _description_
    """    
    rospy.loginfo(rospy.get_caller_id() + "I heard %s from ROSDarknet", data.data)


def ROSDarknet_Data():
    """Get the action to do.

    Returns:
        _type_: _description_
    """    
    # Retrieve data from ROSDarknet
    distance_from_object_center = distance_from_object_center()
    detected_objects = detected_objects()
    bounding_boxes = bounding_boxes()
    confidence_score = confidence_score()

    return distance_from_object_center, detected_objects, bounding_boxes, confidence_score


# Sending the action to do to the robot
def send_action_to_robot():
    """Send the action to do to the robot.

    Args:
        actions (list): List of actions to do for the robot (according to the correlation table)
    Returns:
        _type_: _description_
    """    
    information_send = False
    try:
        # Send the action to do to the robot
        actions = actions_df.loc[(actions_df['object'] == detected_object) & (actions_df['distance'] == distance_from_object_center), 'action'].iloc[0]
        # Create a publisher to send the actions to do to the robot in topic "/actions"
        action_publisher = rospy.Publisher("/actions", String, queue_size=10)
        # Publish the action to do
        action_publisher.publish(actions)
        # Logging 
        rospy.loginfo("Action to do: %s", actions)
        # Return the information sendding state to main function
        information_send = True
        return information_send

    except BaseException as sending_error: 
        # Logging and returning the error
        rospy.loginfo("Error sending action to do: %s", sending_error)
        information_send = False
        return information_send
        

if __name__ == '__main__':
    """Main function.
    """    
    # Get the action to do
    distance_from_object_center, detected_objects, bounding_boxes, confidence_score = ROSDarknet_Data()
    for detected_object in detected_objects:
        # if detected_object == "person":
        #     bounding_boxes.append("person")
        # elif detected_object == "poussette":
        #     bounding_boxes.append("poussette")
        # elif detected_object == "bicycle":
        #     bounding_boxes.append("bicycle")
        # elif detected_object == "motorcycle":
        #     bounding_boxes.append("motorcycle")
        
        # Through the correlation table, we can determine the action to do for each object and distance
        object_location = bounding_boxes[detected_objects.index(detected_object)]
        # Send the action to do to the robot
        state = send_action_to_robot() 
        print(f"New actions has been sent to the robot : {state} ")