import numpy as np
import rospy

# BEGIN QUESTION 2.3
from geometry_msgs.msg import PoseStamped
# END QUESTION 2.3


def norm_python(data):
    """Compute the norm for each row of a numpy array using Python for loops.

    >>> data = np.array([[3, 4],
    ...                  [5, 12]])
    >>> norm_python(data)
    array([ 5., 13.])
    """
    n, d = data.shape
    norm = np.zeros(n)
    # BEGIN QUESTION 2.1
    for i in range(n):
        sum_of_squares = 0
        for j in range(d):
            sum_of_squares += data[i, j] ** 2
        norm[i] = sum_of_squares ** 0.5
    # END QUESTION 2.1
    return norm


def norm_numpy(data):
    """Compute the norm for each row of a numpy array using numpy functions.

    >>> data = np.array([[3, 4],
    ...                  [5, 12]])
    >>> norm_numpy(data)
    array([ 5., 13.])
    """
    # You can call np.sqrt, np.sum, np.square, etc.
    # Hint: you may find the `axis` parameter useful.
    # BEGIN QUESTION 2.2
    return np.sqrt(np.sum(np.square(data), axis=1))
    # END QUESTION 2.2


class PoseListener:
    """Collect car poses."""

    def __init__(self, size=100):
        self.size = size
        self.done = False
        self.storage = []  # a list of (x, y) tuples
        # Create a subscriber for the car pose.
        # Hint: once you've figured out the right message type, don't forget to
        # import it at the top! If the message type from `rostopic info` is
        # "X_msgs/Y", the Python import would be "from X_msgs.msg import Y".
        # BEGIN QUESTION 2.3
        self.subscriber = rospy.Subscriber("/car/car_pose", PoseStamped, self.callback)
        # END QUESTION 2.3

    def callback(self, msg):
        """Store the x and y coordinates of the car."""
        header = msg.header
        rospy.loginfo(
            "Received a new message with timestamp " + str(header.stamp.secs) + "(s)"
        )

        # Extract and store the x and y position from the message data
        # BEGIN QUESTION 2.4
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.storage.append((x, y))
        # END QUESTION 2.4
        if len(self.storage) == self.size:
            self.done = True
            rospy.loginfo("Received enough samples, trying to unsubscribe")
            self.subscriber.unregister()
