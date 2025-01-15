import rospy
import tf
from tf import TransformListener
import numpy as np
from scipy.spatial.transform import Rotation as R

def get_end_effector_transform():
    rospy.init_node("frame_transformer", anonymous=True)
    listener = TransformListener()
    try:
        listener.waitForTransform('/base_link', '/tool0', rospy.Time(0), rospy.Duration(5.0))
        (trans, rot) = listener.lookupTransform('/base_link','/tool0', rospy.Time(0))
        return np.array(trans), np.array(rot)
    
    except tf.Exception as e:
        rospy.logerr(e)
        return
    
def transform_to_local_frame(global_vector, translation, rotation_quat):
    rotation = R.from_quat(rotation_quat)
    rotation_matrix=rotation.as_dcm()
    local_vector = np.dot(rotation_matrix.T, global_vector)
    return local_vector

if __name__ == "__main__":
    global_direction = np.array([1,0,0])
    translation, rotation_quat = get_end_effector_transform()
    if translation is not None and rotation_quat is not None:
        local_direction = transform_to_local_frame(global_direction, translation, rotation_quat)
        print(global_direction)
        print(local_direction)
    else:
        print("Failed to retrieve")
    
