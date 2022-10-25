import rospy
import tf
import tf2_ros
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped


from py_openshowvar import OpenShowVar


class OsvNode:
    def __init__(self, ip: str, port: int) -> None:

        rospy.init_node('osv')
        self.name = rospy.get_name()
        # init osv connection
        # self.client = OpenShowVar(ip=ip, port=port)
        self.tf_br = tf2_ros.TransformBroadcaster()
        # rospy.loginfo(
        #     f'{self.name} node started, KVP CONNECTED {self.client.can_connect}'
        # )

    def read_var(self, var: str) -> str:
        # return self.client.read("$POS_ACT", debug=False)
        return '{E6POS: X 12.84936, Y -394.9386, Z 368.6748, A 17.71607, B 89.99682, C -1.162400, S 18, T 10, E1 90.00000, E2 0.0, E3 0.0, E4 0.0, E5 0.0, E6 0.0}'

    def broadcast_tf_from_var(self):

        while not rospy.is_shutdown():

            var = self.read_var('$POS_ACT')

            var = var.replace(',', '').split(' ')
            pos = list(
                map(float, [var[2], var[4], var[6], var[8], var[10], var[12], var[14]])
            )
            
            quat = tf.transformations.quaternion_from_euler(pos[4], pos[5], pos[6])

            tfs = TransformStamped()

            tfs.header.stamp = rospy.Time.now()
            tfs.header.frame_id = 'base_link'
            tfs.child_frame_id = 'tcp'
            tfs.transform.translation.x = pos[0]
            tfs.transform.translation.y = pos[1]
            tfs.transform.translation.z = pos[2]
            tfs.transform.rotation.x = quat[0]
            tfs.transform.rotation.y = quat[1]
            tfs.transform.rotation.z = quat[2]
            tfs.transform.rotation.w = quat[3]

            self.tf_br.sendTransform(tfs)


if __name__ == '__main__':

    osv = OsvNode("192.168.1.100", 7000)
    osv.broadcast_tf_from_var()
    
