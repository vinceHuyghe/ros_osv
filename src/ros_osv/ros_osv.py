import rospy
import tf
from tf.msg import tfMessage
from geometry_msgs.msg import Transform, TransformStamped, Vector3, Quaternion
from typing import List


from py_openshowvar import OpenShowVar


class OsvNode:
    def __init__(self, ip: str, port: int) -> None:

        rospy.init_node('osv')
        self.name = rospy.get_name()
        # init osv connection
        self.client = OpenShowVar(ip=ip, port=port)
        self.tf_br = tf.TransformBroadcaster()
        rospy.loginfo(
            f'{self.name} node started, KVP CONNECTED {self.client.can_connect}'
        )

    def read_var(self, var: str) -> str:
        # return self.client.read("$POS_ACT", debug=False)
        return '{E6POS: X 12.84936, Y -394.9386, Z 368.6748, A 17.71607, B 89.99682, C -1.162400, S 18, T 10, E1 90.00000, E2 0.0, E3 0.0, E4 0.0, E5 0.0, E6 0.0}'

    def broadcast_tf_from_var(self):

        while not rospy.is_shutdown():

            var = self.read_var('$POS_ACT')

        var = var.replace(',', '').split(' ')
        pos = [var[2], var[4], var[6], var[8], var[10], var[12], var[14]]

        quat = transformations.quaternion_from_euler(
            float(pos[4]), float(pos[5]), float(pos[6])
        )

        tf_msg = Transform(
            Vector3(x=pos[0], y=pos[1], z=pos[2]),
            Quaternion(
                x=quat[1],
            ),
        )


if __name__ == '__main__':

    osv = OsvNode("192.168.1.100", 7000)
    print(osv.read_var("$POS_ACT"))
