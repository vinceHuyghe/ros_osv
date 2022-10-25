import rospy
import tf
from tf.msg import tfMessage


from py_openshowvar import OpenShowVar


class OsvNode():

    def __init__(self, ip: str, port: int) -> None:
        rospy.init_node('osv')
        self.name = rospy.get_name()
        # init connection
        self.client = OpenShowVar(ip=ip, port=port)
        # self.tf_br = tf.TransformBroadcaster()
        rospy.loginfo(
            f'{self.name} node started, KVP CONNECTED {self.client.can_connect}')
        # self.tfmsg = tfMessage()
        print(self.client.can_connect)

    def read_var(self, var: str) -> str:
        return self.client.read(var)

    # def resp_to_tf(self, resp: str) -> tfMessage:

    #     self.tfmsg = resp

    #     return self.tfmsg

    # def run(self):
    #     while not rospy.is_shutdown:
    #         self.tf_br.sendTransform(
    #             self.resp_to_tf(self.read_var('$POS_ACT')))


if __name__ == '__main__':

    osv = OsvNode('192.168.1.100', 7001)
    print(osv.read_var('$POS_ACT'))
