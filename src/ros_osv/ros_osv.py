import rospy
import tf
from tf.msg import tfMessage


from py_openshowvar import OpenShowVar()


class OsvNode():
    
    def __init__(self,ip: str, port: int) -> None:
        rospy.init_node('osv')
        self.name = rospy.get_name()
        # init connection
        self.client=OpenShowVar(ip,port)
        self.tf_br = tf.TransformBroadcaster()
        rospy.loginfo(f'{self.name} node started, KVP CONNECTED {client.can_connect}')
        self.tfmsg = tfMessage()
                
    def read_var(self, var: str)->str:
        self.client.read(var)
        pass
    
    def resp_to_tf(self, resp: str) -> tfMessage:
        
        self.tfmsg = resp
        
        return self.tfmsg
        
        
            
    def run(self):      
        while not rospy.is_shutdown:
            self.tf_br.sendTransform(self.resp_to_tf(self.read_var('tcp_pose')))
            

if __name__ == '__main__':
    
    osv = OsvNode('192.168.19.132',7001)
    osv.run()
  