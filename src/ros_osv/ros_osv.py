import rospy

from py_openshowvar import OpenShowVar()


class OsvNode():
    
    def __init__(self,ip: str, port: int) -> None:
        name = 'osv'
        rospy.init_node(name)
        # init connection
        client=OpenShowVar(ip,port)
        rospy.loginfo(f'{name} node started, CONNECTED {client.can_connect}')
        
        
        # tf publisher
          
        pass
    
    def publish_tf_from_var():
        pass
        
    def read_var():
        pass
    
    def main():
        osv = OsvNode('192.168.19.132',7001)
        while not rospy.is_shutdown:
            

if __name__ == '__main__':
    
   main()
  