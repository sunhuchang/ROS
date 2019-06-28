#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    #调用publisher函数 创建发布节点 ，定义数据类型，
    #queue_size参数是当其接受者接受不够快造成信号堵塞中队列信息的数量限制

    rospy.init_node('talker', anonymous=True)
    #启动节点同时为节点命名， 若anoymous为真则节点会自动补充名字，实际名字以talker_322345等表示
    #若为假，则系统不会补充名字，采用用户命名。但是一次只能有一个同名节点，若后面有一个相同listener
    #名字的节点则后面的节点启动会注销前面的相同节点名。

    rate = rospy.Rate(10) # 10hz
    #延时的时间变量赋值，通过rate.sleep()实现延时

    while not rospy.is_shutdown():
    # 判定开始方式，循环发送，以服务程序跳出为终止点 一般ctrl+c也可

        hello_str = "hello world %s" % rospy.get_time()
        # 数据变量的内容 rospy.get_time() 是指ros系统时间，精确到0.01s 
        # 也可以使用 import time  time.strftime('%Y%m%d%H%M%S')

        rospy.loginfo(hello_str)
        #在运行的terminal界面info 出信息，可不加，可随意改

        pub.publish(hello_str)
        #发布数据 必须发布

        rate.sleep()
        #ros中的延时表示，也可用系统的延时 import time  time.sleep(1) 
        #ros系统中的延时应该比ubuntu自带好好，尽量用ros的
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

