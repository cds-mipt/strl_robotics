import rospy
from sensor_msgs.msg import Joy
class dh4():
    x_btn      = "x"
    crcl_btn   = "o"
    sqr_btn    ="[]"
    trngl_btn  = "^"
    def __init__(self):
        self.btn_decoder = {self.x_btn: 0, self.crcl_btn: 1, self.sqr_btn: 3, self.trngl_btn:2}
        self.btns = {self.x_btn:0, self.crcl_btn: 0, self.sqr_btn: 0, self.trngl_btn: 0}
        self.triggers = {self.x_btn: None, self.crcl_btn: None, self.trngl_btn : None, self.sqr_btn : None}
        self.sub = rospy.Subscriber("/joy_teleop/joy", Joy, self._joy_callback, queue_size=1)
    
    def _joy_callback(self, joy_data):
        for k in self.btns.keys():
            if (not self.triggers[k] is None) and (self.btns[k] == 0 and joy_data.buttons[self.btn_decoder[k]]==1):
                self.triggers[k]()

        self.btns[self.x_btn]  = joy_data.buttons[0]
        self.btns[self.crcl_btn]  = joy_data.buttons[1]
        self.btns[self.sqr_btn] = joy_data.buttons[3] 
        self.btns[self.trngl_btn]  = joy_data.buttons[2]

    def bind_trigger(self, btn, f):
        self.triggers[btn] = f  
