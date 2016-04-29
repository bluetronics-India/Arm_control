##Arm control

- **serial通訊**
使用SerialDataGateway進行通訊，在開啟程式時記得調整Serial port ，預設值為 ttyACM0，藉由接收std_msgs/String 文字指令，使機械手臂進行initialize、poke、grab、putback的動作
- **軌跡規劃**
利用calMultiPointTrajectory，藉由給定的loop rate, linear speed limitation, angular speed limitation，線性內插出六軸的軌跡
- **動作定義**
利用motionGen，對四個預設的位置進行簡易的修改，產生中途點(waypoint)，這四個中途點是利用Arm_IK計算出來
- **任務執行**
利用motionGen產生中途點，將各個中途點丟入calMultiPointTrajectory，產生整體任務的軌跡，藉由cvt2SerialData將其轉換成byte資訊，利用publishArmCmd將資料傳給Arduino
