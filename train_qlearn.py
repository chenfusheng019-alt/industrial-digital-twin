import numpy as np
import rospy
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from arm2d_real_env import Arm2DRealEnv
from object_sorting.srv import SetTarget
from std_msgs.msg import Float32, Float32MultiArray, Int32
import signal
import sys
import time

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    rospy.signal_shutdown('Keyboard interrupt')
    sys.exit(0)

def set_target_color(colors=['red'], tags=[]):
    rospy.wait_for_service('/object_sorting/set_target')
    try:
        set_target = rospy.ServiceProxy('/object_sorting/set_target', SetTarget)
        resp = set_target(colors, tags)
        print(f"Set target color: {colors}, resp: {resp}")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def main():
    try:
        print("Initializing ROS node...")
        rospy.init_node('arm2d_qlearn', anonymous=True)
        
        rospy.set_param('/rosout/queue_size', 10)  
        
        print("Setting up signal handler...")
        signal.signal(signal.SIGINT, signal_handler)
        
        print("Creating publisher...")
        joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', 
                                   MultiRawIdPosDur, queue_size=1)
        
        print("Waiting for publisher to initialize...")
        rospy.sleep(1)
        
        print("Setting target color...")
        set_target_color(['red'])
        

        print("Resetting training data files...")
        try:

            #with open('training_data.csv', 'w') as f:
                #f.write('step,state1,state2,state3,state4,action,reward,done,episode,episode_step\n')
            

            #with open('episode_rewards.csv', 'w') as f:
                #f.write('episode,steps,total_reward,avg_reward\n')
                
            print("Training data files reset successfully")
        except Exception as e:
            print(f"Error resetting training data files: {e}")

        print("Creating environment...")
        env = Arm2DRealEnv(joints_pub)
        
        print("Waiting for MATLAB connection...")
        start_time = time.time()
        while env.rl_data_pub.get_num_connections() == 0:
            if time.time() - start_time > 10:
                print("Timeout waiting for MATLAB connection")
                return
            rospy.sleep(0.1)
        
        print("MATLAB connected!")
        
        state = env._get_state()
        data = Float32MultiArray()
        data.data = [
            float(state[0]), float(state[1]), float(state[2]), float(state[3]),  
            -1.0, 0.0,  
            float(env.done),  
            float(env.episode), float(env.episode_steps)  
        ] 
        env.rl_data_pub.publish(data)
        print("Initial state sent to MATLAB")
        
        print("Starting DQN training...")
        
        rate = rospy.Rate(5)  
        iteration = 0
        last_cleanup_time = time.time()
        last_action_time = time.time()
        action_timeout = 5.0  
        
        while not rospy.is_shutdown():
            try:
                iteration += 1
                current_time = time.time()
                
                if current_time - last_cleanup_time > 60:
                    import gc
                    gc.collect()
                    last_cleanup_time = current_time
                    print("Performed memory cleanup")
                
                if env.rl_data_pub.get_num_connections() == 0:
                    print("MATLAB disconnected, waiting for reconnection...")
                    while env.rl_data_pub.get_num_connections() == 0 and not rospy.is_shutdown():
                        rospy.sleep(0.1)
                    if not rospy.is_shutdown():
                        print("MATLAB reconnected!")
                        state = env._get_state()
                        data = Float32MultiArray()
                        data.data = [state[0], state[1], state[2], state[3], 0, 0]
                        env.rl_data_pub.publish(data)
                
                action_subscribers = env.action_sub.get_num_connections()
                if action_subscribers == 0:
                    print("No action subscribers, waiting...")
                    rospy.sleep(0.1)
                    continue
                
                if hasattr(env, 'last_action_time'):
                    if current_time - env.last_action_time > action_timeout:
                        print("Action timeout, resending current state...")
                        state = env._get_state()
                        data = Float32MultiArray()
                        data.data = [state[0], state[1], state[2], state[3], 0, 0]
                        env.rl_data_pub.publish(data)
                        env.last_action_time = current_time
                
                if iteration % 10 == 0:
                    print(f"\nIteration {iteration}")
                    print(f"Current state: {env._get_state()}")
                    print(f"Action subscribers: {action_subscribers}")
                    if hasattr(env, 'last_action'):
                        print(f"Last action: {env.last_action}")
                    if hasattr(env, 'last_action_time'):
                        print(f"Time since last action: {current_time - env.last_action_time:.2f}s")
                
                rate.sleep()
                
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                print(f"Error in main loop: {e}")
                rospy.logerr(f"Error in main loop: {e}")
                rospy.sleep(1)
                continue
            
    except Exception as e:
        print(f"Error in main: {e}")
        rospy.logerr(f"Error in main: {e}")
    finally:
        print("Cleaning up...")
        if 'env' in locals():
            del env
        gc.collect()
        print("Shutting down...")
        rospy.loginfo("Shutting down...")

if __name__ == '__main__':
    try:
        print("Starting main program...")
        main()
    except Exception as e:
        print(f"Fatal error: {e}")
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        print(f"Traceback: {traceback.format_exc()}")
    finally:
        print("Program terminated")
        rospy.signal_shutdown('Program terminated')
