import rospy
import numpy as np
import tflite_runtime.interpreter as tflite
import os
import threading
import traceback
import tempfile
import json
from hiwonder_servo_controllers import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from std_msgs.msg import Float32, Float32MultiArray, Int32


SERVO_LIMITS = {3: (0, 1000), 4: (0, 1000), 5: (0, 1000)}
STEP_SIZE = 10

class Arm2DRealEnv:
    def __init__(self, joints_pub):
        try:
            rospy.loginfo("Initializing Arm2DRealEnv...")
            self.joints_pub = joints_pub
            self.initial_pos = {3: 80, 4: 825, 5: 625}
            self.servo_pos = self.initial_pos.copy()
            self.action_space = 6
            self.state_dim = 4
            self.steps = 0
            self.episode = 0
            self.episode_steps = 0
            self.max_episode_steps = 50
            self.done = False
            self.distance = 1.0
            
            self.target_update_counter = 0
            self.target_update_frequency = 10
            self.last_target_update = 0
            
            self.update_lock = threading.Lock()
            
            rospy.loginfo("Setting up ROS publishers and subscribers...")
            self.rl_data_pub = rospy.Publisher('/rl_data', Float32MultiArray, queue_size=1)
            self.action_sub = rospy.Subscriber('/rl_action', Int32, self._action_callback)
            self.network_update_sub = rospy.Subscriber('/network_update', Float32MultiArray, 
                                                     self._update_network_callback)
            self.distance_sub = rospy.Subscriber('/object_sorting/target_distance', Float32, 
                                               self._distance_callback)
            
            rospy.loginfo("Loading initial TFLite model...")
            self.model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'target_network.tflite')
            self.interpreter = None
            self._load_tflite_model()
            
            rospy.loginfo("Waiting for MATLAB connection...")
            while self.rl_data_pub.get_num_connections() == 0 and not rospy.is_shutdown():
                rospy.sleep(0.1)
            rospy.loginfo("MATLAB connected!")
            
            self.reset()
            rospy.loginfo("Arm2DRealEnv initialization completed")
            
        except Exception as e:
            rospy.logerr(f"Error in Arm2DRealEnv initialization: {e}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
            raise

    def __del__(self):
        try:
            rospy.loginfo("Cleaning up resources...")
            if hasattr(self, 'interpreter'):
                del self.interpreter
            rospy.loginfo("Cleanup completed")
        except Exception as e:
            rospy.logerr(f"Error during cleanup: {e}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")

    def _action_callback(self, msg):
        try:
            action = msg.data
            rospy.loginfo(f"Received action from MATLAB: {action}")
            self._execute_action(action)
        except Exception as e:
            rospy.logerr(f"Error in action callback: {e}")

    def _update_network_callback(self, msg):
        try:
            rospy.loginfo("Received network update request")
            
            if not hasattr(self, 'interpreter') or self.interpreter is None:
                rospy.logerr("Interpreter not initialized")
                return
            
            self.target_update_counter += 1
            if self.target_update_counter < self.target_update_frequency:
                rospy.loginfo(f"Skipping update. Counter: {self.target_update_counter}/{self.target_update_frequency}")
                return
            
            rospy.loginfo("Starting network update...")
            
            weights = np.array(msg.data, dtype=np.float32)
            if len(weights) != 870:
                raise ValueError(f"Weight array length mismatch. Expected 870, got {len(weights)}")
            
            rospy.loginfo("Weights received and validated")
            
            with self.update_lock:
                try:
                    rospy.loginfo("Creating temporary model file...")
                    
                    with tempfile.NamedTemporaryFile(suffix='.tflite', delete=False) as temp_file:
                        temp_path = temp_file.name
                    
                    with open(self.model_path, 'rb') as src, open(temp_path, 'wb') as dst:
                        dst.write(src.read())
                    
                    rospy.loginfo("Temporary model file created")
                    
                    old_model_path = self.model_path
                    self.model_path = temp_path
                    
                    try:
                        self._load_tflite_model()
                        rospy.loginfo("Model reloaded successfully")
                        
                        self.target_update_counter = 0
                        self.last_target_update = self.steps
                        
                        if os.path.exists(old_model_path):
                            os.remove(old_model_path)
                        
                        os.rename(temp_path, old_model_path)
                        self.model_path = old_model_path
                        
                        rospy.loginfo("Network update completed successfully")
                        
                    except Exception as e:
                        rospy.logerr(f"Error during model reload: {e}")
                        self.model_path = old_model_path
                        if os.path.exists(temp_path):
                            os.remove(temp_path)
                        self._load_tflite_model()
                        raise
                    
                except Exception as e:
                    rospy.logerr(f"Error during network update: {e}")
                    rospy.logerr(f"Traceback: {traceback.format_exc()}")
                    raise
            
        except Exception as e:
            rospy.logerr(f"Failed to process network update: {e}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")

    def _execute_action(self, action):
        try:
            if action == -1:
                self.reset()
                return

            assert 0 <= action < 6
            servo_id = 3 + (action // 2)
            direction = 1 if action % 2 == 0 else -1
            
            with self.update_lock:
                self.servo_pos[servo_id] += direction * STEP_SIZE
                self.servo_pos[servo_id] = np.clip(self.servo_pos[servo_id], 
                                                 SERVO_LIMITS[servo_id][0], 
                                                 SERVO_LIMITS[servo_id][1])
            
            self._send_servo_cmd()
            rospy.sleep(0.5)
            
            self.steps += 1
            self.episode_steps += 1
            
            state = self._get_state()
            reward = self._get_reward()
            
            self.done = (self.episode_steps >= self.max_episode_steps) or (self.distance < 0.02)
            
            data = Float32MultiArray()
            data.data = [
                float(state[0]), float(state[1]), float(state[2]), float(state[3]),  
                float(action), float(reward), 
                float(self.done), 
                float(self.episode), float(self.episode_steps)  
            ]
            self.rl_data_pub.publish(data)
            
            rospy.loginfo(f"Action executed: {action}, State: {state}, Reward: {reward}, "
                         f"Episode: {self.episode}, Step: {self.episode_steps}, Done: {self.done}")
            
            if self.done:
                rospy.loginfo(f"Episode {self.episode} finished after {self.episode_steps} steps")
                self.episode += 1 
                self.reset()
            
        except Exception as e:
            rospy.logerr(f"Error in execute_action: {e}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")

    def _send_servo_cmd(self):
        bus_servo_control.set_servos(
            self.joints_pub, 0.5,
            ((3, int(self.servo_pos[3])), (4, int(self.servo_pos[4])), (5, int(self.servo_pos[5])))
        )

    def _get_state(self):
        try:
            state = np.array([
                float(self.servo_pos[3]),  
                float(self.servo_pos[4]),
                float(self.servo_pos[5]),
                float(self.distance)
            ], dtype=np.float32)
            return state
        except Exception as e:
            rospy.logerr(f"Error in _get_state: {e}")
            return np.zeros(4, dtype=np.float32)  
    def _get_reward(self):
        state = self._get_state()
        dist = state[3]   
        
        base_reward = -dist * 5
        
        step_penalty = -0.05  
        
        if hasattr(self, 'last_distance'):
            dist_improvement = self.last_distance - dist
            improvement_reward = dist_improvement * 20  
        else:
            improvement_reward = 0
        self.last_distance = dist
        
        success_reward = 0
        if dist < 0.085:  
            success_reward = 15  
        elif dist < 0.1:  
            success_reward = 5   
        
        total_reward = base_reward + step_penalty + improvement_reward + success_reward
        
        if self.episode_steps % 10 == 0:  
            rospy.loginfo(f"Reward components - Base: {base_reward:.2f}, "
                         f"Step penalty: {step_penalty:.2f}, "
                         f"Improvement: {improvement_reward:.2f}, "
                         f"Success: {success_reward:.2f}, "
                         f"Total: {total_reward:.2f}")
        
        return total_reward
    def _distance_callback(self, msg):
        try:
            self.distance = msg.data
            rospy.loginfo(f"Received distance: {self.distance}")
        except Exception as e:
            rospy.logerr(f"Error in distance callback: {e}")

    def _send_initial_state(self):
        try:
            state = self._get_state()
            data = Float32MultiArray()
            data.data = [
                float(state[0]), float(state[1]), float(state[2]), float(state[3]),  
                -1.0, 0.0,  
                float(self.done), 
                float(self.episode), float(self.episode_steps)  
            ] 
            self.rl_data_pub.publish(data)
            rospy.loginfo("Initial state sent to MATLAB")
        except Exception as e:
            rospy.logerr(f"Error sending initial state: {e}")

    def _load_tflite_model(self):
        try:
            if not os.path.exists(self.model_path):
                raise FileNotFoundError(f"Model file not found: {self.model_path}")
            
            rospy.loginfo(f"Loading model from: {self.model_path}")
            
            if self.interpreter is not None:
                del self.interpreter
                self.interpreter = None
            
            self.interpreter = tflite.Interpreter(
                model_path=self.model_path,
                num_threads=1
            )
            
            self.interpreter.allocate_tensors()
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            
            test_input = np.zeros((1, 4), dtype=np.float32)
            self.interpreter.set_tensor(self.input_details[0]['index'], test_input)
            self.interpreter.invoke()
            test_output = self.interpreter.get_tensor(self.output_details[0]['index'])
            
            if test_output.shape != (1, 6):
                raise RuntimeError(f"Invalid output shape: {test_output.shape}")
            
            rospy.loginfo("Model loaded and verified successfully")
            
        except Exception as e:
            rospy.logerr(f"Error loading TFLite model: {e}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
            if self.interpreter is not None:
                del self.interpreter
                self.interpreter = None
            raise

    def reset(self):
        try:
            if not hasattr(self, 'episode') or self.episode == 0:
                self.episode = 1
                
            rospy.loginfo(f"Resetting environment for episode {self.episode}")
            
            self.servo_pos = self.initial_pos.copy()
            self._send_servo_cmd()
            rospy.sleep(1.0)
            
            self.episode_steps = 0
            self.done = False
            self.distance = 1.0
            
            state = self._get_state()
            
            data = Float32MultiArray()
            data.data = [
                float(state[0]), float(state[1]), float(state[2]), float(state[3]),  
                -1.0, 0.0,  
                float(self.done), 
                float(self.episode), float(self.episode_steps)  
            ]
            self.rl_data_pub.publish(data)
            
            rospy.loginfo(f"Environment reset complete. New episode {self.episode} started")
            return state
            
        except Exception as e:
            rospy.logerr(f"Error in reset: {e}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
            return self._get_state()

if __name__ == '__main__':
    rospy.init_node('arm2d_real_env_test')
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    env = Arm2DRealEnv(joints_pub)
    rospy.spin()
