import time

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        
        self.kp = kp if kp[1] <= kp[0] <= kp[2] else (kp[1] if kp[0] < kp[1] else kp[2])
        self.ki = ki if ki[1] <= ki[0] <= ki[2] else (ki[1] if ki[0] < ki[1] else ki[2])
        self.kd = kd if kd[1] <= kd[0] <= kd[2] else (kd[1] if kd[0] < kd[1] else kd[2])

        self.setpoint = setpoint
        
        self.prev_error = 0
        self.integral = 0
        
    def update(self, feedback, controller_sampling_time):
        error = self.setpoint - feedback
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral += error * controller_sampling_time 
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = (self.kd * (error - self.prev_error)) / controller_sampling_time 
        
        # PID output
        controller_output = proportional + integral + derivative
        
        self.prev_error = error
        
        return controller_output

# Example usage
if __name__ == "__main__":
    setpoint = 50.0
    feedback = 0.0
    controller_sampling_time = 0.05
    process_time_constant = 0.1 # should be chosen based on the dynamics of the system
                                # this need not be same as sampling time of the controller
    # state the values of the follwing in the format [value, lower limit, upper limit]
    kp = [1.0, -100.0, 100.0]
    ki = [0.001, -0.1, 0.1]
    kd = [0.00001, -0.1, 0.1]    
    
    pid = PIDController(kp, ki, kd, setpoint)  # Initialize PID controller with parameters

    while True:
        control_signal = pid.update(feedback, controller_sampling_time)
        
        # Simulate a process (just a simple integration)
        feedback += control_signal * process_time_constant  # Simulated process time constant is 0.1
        
        print("Control signal:", control_signal, "Feedback:", feedback)
        
        time.sleep(controller_sampling_time)  # Sample time delay
