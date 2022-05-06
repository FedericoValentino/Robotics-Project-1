
# Robotics Project 1
## General Project Structure
Our project is divided into 3 nodes, each of which, has a precise role.

 - **Velocity_Comp Node**: it subscribes to the wheel_states topic and publishes the robot linear and angular velocity in the cmd_vel topic;
 - **OdometryPublisher Node**: it subscribes to the previously published cmd_vel topic and publishes the robot odometry in the odom topic;
 - **WheelSpeed Node**: it subscribes to cmd_vel topic in order to recompute back the each of the robot wheel speed and publishes them over a custom message that is published on the wheels_rpm topic;

We also have included a reset service, for resetting the odometry at any given pose and a dynamic reconfigure, to change the integration method between Euler and Runge-Kutta.

### Velocity_Comp Node

This first node is built around a custom C++ class called VelocityPublisher.

    class VelocityPublisher
    {
    public:
      VelocityPublisher();
      void wheelsCallback(const sensor_msgs::JointState::ConstPtr& msg);
      void computeVelocities();
      void publishVelocities();
      double timeDiff(const ros::Time t1, const ros::Time t2);
    private:
      ros::NodeHandle n;
      ros::Subscriber sub;
      ros::Publisher pub;
      ros::Publisher pubTest;

      sensor_msgs::JointState msg1Data;
      sensor_msgs::JointState msg2Data;
      geometry_msgs::Vector3 velocity;
      geometry_msgs::Vector3 omega;
      int totalMessage;
    };

The class handles the subscribing to the Wheel_states topic and the publishing of the cmd_vel topic.
Everytime a message is received the wheelsCallback method is called, saving the current message either in msg1Data or in msg2Data; when at least 2 messages have been received the computeVelocities method is called and the actual computation can start.
We first obtain the each of the wheel velocities from the encoders using the formula:

$\dfrac{msg2Data.CurrentTicks - msg1Data.CurrentTicks}{msg2Data.CurrentTime-ms1Data.currentTime}\dfrac{1}{EncoderResolution} \dfrac{1}{GearRatio} 2\pi$

After obtaining the wheel velocities in $\dfrac{rad}{s}$ we compute $V_x$, $V_y$ and $\omega_z$ as follows:

$V_x  = (\omega_{fl} + \omega_{fr} + \omega_{rl} + \omega_{rr}) \cdot \frac{r}{4}$
$V_y  = (-\omega_{fl} + \omega_{fr} + \omega_{rl} - \omega_{rr}) \cdot \frac{r}{4}$
$\omega_z = (-\omega_{fl} + \omega_{fr} - \omega_{rl} + \omega_{rr}) \cdot \frac{r}{4(l_x + l_y)}$

All of that is then assembled into a TwistStamped message and published via the publishVelocities method.

As you can see the class doesn't quite end there. For example the pubTest publisher is used to publish the computed wheel speeds in $RPM$ in order to confront them with the results obtained by the **Wheel_speed** node.
The totalMessage integer is just used to count up the number of messages received by the node.

### OdometryPublisher node
This node is represented by a C++ class named `OdometryPublisher` that has the following interface.
```
enum class IntegrationMethod {
    EULER, RUNGE_KUTTA
};

class OdometryPublisher {
public:
  OdometryPublisher(); 
  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void eulerOdometry();
  void rungeKuttaOdometry();
  void publishOdometry(const std_msgs::Header header);
  void broadcastTFOdometry(const std_msgs::Header header);
  bool resetOdometryCallback(project1::ResetOdometry::Request &req, project1::ResetOdometry::Response &res);
  void integrationMethodReconfigureCallback(project1::integrationParameterConfig &config, uint32_t level);

private:
  ros::NodeHandle n; 
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::ServiceServer service;
  tf2_ros::TransformBroadcaster transformBroadcaster;
  geometry_msgs::TransformStamped transformStamped;

  dynamic_reconfigure::Server<project1::integrationParameterConfig> dynServer;
  dynamic_reconfigure::Server<project1::integrationParameterConfig>::CallbackType f;

  double x_k;
  double y_k;
  double theta_k;
  ros::Time t_k;
  ros::Time t_k_new;
  double v_x_k;
  double v_y_k;
  double omega_k;
  IntegrationMethod integrationMethod;
  bool isInitialized;
};
```
The class subscribe the **/cmd_vel** topic and uses its content to compute the odometry, that will be published in the **/odom** topic. Method `velocityCallback` is called everytime a message from **/cmd_vel** is subscribed; this method updates the current velocities of the robot in the robot frame and the current time. Depending on the value of the *integrationMethod* attribute, method `velocityCallback` calls `eulerOdometry` (if the *integrationMethod* is **EULER**) or `rungeKuttaOdometry` (if the *integrationMethod* is **RUNGE_KUTTA**); these two methods update robot's odometry. 

To compute odometry, the linear velocities that are obtained from the **/cmd_vel** topic need to be transformed from the robot frame to the global frame:
$v_x = v_x_k \cdot \cos(theta_k) - v_y_k \cdot \sin(theta_k) </br>
$v_y = v_x_k \cdot \sin(theta_k) + v_y_k \cdot \cos(theta_k) </br>
where *v_x* and *v_y* are in the global frame and *v_x_k* and *v_y_k* are in the robot frame.

Method `eulerOdometry` uses Euler integration to compute odometry:</br>
$x_k = x_k + v_x \cdot Ts </br>
$y_k = y_k + v_y \cdot Ts </br>
$theta_k = theta_k + omega \cdot Ts </br>
where Ts is the interval between the last received message (*t_k*) and the new received message(*t_k_new*).

Method `rungeKuttaOdometry` uses Runge-Kutta integration to compute odometry: </br>
$x_k = x_k + v \cdot Ts \cdot \cos(theta_k + (omega * Ts / 2) </br>
$y_k = y_k + v \cdot Ts \cdot \sin(theta_k + (omega * Ts / 2) </br>
$theta_k = theta_k + omega \cdot Ts

After computing odometry, `publishOdometry` and `broadcastTFOdometry` methods are called: the first one publish robot's odometry in the **/odom** topic; the second one broadcast TF from frame **odom** to frame **base_link**.

Method `resetOdometryCallback` is the callback function of the ResetOdometry service that reset the robot pose to a given pose. Method `integrationMethodReconfigureCallback` is the server callback for the dynamic reconfigure of the *integrationMethod* attribute; by default, the nose initialize the *integrationMethod* attribute to compute odometry with Runge-Kutta integration

### WheelSpeed node
This node is concerned to test the correctness of the computed speed published by the **computing** node. It subscribes to the topic **/cmd_vel**, then it calls the *publishSpeed* callback function which computes starting from the robot's linear and angular speeds each of the single wheel speeds, by using the following formula:

$\small \begin{bmatrix} v_{l1}\\ v_{r1}\\v_{r2}\\v_{l2} \end{bmatrix}$ = $\small \dfrac{1}{r}$$\small \begin{bmatrix}-l-w&&1&&-1\\l+w&&1&&1\\l+w&&1&&-1\\-l-w&&1&&1\end{bmatrix}$ $\small \begin{bmatrix} \omega \\ v_{x} \\ v_{y} \end{bmatrix}$

Notice that the result is given in $\scriptsize \dfrac{rad}{s}$ , so every single value is corrected with a conversion factor ($\scriptsize \bold{CONV\_FACTOR}$) to transform them into $\small rpm$. The results are then published into a custom message *WheelSpeed.msg*, on the topic **/wheels_rpm**.

## Services
We implemented a service called `ResetOdometry` that resets the robot pose to any given pose. This service is described by the **ResetOdometry.srv** file: the Request of the service specify the new pose and the new time; the Response specify the old pose and the old time.
To call the `ResetOdometry` service: </br>
`rosservice call /reset param1 param2 param3`</br>
where *param1* is the new *x*, *param2* is the new *y* and *param3* is the new *theta*.

## Dynamic Reconfigure
We used also dynamic reconfigure to reconfigure the integration method for computing the robot odometry. To call the dynamic reconfigure:</br>
`rosrun rqt_reconfigure rqt_reconfigure`</br>
the name of the node on which the parameter is recofigured is odometry_publisher; to set Euler as the integration method insert 0, to set Runge-Kutta insert 1.

This can be done also with the command line as </br>
`rosrun dynamic_reconfigure dynparam set /odometry_publisher integrationMethod param`</br>
where *param* can be 0 or 1.

## Parameters Calibration
We prefered to calibrate our r, l, w and N values manually rather than making our nodes to it automatically. For each parameter we have applied a different approach.

### Encoder Resolution(N)
What we have done to get the perfect Encoder Resolution is run the Bag1.bag data through a Perceptron. We first started by getting the data out of the bag by using our first node, Velocity_comp(you can see the code lines we used to do it in the node source code, commented out). One file contained the raw wheel speeds in $\scriptsize \dfrac{rad}{s}$, the other one contained our wheel speeds, not yet multiplied by $\scriptsize \dfrac{1}{EncoderResolution}$. What we fed the neuron were our speeds and what we got out of it after some training were our raw wheel speeds. The Perceptron guessed after many inputs the best Encoder Resolution to use, which was 39,8475. The code follows:

    #include <stdio.h>
    #include<iostream>
    #include<stdlib.h>


    class Perceptron
    {
    public:
      double input;
      double weight;
      double output;
      double actualOutput;
      double error;
      double adjustment;
      double learningFactor;
      double bias;


      Perceptron()
      {
        weight = 1/39.89;
        learningFactor = 0.01;
        bias = 1;
      }

      void CycleData(double x, double z)
      {
        input = x;
        actualOutput = z;
        output = input * weight + bias;
        error = actualOutput - output;
        if(abs(error) > 0.001)
        {
          adjustment = error / input * learningFactor;
          weight += adjustment;
          bias += error * learningFactor;
        }
      }
    };


    int main()
    {
      FILE* trainingData = fopen("outputObtained.txt", "r");
      FILE* realData = fopen("outputOriginal.txt", "r");
      Perceptron p;
      while(!feof(realData))
      {
        double x;
        double z;
        fscanf(trainingData, "%lf", &x);
        fscanf(realData, "%lf", &z);
        if(x != 0)
        {
          p.CycleData(x, z);
          std::cout<<1/p.weight<<std::endl;
        }

      }
    }

### Wheel radius R

### Length (l) and Width (w)
