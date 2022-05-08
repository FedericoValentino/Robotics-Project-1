10679810 - Federico Valentino
10660072 - Carlo Sgaravatti
10723028 - Francesco Scroccarello
  
# Robotics Project 1
## General Project Structure
Our project is divided into 3 nodes, each of which, has a precise role.

 - **Velocity_Comp Node**: it subscribes to the wheel_states topic and publishes the robot linear and angular velocity in the cmd_vel topic;
 - **OdometryPublisher Node**: it subscribes to the previously published cmd_vel topic and publishes the robot odometry in the odom topic;
 - **WheelSpeed Node**: it subscribes to cmd_vel topic in order to recompute back each of the robot wheel speed and publishes them over a custom message that is published on the wheels_rpm topic;

We also have included a reset service, for resetting the odometry at any given pose and a dynamic reconfigure, to change the integration method between Euler and Runge-Kutta.

To start all nodes (except the bag) insert `roslaunch project1 start.launch`; to play different bags multiple times you will have to restart all nodes with that command (this because the initial position of the robot needs to be resetted). In the launch file we have also included a static tranformation that align the *odom* frame and the *world* frame.


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
We first obtain each of the wheel velocities from the encoders using the formula:

![equation](https://user-images.githubusercontent.com/100689535/167306555-c5dfcc39-bc82-4703-8151-c5a1f54ca64e.png)

After obtaining the wheel velocities in ![rads](https://user-images.githubusercontent.com/100689535/167306953-7b720742-ba9f-4ef4-a68e-edb4fb28aac4.png) we compute ![vx](https://user-images.githubusercontent.com/100689535/167306957-2126cc47-9440-4dc4-a8ed-6d6d7a68b6e1.png), ![vy](https://user-images.githubusercontent.com/100689535/167306960-fb43d11e-7f63-4b86-bf0c-66ed2485a837.png)
 and ![omegaz](https://user-images.githubusercontent.com/100689535/167306963-429f64e4-9970-427e-802d-34eae0bcf9dd.png)
 as follows:

![equation(1)](https://user-images.githubusercontent.com/100689535/167306759-c679860e-4030-482c-9a8d-ba72a01fef2c.png)

![equation(2)](https://user-images.githubusercontent.com/100689535/167306834-865a17cc-0e8a-4442-ae7e-24f62ad643ab.png)

![equation(3)](https://user-images.githubusercontent.com/100689535/167306773-8ce7e28f-b786-49bc-81dc-f11e1d3f6c5a.png)


All of that is then assembled into a TwistStamped message and published via the publishVelocities method.

As you can see the class doesn't quite end there. For example the pubTest publisher is used to publish the computed wheel speeds in $\small rpm$ in order to confront them with the results obtained by the **Wheel_speed** node.
The totalMessage integer is just used to count the number of messages received by the node.

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
This class subscribes to the **/cmd_vel** topic and uses its content to compute the odometry, that will be published in the **/odom** topic. Method `velocityCallback` is called everytime a message from **/cmd_vel** is subscribed; this method updates the current velocities of the robot in the robot frame and the current time. Depending on the value of the *integrationMethod* attribute, method `velocityCallback` calls `eulerOdometry` (if the *integrationMethod* is **EULER**) or `rungeKuttaOdometry` (if the *integrationMethod* is **RUNGE_KUTTA**); these two methods update the robot's odometry.

To compute odometry, linear velocities that are obtained from the **/cmd_vel** topic need to be transformed from the robot frame to the global frame:</br>
![velocities](https://user-images.githubusercontent.com/58942793/167296040-053292db-4779-4c23-9af8-c53ae16613eb.png)</br>
where *v_x* and *v_y* are in the global frame and *v_x_k* and *v_y_k* are in the robot frame.

Method `eulerOdometry` uses Euler integration to compute odometry:</br>
![euler](https://user-images.githubusercontent.com/58942793/167296078-5b07f813-be12-4b65-a5ec-9fe6a9ccc041.png)</br>
where *Ts* is the interval between the last received message (*t_k*) and the newly received message(*t_k_new*).

Method `rungeKuttaOdometry` uses Runge-Kutta integration to compute odometry: </br>
![runge-kutta](https://user-images.githubusercontent.com/58942793/167296097-3644fd15-287e-45f6-97c6-412e97fe1765.png)</br>
where *v* is the module of the velocity and *alpha* is the velocity orientation, whith respect to the robot frame (so *omega + alpha* is the orientation of the velocity in the global frame).

After computing odometry, `publishOdometry` and `broadcastTFOdometry` methods are called: the first one publishes the robot's odometry in the **/odom** topic; the second one broadcasts TF from frame **odom** to frame **base_link**.

Method `resetOdometryCallback` is the callback function of the ResetOdometry service that resets the robot pose to any given pose. Method `integrationMethodReconfigureCallback` is the server callback for the dynamic reconfigure of the *integrationMethod* attribute; by default, the node initializes the *integrationMethod* attribute to compute odometry with Runge-Kutta integration.

### WheelSpeed node
This node is concerned to test the correctness of the computed speed published by the **computing** node. It subscribes to the topic **/cmd_vel**, then it calls the *publishSpeed* callback function which computes, starting from the robot's linear and angular speeds, each of the single wheel speeds, by using the following formula:

![Matrix](https://user-images.githubusercontent.com/100689535/167307259-9bf00148-9a42-4156-854a-67d94c2331f3.png)


Notice that the result is given in  ![rads](https://user-images.githubusercontent.com/100689535/167306953-7b720742-ba9f-4ef4-a68e-edb4fb28aac4.png), so every single value is corrected with a conversion factor (CONVFACTOR = 9.549297) to transform them into RPM. The results are then published into a custom message *WheelSpeed.msg*, on the topic **/wheels_rpm**.

## Structure of the TF tree
![tf_tree](https://user-images.githubusercontent.com/58942793/167296121-4e0e301c-90a8-4f86-b293-9e1b4a99363c.png)

## Services
We implemented a service called `ResetOdometry` that resets the robot pose to any given pose. This service is described by the **ResetOdometry.srv** file: the Request of the service specifies the new pose; the Response specifies the old pose.
To call the `ResetOdometry` service: </br>
`rosservice call /reset param1 param2 param3`</br>
where *param1* is the new *x*, *param2* is the new *y* and *param3* is the new *theta*.

We have also added a small node called `PoseResetter` that read the first data from the ground truth pose and reset the OdometryPublisher pose at that pose. This is to make sure that the robot starts from the correct position.

## Dynamic Reconfigure
We also used dynamic reconfigure to reconfigure the integration method for computing the robot odometry. To call the dynamic reconfigure:</br>
`rosrun rqt_reconfigure rqt_reconfigure`</br>
the name of the node on which the parameter is recofigured is odometry_publisher; to set Euler as the integration method insert 0, to set Runge-Kutta insert 1.

This can also be done with the command line as </br>
`rosrun dynamic_reconfigure dynparam set /odometry_publisher integrationMethod param`</br>
where *param* can be 0 or 1.

## Parameters Calibration
We preferred to calibrate our r, l, w and N values manually rather than making our nodes do it automatically. For each parameter we have applied a different approach.

### Encoder Resolution(N)
What we have done to get the perfect Encoder Resolution is run the Bag1.bag data through a Perceptron. We started by getting the data out of the bag by using our first node, Velocity_comp(you can see the code lines we used to do it in the node source code, commented out). One file contained the raw wheel speeds in $\scriptsize \dfrac{rad}{s}$, the other one contained our wheel speeds, not yet multiplied by $\scriptsize \dfrac{1}{EncoderResolution}$. What we fed the neuron were our computed speeds and what we got out of it after some training were the robot raw wheel speeds. The Perceptron guessed after many inputs the best Encoder Resolution to use, which was 39,45799077400557. The code follows:

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
          adjustment = error * input * learningFactor;
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
We tried different methods for the calibration of this parameter. At first we thought using again the perceptron wasn't an option and so we got, after manually trying some values, the radius to be equal to 0.0688, but after thinking about it more, we started trying to implement the perceptron once again. The main problem was the data to use it with. We decided to compare our computed pose against the /robot/pose topic. Problem was our OdometryPublisher node was publishing its messages at around 48 Hz, while the /robot/pose message from the bag was coming out at around 128hz, so our calculations were only one third of what they needed to be. After thinking about it we decided to only take the data that came out at the same timestamp from both topics. We put the data inside our perceptron, only to find out that the noise from the /robot/pose topic was interfering. We were getting results that ranged from 0.03 up to 0.09. Once we looked more into it, we saw that the perceptron was actually getting results around 0.069 but at the end diverging from that. We also tried to create a node, where its objective was to add or subtract some value from the radius and then publish it on a /radius topic, but that didn't work out either. At the end we decided to stick with R = 0.0688 since the results we were getting in rviz with that value were optimal.

### Length (L) and Width (W)
After calibrating the radius we saw that the only value we were still having problems with was $\omega$ and since L and W did take a part in that calculation, we decided to calibrate them manually and we got them down to be at around L = 0.192 and W = 0.161.
