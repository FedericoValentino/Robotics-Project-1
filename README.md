# Robotics Project 1
## General Project Structure
Our project is divided into 3 nodes, each of which, has a precise role.

 - **Velocity_Comp Node**: it subscribes to the wheel_states topic and publishes the robot linear and angular velocity in the cmd_vel topic;
 - **OdometryPublisher Node**: it subscribes to the previously published cmd_vel topic and publishes the robot odometry in the odom topic;
 - **Wheel_speed Node**: it subscribes to cmd_vel topic in order to recompute back the each of the robot wheel speed and publishes them over a custom message that is published on the wheels_rpm topic;

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

### Wheel_Speed node
This node is concerned to test the correctness of the computed speed published by the computing node. It subscribes to the topic **/cmd_vel**, then it calls the *publishSpeed* callback function which computes starting from the robot's linear and angular speeds each of the single wheel speeds. Notice that every single value is corrected with a conversion factor (**CONV_FACTOR**) to pass from $\dfrac{rad}{s}$ to $rpm$. Then it publish the results into a custom message *WheelSpeed.msg*, on the topic **/wheels_rpm**.

## Services

## Parameters Calibration
We prefered to calibrate our r, l, w and N values manually rather than making our nodes to it automatically. For each parameter we have applied a different approach.

### Encoder Resolution(N)
What we have done to get the perfect Encoder Resolution is run the Bag1.bag data through a Perceptron. We first started by getting the data out of the bag by using our first node, Velocity_comp(you can see the code lines we used to do it in the node source code, commented out). One file contained the raw wheel speeds in $\dfrac{rad}{s}$, the other one contained our wheel speeds, not yet multiplied by $\dfrac{1}{EncoderResolution}$. What we fed the neuron were our speeds and what we got out of it after some training were our raw wheel speeds. The Perceptron guessed after many inputs the best Encoder Resolution to use, which was 39,8475. The code follows:

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
