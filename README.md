## CARLA Simulator Ubuntu 22.04 + NICE DCV + GPU

This [AWS CloudFormation](https://aws.amazon.com/cloudformation/) template will deploy [CARLA Simulator](https://carla.org/) into an accelerated computing instance running the [NICE DCV](https://aws.amazon.com/hpc/dcv/) server.

![CARLA Animation](/images/carla-ubuntu-2204.gif "CARLA Animation")

### CARLA

CARLA is an open-source simulator for autonomous driving research to support the development, training, and validation of autonomous driving systems.

### NICE DCV

![Architecture](/images/arch.png "Architecture")

NICE DCV is a high-performance remote display protocol that provides customers with a secure way to deliver remote desktops and application streaming from any cloud or data center to any device over varying network conditions.
For security reasons we do not put the EC2 instance in a public subnet, therefore you need to use the AWS systems manager with the [Session Manager plugin](https://docs.aws.amazon.com/systems-manager/latest/userguide/install-plugin-debian-and-ubuntu.html) to tunnel the DCV port to your local machine.
```
aws ssm start-session --target i-<instance id> --document-name AWS-StartPortForwardingSession --parameters '{"portNumber":["8443"], "localPortNumber":["8443"]}'
```
Then setup the NICE DCV client to connect to localhost:8443 and set Connection Setting to be WebSockets/TCP instead of QUIC.

To use the simplified architecture without Transit Gateway (using `template-unicast.yml`), follow these steps:

1. Be aware that this setup does not support UDP multicasting.
2. Configure both the device simulator and CARLA for unicast when using ROS2.
3. Find the example configuration for `cyclonedds` at: `carla-client/ros2/cyclonedds_unicast.yaml`.
4. Before running the ROS2 example, set the `CYCLONEDDS_URI` environment variable with this command:
   ```
   export CYCLONEDDS_URI=carla-client/ros2/cyclonedds_unicast.yaml
   ```

### Deployment instructions

Download desired template file and login into the AWS [CloudFormation console](https://console.aws.amazon.com/cloudformation/home#/stacks/create/template). Choose **Create Stack** and **Upload** the template file **"template.yml".**

Specify a **Stack name** and specify parameter values. All fields are required.

- **imageId** : [System Manager Parameter](https://aws.amazon.com/blogs/compute/using-system-manager-parameter-as-an-alias-for-ami-id/) path to AMI ID.
- **instanceType:** appropriate [instance type](https://docs.aws.amazon.com/AWSEC2/latest/UserGuide/instance-types.html).
- **ec2Name:** name of EC2 instance
- **vpcID:** [VPC](https://docs.aws.amazon.com/vpc/latest/userguide/what-is-amazon-vpc.html) with internet connectivity.
- **subnetID:** subnet with internet connectivity.
- **ingressIPv4:** allowed IPv4 source prefix to NICE DCV listening ports at 8443. Get source IP from [https://checkip.amazonaws.com](https://checkip.amazonaws.com/)

![CloudFormation parameters](/images/cloudformation-parameters.png "Parameters")

Continue **Next** with [Configure stack options](https://docs.aws.amazon.com/AWSCloudFormation/latest/UserGuide/cfn-console-add-tags.html), [Review](https://docs.aws.amazon.com/AWSCloudFormation/latest/UserGuide/cfn-using-console-create-stack-review.html) settings, and click **Create Stack** to launch your stack.

It may take up to 60 minutes to provision the EC2 instance. After your stack has been successfully created, its status changes to **CREATE\_COMPLETE**. Go to **the Outputs** tab.

![CloudFormation outputs](/images/cloudformation-outputs.png "Outputs")

Go to the **SSMSessionManager** row, open the URL (in the form _https://\<REGION\>.console.aws.amazon.com/systems-manager/session-manager/\<InstanceID\>_) in a new browser tab to log in via SSM Session Manager to change login user password. The password change command is " **sudo passwd ubuntu**".

Do start the
```
aws ssm start-session --target i-<instance id> --document-name AWS-StartPortForwardingSession --parameters '{"portNumber":["8443"], "localPortNumber":["8443"]}'
```
Use the local **DCVConsole** and log in as an ubuntu user with the recently changed password. Do not forget setting to be WebSockets/TCP instead of QUIC.

### Installing CARLA Simulator

Once logged in to the Ubuntu remote desktop, run the command "./install-carla" from your user's root directory. The installation will download everything needed to set up CARLA, change the default Python version to 3.8, and creates a virtual environment installing all package required. It may take 5 to 10 minutes to finish based on the large size of the downloaded binaries.

Then, you can test your CARLA installation by running "./CarlaUE4.sh" from "/opt/carla-simulator". You will see an image with the world simulation running successfully.

![CARLA Running](/images/carla-running.png "CARLA Running")

### Testing the PythonAPI

The best way to test the Python API is to drive a car. In order to do so, you will have to follow the steps below:

- Start the server "/opt/carla-simulator/CarlaUE4.sh -no-rendering -quality-level=Epic -prefernvidia" in different terminal

1. Activate the virtual environment created by the installation with the command ". venv/bin/activate" from the "ubuntu" user's home.
2. Navigate to the PythonAPI examples folder "cd /opt/carla-simulator/PythonAPI/examples".
3. Run "python manual\_control.py".

Here is the link to the complete [CARLA documentation](https://carla.readthedocs.io/en/0.9.13/)

![CARLA PythonAPI](/images/carla-manual-conrtrol.png "CARLA PythonAPI")

This AWS CloudFormation template has been made possible by using as a reference the [Amazon EC2 NICE DVC Samples](https://github.com/aws-samples/amazon-ec2-nice-dcv-samples).

## Security

See [CONTRIBUTING](CONTRIBUTING.md#security-issue-notifications) for more information.

## License

This library is licensed under the MIT-0 License. See the LICENSE file.
