#include <ros.h>
#include <std_msgs/String.h>
#include <Servo.h>

// Create Servo objects
Servo servo1;
Servo servo2;
Servo servo3;

ros::NodeHandle nh;
bool moving = false;

// Callback function to be called when a message is received
void messageCallback(const std_msgs::String& msg) {
    if (!moving) {
        moving = true;

        // Compare the received message content
        if (strcmp(msg.data, "A") == 0) {
            // Move servos to the positions for command "A"
            servo1.write(80);
            servo2.write(40);
            servo3.write(40);
        } else if (strcmp(msg.data, "B") == 0) {
            // Move servos to the positions for command "B"
            servo1.write(30);
            servo2.write(130);
            servo3.write(40);
            delay(3000);  // Wait 3 seconds for the servos to reach the positions

            // Move servo3 to 120 degrees after delay
            servo3.write(120);
            delay(3000);  // Wait 3 seconds for servo3 to reach 120 degrees

            // Move servos back to initial positions
            servo1.write(80);
            servo2.write(40);
            servo3.write(120);
            delay(500);  // Short delay to complete movement
        }

        moving = false;
    }
}

// Create a Subscriber object
ros::Subscriber<std_msgs::String> sub("Test", &messageCallback);

void setup() {
    nh.initNode();
    nh.subscribe(sub);
    

    // Attach servos to their respective pins
    servo1.attach(9);  // Pin for servo1
    servo2.attach(10); // Pin for servo2
    servo3.attach(11); // Pin for servo3

    // Initialize servos to their initial positions
    servo1.write(80);
    servo2.write(40);
    servo3.write(40);
}

void loop() {
    nh.spinOnce();
    delay(10);  // Small delay to keep the loop from running too fast
}
