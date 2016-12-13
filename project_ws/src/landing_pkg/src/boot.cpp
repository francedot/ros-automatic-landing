#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "boot");

    sleep(10);
    int status = system("rosservice call /ardrone/togglecam");
    // oppure
    // execlp("boot.sh", NULL);

    // TODO si può rimuovere?
    ros::spin();

    return 0;
}