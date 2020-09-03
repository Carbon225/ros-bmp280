#include <ros/ros.h>
#include <ros_bmp280/Barometer.h>
#include <raspberry_bmp280.h>

static double p0 = -1.;
static bool calibrating = true;
static int calibratingStep = 0;

static void calibrate(double p)
{
    if (calibratingStep == 0)
    {
        ROS_INFO("Calibration started");
        p0 = -1.;
    }
    else if (calibratingStep == 50)
    {
        p0 = p;
    }
    else if (calibratingStep > 50 && calibratingStep < 100)
    {
        p0 += 0.15 * (p - p0);
    }
    else if (calibratingStep == 100)
    {
        ROS_INFO("Calibration done. p0 = %.1f Pa", p0);
        calibratingStep = -1;
        calibrating = false;
    }

    calibratingStep++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bmp280_node");
    ros::NodeHandle nh;

    ros::Publisher bmpPub = nh.advertise<ros_bmp280::Barometer>("baro", 16);

    ROS_INFO("BMP node starting");

    BMP280 bmp;
    int8_t ret = 0;
    if ((ret = bmp.init(BMP280::CSBState::High, BMP280_FILTER_COEFF_16, BMP280_OS_16X, BMP280_OS_2X, BMP280_ODR_62_5_MS)) != 0)
    {
        ROS_FATAL("BMP init error: %d", ret);
        return ret;
    }

    ros::Rate rate(1000. / 62.5);
    while ((ret = bmp.read()) == 0 && ros::ok())
    {
        ros::spinOnce();

        if (calibrating)
            calibrate(bmp.getPressure());

        ros_bmp280::Barometer msg{};
        msg.header.stamp = ros::Time::now();

        msg.altitude = p0 < 0. ? 0. : bmp.getAltitude(p0);
        msg.pressure = bmp.getPressure();
        msg.temperature = bmp.getTemperature();

        bmpPub.publish(msg);

        rate.sleep();
    }

    ROS_WARN("BMP node terminating");

    if (ret != 0)
        ROS_FATAL("BMP read error: %d", ret);

    return ret;
}