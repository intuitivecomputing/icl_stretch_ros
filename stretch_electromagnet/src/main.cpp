/*
 * rosserial Service Server
 */

#include <ros.h>
#include <std_srvs/SetBool.h>

ros::NodeHandle nh;
using std_srvs::SetBool;

int pin = 3;
void callback(const SetBool::Request &req, SetBool::Response &res)
{
  if (req.data)
  {
    digitalWrite(pin, HIGH);
    res.message = "magnet ON";
  }
  else
  {
    digitalWrite(pin, LOW);
    res.message = "magnet OFF";
  }
  res.success = true;
}

ros::ServiceServer<SetBool::Request, SetBool::Response> server("magnet", &callback);

void setup()
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertiseService(server);
}

void loop()
{
  nh.spinOnce();
  delay(10);
}
