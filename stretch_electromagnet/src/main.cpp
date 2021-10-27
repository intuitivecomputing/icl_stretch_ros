/*
 * rosserial Service Server
 */

#include <ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

ros::NodeHandle nh;
using std_srvs::SetBool;
using std_srvs::Trigger;

int PIN = 3;
void callback(const SetBool::Request &req, SetBool::Response &res)
{
  if (req.data)
  {
    digitalWrite(PIN, HIGH);
    res.message = "magnet ON";
  }
  else
  {
    digitalWrite(PIN, LOW);
    res.message = "magnet OFF";
  }
  res.success = true;
}

void triggerCallback(const Trigger::Request &req, Trigger::Response &res)
{
  digitalWrite(PIN, digitalRead(PIN) == HIGH ? LOW : HIGH);
  res.success = true;
}

ros::ServiceServer<SetBool::Request, SetBool::Response> server("magnet", &callback);
ros::ServiceServer<Trigger::Request, Trigger::Response> server_trigger("magnet_toggle", &triggerCallback);


void setup()
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN, LOW);

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertiseService(server);
  nh.advertiseService(server_trigger);
}

void loop()
{
  nh.spinOnce();
  delay(10);
}
