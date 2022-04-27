#!/usr/bin/env python
import os
from contextlib import closing
from io import BytesIO

import boto3
import pyttsx3
import rospy
from playsound import playsound
from pydub import AudioSegment
from pydub.playback import play
from sp_msgs.srv import Speech, SpeechResponse
from std_srvs.srv import SetBool, SetBoolRequest


class SpeechSynthesizer(object):
    engine = pyttsx3.init()
    # secrets are stored in ~/.aws/credentials
    polly = boto3.Session().client("polly")
    buffer = {}

    def __init__(self):
        SpeechSynthesizer.engine.setProperty("volume", 1.0)
        SpeechSynthesizer.engine.setProperty("rate", 170)

    def speak_pyttsx3(self, text):
        SpeechSynthesizer.engine.stop()
        SpeechSynthesizer.engine.say(text)
        SpeechSynthesizer.engine.runAndWait()

    def speak_polly(self, text):
        response = SpeechSynthesizer.polly.synthesize_speech(
            Engine="neural",
            VoiceId="Joanna",
            OutputFormat="mp3",
            Text=text,
        )

        with closing(response["AudioStream"]) as stream:
            sound = AudioSegment.from_file(BytesIO(stream.read()), format="mp3")
        play(sound)

    def play_and_delete(self):
        playsound(self.out_path)
        os.remove(self.out_path)


def tts_server():
    rospy.init_node("tts_service_node")
    trigger_robot_lip_service = rospy.ServiceProxy("/robot_face/talk", SetBool)

    speaker = SpeechSynthesizer()

    def handle_tts(req):
        msg = SetBoolRequest()

        msg.data = True
        trigger_robot_lip_service(msg)

        # speaker.speak(req.data)
        rospy.loginfo(f"TTS recv: {req.data}")
        speaker.speak_polly(req.data)

        msg.data = False
        trigger_robot_lip_service(msg)
        return SpeechResponse(True, req.data)

    def handle_tts_machine(req):
        msg = SetBoolRequest()

        msg.data = True
        trigger_robot_lip_service(msg)
        rospy.loginfo(f"TTS recv: {req.data}")
        speaker.speak_pyttsx3(req.data)

        msg.data = False
        trigger_robot_lip_service(msg)
        return SpeechResponse(True, req.data)

    s = rospy.Service("/tts", Speech, handle_tts)

    s2 = rospy.Service("/tts_machine", Speech, handle_tts_machine)
    rospy.spin()


if __name__ == "__main__":
    tts_server()
