#!/usr/bin/env python
import os
import pickle
import subprocess
from distutils.command.config import config
from pathlib import Path
from threading import Thread

import rospy
from playsound import playsound
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
from sp_msgs.srv import Speech, SpeechResponse
from std_srvs.srv import SetBool, SetBoolRequest
from TTS.utils.manage import ModelManager
from TTS.utils.synthesizer import Synthesizer


def speak(text):
    from gtts import gTTS

    tts = gTTS(text=text, lang="en")

    filename = "tts_output.mp3"
    tts.save(filename)
    play_and_delete(filename)


def make_synthesizer():
    model_name = "tts_models/en/ljspeech/tacotron2-DDC"
    path = Path(__file__).parent / "../.models.json"
    manager = ModelManager(path)
    model_path, config_path, model_item = manager.download_model(model_name)
    vocoder_name = model_item["default_vocoder"]
    vocoder_path, vocoder_config_path, _ = manager.download_model(vocoder_name)
    # load models
    synthesizer = Synthesizer(
        model_path,
        config_path,
        tts_speakers_file=None,
        tts_languages_file=None,
        vocoder_checkpoint=vocoder_path,
        vocoder_config=vocoder_config_path,
        encoder_checkpoint=None,
        encoder_config=None,
        use_cuda=False,
    )
    return synthesizer


class SpeechSynthesizer(object):
    synthesizer = make_synthesizer()
    buffer = {}

    def __init__(self):
        self.out_path = "/home/hello-robot/catkin_ws/tts_output.wav"
        self.buffer_path = Path(__file__).parent / "../buffer.pickle"
        # self.voice_thread = Thread(target=self.play_and_delete)

        # load buffer
        if SpeechSynthesizer.buffer == {} and self.buffer_path.is_file():
            with open(self.buffer_path, "rb") as handle:
                SpeechSynthesizer.buffer = pickle.load(handle)
        # print(SpeechSynthesizer.buffer.keys())

    def __del__(self):
        with open(self.buffer_path, "wb") as handle:
            pickle.dump(
                SpeechSynthesizer.buffer,
                handle,
                protocol=pickle.HIGHEST_PROTOCOL,
            )

    def speak(self, text):
        # play = lambda: playsound(self.out_path)

        if text in self.buffer:
            wav = SpeechSynthesizer.buffer[text]
        else:
            wav = self.synthesizer.tts(text)
            SpeechSynthesizer.buffer[text] = wav
        self.synthesizer.save_wav(wav, self.out_path)
        playsound(self.out_path, block=True)
        # self.voice_thread.start()

    def play_and_delete(self):
        playsound(self.out_path)
        os.remove(self.out_path)


# def handle_tts(req):
#     speak(req.data)
#     return SpeechResponse(True, "")


def tts_server():
    rospy.init_node("tts_service_node")
    trigger_robot_lip_service = rospy.ServiceProxy("/robot_face/talk", SetBool)

    speaker = SpeechSynthesizer()

    def handle_tts(req):
        # speaker.speak(req.data)
        msg = SetBoolRequest()

        msg.data = True
        trigger_robot_lip_service(msg)

        speaker.speak(req.data)

        msg.data = False
        trigger_robot_lip_service(msg)
        return SpeechResponse(True, req.data)

    s = rospy.Service("/tts", Speech, handle_tts)
    soundhandle = SoundClient()

    def handle_tts_machine(req):
        voice = "voice_kal_diphone"
        volume = 10.0
        # msg = SetBoolRequest()
        # msg.data = True
        # trigger_robot_lip_service(msg)

        soundhandle.say(req.data, voice, volume)

        # msg.data = False
        # trigger_robot_lip_service(msg)
        return SpeechResponse(True, "")

    rospy.sleep(1)

    s2 = rospy.Service("/tts_machine", Speech, handle_tts_machine)
    rospy.spin()


if __name__ == "__main__":
    tts_server()
