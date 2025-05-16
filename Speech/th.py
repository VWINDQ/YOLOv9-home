#!/usr/bin/env python3

import speech_recognition as sr
from gtts import gTTS
import os
import time

def speak(text):
    tts = gTTS(text=text, lang='th')
    tts.save('/tmp/temp.mp3')
    os.system('mpg321 /tmp/temp.mp3')

def wait_for_response():
    # Use speech recognition to wait for a voice response in Thai
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("กำลังฟัง...")  # เพียงแค่แสดงข้อความในคอนโซลแทนการพูด
        audio = recognizer.listen(source)

    try:
        # Recognize speech in Thai
        response = recognizer.recognize_google(audio, language="th-TH")
        print(f"Response: {response}")
        if "ใช่" in response:
            return "yes"
        elif "ไม่" in response:
            return "no"
        else:
            speak("ไม่เข้าใจ กรุณาลองอีกครั้ง")
            return wait_for_response()  # Try again if response is unclear
    except sr.UnknownValueError:
        speak("ขอโทษค่ะ ฟังไม่ชัดเจน กรุณาลองอีกครั้ง")
        return wait_for_response()

def main():
    speak("สวัสดีค่ะ คุณชื่อสมชายใช่หรือไม่")
    response = wait_for_response()
    
    if response == "yes":
        speak("คุณต้องการให้ถือกระเป๋าให้ไหม")
        response = wait_for_response()
        if response == "yes":
            speak("กรุณายื่นของให้ฉันหน่อย")
        else:
            speak("ขอบคุณค่ะ กรุณาเดินตามฉันมา")
    else:
        speak("ขอโทษค่ะ อาจจะเป็นการเข้าใจผิด")

if __name__ == '__main__':
    main()

