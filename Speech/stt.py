#!/usr/bin/env python3
import speech_recognition as sr

recognizer = sr.Recognizer()

with sr.Microphone() as source:
    print("พูดได้เลย...")
   
    recognizer.adjust_for_ambient_noise(source)
    
    audio = recognizer.listen(source)

    try:
        
        text = recognizer.recognize_google(audio, language="th-TH")  
        print(f"คุณพูดว่า: {text}")
    except sr.UnknownValueError:
        print("ไม่สามารถเข้าใจเสียงของคุณได้")
    except sr.RequestError:
        print("ไม่สามารถเชื่อมต่อกับบริการ Google ได้")

